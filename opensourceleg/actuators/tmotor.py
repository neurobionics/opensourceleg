import os
import time
import warnings
import csv
import traceback
from enum import Enum
from math import isfinite
from typing import Optional, Any, Dict, Union
from dataclasses import dataclass

import can
import numpy as np

from opensourceleg.actuators.base import (
    CONTROL_MODE_CONFIGS,
    CONTROL_MODES,
    MOTOR_CONSTANTS,
    ActuatorBase,
    ControlModeConfig,
    ControlGains,
)
from opensourceleg.actuators.decorators import (
    check_actuator_connection,
    check_actuator_open,
    check_actuator_stream,
)
from opensourceleg.math import ThermalModel
from opensourceleg.utilities import SoftRealtimeLoop
from opensourceleg.logging.logger import LOGGER


# TMotor servo mode constants
TMOTOR_SERVO_CONSTANTS = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=3600,  # 360 degrees * 10 (0.1 degree resolution)
    NM_PER_AMP=0.115,  # Approximate for AK80-9
    NM_PER_RAD_TO_K=0.0,  # Servo mode handles stiffness differently
    NM_S_PER_RAD_TO_B=0.0,  # Servo mode handles damping differently
    MAX_CASE_TEMPERATURE=80.0,
    MAX_WINDING_TEMPERATURE=110.0,
)


@dataclass
class ServoMotorParams:
    """Parameters for TMotor servo mode configuration."""
    P_min: float
    P_max: float
    V_min: float
    V_max: float
    Curr_min: float
    Curr_max: float
    T_min: float
    T_max: float
    Temp_min: float
    Temp_max: float
    Kt_actual: float
    GEAR_RATIO: float
    NUM_POLE_PAIRS: int


# Servo motor parameters dictionary
SERVO_PARAMS: Dict[str, Any] = {
    'ERROR_CODES': {
        0: 'No Error',
        1: 'Over voltage fault',
        2: 'Under voltage fault',
        3: 'DRV fault',
        4: 'Absolute over current fault',
        5: 'Over temp FET fault',
        6: 'Over temp motor fault',
        7: 'Gate driver over voltage fault',
        8: 'Gate driver under voltage fault',
        9: 'MCU under voltage fault',
        10: 'Booting from watchdog reset fault',
        11: 'Encoder SPI fault',
        12: 'Encoder sincos below min amplitude fault',
        13: 'Encoder sincos above max amplitude fault',
        14: 'Flash corruption fault',
        15: 'High offset current sensor 1 fault',
        16: 'High offset current sensor 2 fault',
        17: 'High offset current sensor 3 fault',
        18: 'Unbalanced currents fault'
    },
    'AK10-9': {
        'P_min': -32000,      # -3200 deg (0.1 deg resolution: -32000 * 0.1 = -3200)
        'P_max': 32000,       # 3200 deg (0.1 deg resolution: 32000 * 0.1 = 3200)
        'V_min': -100000,     # -100000 ERPM electrical speed
        'V_max': 100000,      # 100000 ERPM electrical speed
        'Curr_min': -60000,   # -60A (protocol: -60000 = -60A, 1000 = 1A)
        'Curr_max': 60000,    # 60A (protocol: 60000 = 60A, 1000 = 1A)
        'T_min': -15,         # NM
        'T_max': 15,          # NM
        'Temp_min': -20,      # -20°C (based on protocol doc)
        'Temp_max': 127,      # 127°C (based on protocol doc)
        'Kt_TMotor': 0.16,    # from TMotor website (actually 1/Kvll)
        'Current_Factor': 0.59, # UNTESTED CONSTANT!
        'Kt_actual': 0.206,   # UNTESTED CONSTANT!
        'GEAR_RATIO': 9.0,
        'NUM_POLE_PAIRS': 21,
        'Use_derived_torque_constants': False, # true if you have a better model
    },
    'AK80-9': {
        'P_min': -32000,      # -3200 deg (0.1 deg resolution: -32000 * 0.1 = -3200)
        'P_max': 32000,       # 3200 deg (0.1 deg resolution: 32000 * 0.1 = 3200)
        'V_min': -32000,      # -320000 rpm electrical speed (note: different from AK10-9)
        'V_max': 32000,       # 320000 rpm electrical speed (note: different from AK10-9)
        'Curr_min': -60000,   # -60A (protocol: -60000 = -60A, 1000 = 1A)
        'Curr_max': 60000,    # 60A (protocol: 60000 = 60A, 1000 = 1A)
        'T_min': -30,         # NM
        'T_max': 30,          # NM
        'Temp_min': -20,      # -20°C (based on protocol doc)
        'Temp_max': 127,      # 127°C (based on protocol doc)
        'Kt_TMotor': 0.091,   # from TMotor website (actually 1/Kvll)
        'Current_Factor': 0.59,
        'Kt_actual': 0.115,
        'GEAR_RATIO': 9.0,
        'NUM_POLE_PAIRS': 21,
        'Use_derived_torque_constants': False, # true if you have a better model
    },
    'CAN_PACKET_ID': {
        'CAN_PACKET_SET_DUTY': 0,        # Motor runs in duty cycle mode
        'CAN_PACKET_SET_CURRENT': 1,     # Motor runs in current loop mode
        'CAN_PACKET_SET_CURRENT_BRAKE': 2, # Motor current brake mode operation
        'CAN_PACKET_SET_RPM': 3,         # Motor runs in velocity mode
        'CAN_PACKET_SET_POS': 4,         # Motor runs in position loop mode
        'CAN_PACKET_SET_ORIGIN_HERE': 5, # Set origin mode
        'CAN_PACKET_SET_POS_SPD': 6,     # Position velocity loop mode
    },
}


class ServoControlMode(Enum):
    """Internal servo control modes for TMotor communication."""
    DUTY_CYCLE = 0
    CURRENT_LOOP = 1
    CURRENT_BRAKE = 2
    VELOCITY = 3
    POSITION = 4
    SET_ORIGIN = 5
    POSITION_VELOCITY = 6
    IDLE = 7


@dataclass
class ServoMotorState:
    """Data structure to store servo motor state information."""
    position: float = 0.0
    velocity: float = 0.0
    current: float = 0.0
    temperature: float = 0.0
    error: int = 0
    acceleration: float = 0.0

    def set_state(self, position: float, velocity: float, current: float,
                  temperature: float, error: int, acceleration: float) -> None:
        """Update all state values."""
        self.position = position
        self.velocity = velocity
        self.current = current
        self.temperature = temperature
        self.error = error
        self.acceleration = acceleration

    def set_state_obj(self, other_state: 'ServoMotorState') -> None:
        """Copy state from another ServoMotorState object."""
        self.position = other_state.position
        self.velocity = other_state.velocity
        self.current = other_state.current
        self.temperature = other_state.temperature
        self.error = other_state.error
        self.acceleration = other_state.acceleration


@dataclass
class ServoCommand:
    """Data structure to store servo commands."""
    position: float = 0.0
    velocity: float = 0.0
    current: float = 0.0
    duty: float = 0.0
    acceleration: float = 0.0


class CANManagerServo:
    """Singleton CAN manager for servo mode communication."""

    _instance: Optional['CANManagerServo'] = None
    _initialized: bool = False
    debug: bool = False
    """Set to true to display every message sent and received for debugging."""

    def __new__(cls) -> 'CANManagerServo':
        """Ensure singleton pattern."""
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self) -> None:
        """Initialize CAN bus connection."""
        if self._initialized:
            return

        LOGGER.info("Initializing CAN Manager for TMotor Servo Mode")

        try:
            # Configure CAN interface for Linux/RPi
            os.system('sudo /sbin/ip link set can0 down')
            os.system('sudo /sbin/ip link set can0 up type can bitrate 1000000')
            os.system('sudo ifconfig can0 txqueuelen 1000')

            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.notifier = can.Notifier(bus=self.bus, listeners=[])

            LOGGER.info(f"CAN bus connected: {self.bus}")
            self._initialized = True

        except Exception as e:
            LOGGER.error(f"CAN bus initialization failed: {e}")
            raise RuntimeError("CAN bus initialization failed") from e

    def __del__(self) -> None:
        """Clean up CAN interface."""
        try:
            os.system('sudo /sbin/ip link set can0 down')
        except Exception as e:
            LOGGER.warning(f"Error shutting down CAN interface: {e}")

    def add_motor(self, motor: 'TMotorServoActuator') -> None:
        """Add a motor to the CAN listener."""
        listener = MotorListener(self, motor)
        self.notifier.add_listener(listener)

    def send_message(self, motor_id: int, data: list, data_len: int) -> None:
        """Send CAN message to motor."""
        if data_len > 8:
            raise ValueError(f'Data too long in message for motor {motor_id}')

        if self.debug:
            LOGGER.info(f'ID: {hex(motor_id)}   Data: [{", ".join(hex(d) for d in data)}]')

        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=True)

        try:
            self.bus.send(message)
            if self.debug:
                LOGGER.info(f"Message sent on {self.bus.channel_info}")
        except can.CanError as e:
            if self.debug:
                LOGGER.error(f"Message NOT sent: {e}")
            else:
                LOGGER.error(f"Failed to send CAN message: {e}")

    def power_on(self, motor_id: int) -> None:
        """Send power on command."""
        self.send_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], 8)

    def power_off(self, motor_id: int) -> None:
        """Send power off command."""
        self.send_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], 8)

    def set_current(self, controller_id: int, current: float) -> None:
        """Send current control command."""
        # Convert current to protocol format: -60000 to 60000 represents -60A to 60A
        # So 1A = 1000 in protocol units (as per protocol documentation)
        current_protocol = int(current * 1000.0)
        buffer = self._pack_int32(current_protocol)
        message_id = controller_id | (SERVO_PARAMS['CAN_PACKET_ID']['CAN_PACKET_SET_CURRENT'] << 8)
        self.send_message(message_id, buffer, len(buffer))

    def set_current_brake(self, controller_id: int, current: float) -> None:
        """Send current brake control command."""
        # Convert current to protocol format: -60000 to 60000 represents -60A to 60A
        # So 1A = 1000 in protocol units (as per protocol documentation)
        current_protocol = int(current * 1000.0)
        buffer = self._pack_int32(current_protocol)
        message_id = controller_id | (SERVO_PARAMS['CAN_PACKET_ID']['CAN_PACKET_SET_CURRENT_BRAKE'] << 8)
        self.send_message(message_id, buffer, len(buffer))

    def set_velocity(self, controller_id: int, velocity: float) -> None:
        """Send velocity control command."""
        buffer = self._pack_int32(int(velocity))
        message_id = controller_id | (SERVO_PARAMS['CAN_PACKET_ID']['CAN_PACKET_SET_RPM'] << 8)
        self.send_message(message_id, buffer, len(buffer))

    def set_position(self, controller_id: int, position: float) -> None:
        """Send position control command."""
        buffer = self._pack_int32(int(position * 10.0))  # Position in degrees * 10 (0.1 deg resolution)
        message_id = controller_id | (SERVO_PARAMS['CAN_PACKET_ID']['CAN_PACKET_SET_POS'] << 8)
        self.send_message(message_id, buffer, len(buffer))

    def set_position_velocity(self, controller_id: int, position: float,
                            velocity: float, acceleration: float) -> None:
        """Send position control with velocity profile."""
        buffer = []
        buffer.extend(self._pack_int32(int(position * 10.0)))  # Position in degrees * 10 (0.1 deg resolution)
        buffer.extend(self._pack_int16(int(velocity / 10.0)))
        buffer.extend(self._pack_int16(int(acceleration / 10.0)))
        message_id = controller_id | (SERVO_PARAMS['CAN_PACKET_ID']['CAN_PACKET_SET_POS_SPD'] << 8)
        self.send_message(message_id, buffer, len(buffer))

    def set_origin(self, controller_id: int, mode: int = 1) -> None:
        """Set motor origin/zero position."""
        buffer = [mode]
        message_id = controller_id | (SERVO_PARAMS['CAN_PACKET_ID']['CAN_PACKET_SET_ORIGIN_HERE'] << 8)
        self.send_message(message_id, buffer, len(buffer))

    def set_duty_cycle(self, controller_id: int, duty: float) -> None:
        """Send duty cycle command."""
        buffer = self._pack_int32(int(duty * 100000.0))
        message_id = controller_id | (SERVO_PARAMS['CAN_PACKET_ID']['CAN_PACKET_SET_DUTY'] << 8)
        self.send_message(message_id, buffer, len(buffer))

    @staticmethod
    def _pack_int16(number: int) -> list:
        """Pack 16-bit signed integer into byte list (little-endian)."""
        # Handle negative numbers with 2's complement
        if number < 0:
            number = (1 << 16) + number
        return [number & 0xFF, (number >> 8) & 0xFF]

    @staticmethod
    def _pack_int32(number: int) -> list:
        """Pack 32-bit signed integer into byte list (little-endian)."""
        # Handle negative numbers with 2's complement
        if number < 0:
            number = (1 << 32) + number
        return [number & 0xFF, (number >> 8) & 0xFF,
                (number >> 16) & 0xFF, (number >> 24) & 0xFF]

    def parse_servo_message(self, data: bytes) -> ServoMotorState:
        """Parse received servo message into motor state."""
        if len(data) < 8:
            raise ValueError(f"Invalid message length: {len(data)}")

        # Parse according to TMotor servo protocol
        pos_int = int.from_bytes(data[0:2], byteorder='little', signed=True)
        spd_int = int.from_bytes(data[2:4], byteorder='little', signed=True)
        cur_int = int.from_bytes(data[4:6], byteorder='little', signed=True)

        motor_pos = float(pos_int * 0.1)  # Position in degrees (0.1 deg resolution)
        motor_spd = float(spd_int * 10.0)  # Speed in ERPM
        motor_cur = float(cur_int / 1000.0)  # Current in amps (protocol: 1000 = 1A)
        motor_temp = float(data[6])  # Temperature in Celsius
        motor_error = int(data[7])  # Error code

        if self.debug:
            LOGGER.info(f"Position: {motor_pos}")
            LOGGER.info(f"Velocity: {motor_spd}")
            LOGGER.info(f"Current: {motor_cur}")
            LOGGER.info(f"Temp: {motor_temp}")
            LOGGER.info(f"Error: {motor_error}")

        return ServoMotorState(motor_pos, motor_spd, motor_cur, motor_temp, motor_error, 0.0)


class MotorListener(can.Listener):
    """CAN message listener for motor updates."""

    def __init__(self, canman: CANManagerServo, motor: 'TMotorServoActuator') -> None:
        """Initialize listener."""
        self.canman = canman
        self.motor = motor

    def on_message_received(self, msg: can.Message) -> None:
        """Handle received CAN message."""
        data = bytes(msg.data)
        motor_id = msg.arbitration_id & 0x00000FF
        if motor_id == self.motor.ID:
            self.motor._update_state_async(self.canman.parse_servo_message(data))


def _servo_position_mode_entry(actuator: 'TMotorServoActuator') -> None:
    """Entry callback for position mode."""
    actuator._servo_mode = ServoControlMode.POSITION


def _servo_position_mode_exit(actuator: 'TMotorServoActuator') -> None:
    """Exit callback for position mode."""
    if not actuator.is_offline:
        actuator._canman.set_duty_cycle(actuator.ID, 0.0)


def _servo_current_mode_entry(actuator: 'TMotorServoActuator') -> None:
    """Entry callback for current mode."""
    actuator._servo_mode = ServoControlMode.CURRENT_LOOP


def _servo_current_mode_exit(actuator: 'TMotorServoActuator') -> None:
    """Exit callback for current mode."""
    if not actuator.is_offline:
        actuator._canman.set_current(actuator.ID, 0.0)


def _servo_velocity_mode_entry(actuator: 'TMotorServoActuator') -> None:
    """Entry callback for velocity mode."""
    actuator._servo_mode = ServoControlMode.VELOCITY


def _servo_velocity_mode_exit(actuator: 'TMotorServoActuator') -> None:
    """Exit callback for velocity mode."""
    if not actuator.is_offline:
        actuator._canman.set_velocity(actuator.ID, 0.0)


def _servo_impedance_mode_entry(actuator: 'TMotorServoActuator') -> None:
    """Entry callback for impedance mode (using position control)."""
    actuator._servo_mode = ServoControlMode.POSITION_VELOCITY


def _servo_impedance_mode_exit(actuator: 'TMotorServoActuator') -> None:
    """Exit callback for impedance mode."""
    if not actuator.is_offline:
        actuator._canman.set_duty_cycle(actuator.ID, 0.0)


def _servo_idle_mode_entry(actuator: 'TMotorServoActuator') -> None:
    """Entry callback for idle mode."""
    actuator._servo_mode = ServoControlMode.IDLE


def _servo_idle_mode_exit(actuator: 'TMotorServoActuator') -> None:
    """Exit callback for idle mode."""
    if not actuator.is_offline:
        actuator._canman.set_duty_cycle(actuator.ID, 0.0)


TMOTOR_SERVO_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    POSITION=ControlModeConfig(
        entry_callback=_servo_position_mode_entry,
        exit_callback=_servo_position_mode_exit,
        has_gains=True,
        max_gains=ControlGains(kp=1000.0, ki=100.0, kd=10.0, k=0.0, b=0.0, ff=0.0),
    ),
    CURRENT=ControlModeConfig(
        entry_callback=_servo_current_mode_entry,
        exit_callback=_servo_current_mode_exit,
        has_gains=True,
        max_gains=ControlGains(kp=100.0, ki=1000.0, kd=0.0, k=0.0, b=0.0, ff=100.0),
    ),
    VELOCITY=ControlModeConfig(
        entry_callback=_servo_velocity_mode_entry,
        exit_callback=_servo_velocity_mode_exit,
        has_gains=True,
        max_gains=ControlGains(kp=0.0, ki=0.0, kd=50.0, k=0.0, b=0.0, ff=0.0),
    ),
    IMPEDANCE=ControlModeConfig(
        entry_callback=_servo_impedance_mode_entry,
        exit_callback=_servo_impedance_mode_exit,
        has_gains=True,
        max_gains=ControlGains(kp=0.0, ki=0.0, kd=0.0, k=100.0, b=10.0, ff=0.0),
    ),
    TORQUE=ControlModeConfig(
        entry_callback=_servo_current_mode_entry,
        exit_callback=_servo_current_mode_exit,
        has_gains=True,
        max_gains=ControlGains(kp=100.0, ki=1000.0, kd=0.0, k=0.0, b=0.0, ff=100.0),
    ),
    IDLE=ControlModeConfig(
        entry_callback=_servo_idle_mode_entry,
        exit_callback=_servo_idle_mode_exit,
        has_gains=False,
        max_gains=ControlGains(),
    ),
)


# Default variables to be logged
LOG_VARIABLES = [
    "motor_position",
    "motor_speed",
    "motor_current",
    "motor_temperature"
]
"""Default variables to be logged"""


class TMotorServoActuator(ActuatorBase):
    """
    Examples:
        >>> with TMotorServoActuator(motor_type="AK80-9", motor_ID=1) as motor:
        ...     motor.set_control_mode(CONTROL_MODES.POSITION)
        ...     motor.set_position_gains(kp=10.0, ki=1.0, kd=0.5, ff=0.0)
        ...     motor.set_motor_position(1.57)  # 90 degrees
        ...     motor.update()
    """

    def __init__(
        self,
        tag: str = "TMotorServoActuator",
        motor_type: str = "AK80-9",
        motor_ID: int = 1,
        gear_ratio: float = 1.0,
        frequency: int = 500,
        offline: bool = False,
        max_temperature: float = 80.0,
        CSV_file: Optional[str] = None,
        log_vars: Optional[list] = None,
        **kwargs: Any,
    ) -> None:
        """
        Initialize TMotor servo actuator.

        Args:
            tag: Unique identifier for the actuator
            motor_type: Motor model (AK80-9, AK10-9)
            motor_ID: CAN ID for the motor
            gear_ratio: Gear ratio between motor and output
            frequency: Control loop frequency in Hz
            offline: Whether to run in offline mode
            max_temperature: Maximum operating temperature in Celsius
            CSV_file: A CSV file to output log info to. If None, no log will be recorded.
            log_vars: The variables to log as a python list. The full list of possibilities is
                - "output_angle"
                - "output_velocity"
                - "output_acceleration"
                - "current"
                - "output_torque"
                - "motor_angle"
                - "motor_velocity"
                - "motor_acceleration"
                - "motor_torque"
            **kwargs: Additional arguments passed to ActuatorBase
        """
        # Validate motor type
        if motor_type not in SERVO_PARAMS:
            raise ValueError(f"Unsupported motor type: {motor_type}. "
                           f"Supported types: {list(SERVO_PARAMS.keys())}")

        super().__init__(
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=TMOTOR_SERVO_CONSTANTS,
            frequency=frequency,
            offline=offline,
            **kwargs
        )

        # Motor configuration
        self.motor_type = motor_type
        self.ID = motor_ID
        self.max_temperature = max_temperature

        # Logging configuration
        self.csv_file_name = CSV_file
        self.log_vars = log_vars or LOG_VARIABLES
        self.LOG_FUNCTIONS = {
            "motor_position": self.get_motor_angle_radians,
            "motor_speed": self.get_motor_velocity_radians_per_second,
            "motor_current": self.get_current_qaxis_amps,
            "motor_temperature": self.get_temperature_celsius,
        }

        # Get motor parameters
        self._params = SERVO_PARAMS[motor_type]
        if not isinstance(self._params, dict):
            raise ValueError(f"Invalid motor parameters for {motor_type}")

        # State management
        self._motor_state = ServoMotorState()
        self._motor_state_async = ServoMotorState()
        self._command = ServoCommand()
        self._servo_mode = ServoControlMode.POSITION

        # Control gains
        self._position_gains = ControlGains()
        self._current_gains = ControlGains()
        self._velocity_gains = ControlGains()
        self._impedance_gains = ControlGains()

        # Timing and state tracking
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._last_command_time: Optional[float] = None
        self._entered = False
        self._updated = False
        self._command_sent = False

        # Unit conversions
        self.rad_per_degree = np.pi / 180.0
        self.radps_per_erpm = 2 * np.pi / (60 * self._params['NUM_POLE_PAIRS'])
        self.rad_per_eang = np.pi / self._params['NUM_POLE_PAIRS']

        # Thermal management
        self._thermal_model = ThermalModel(
            temp_limit_windings=self.max_winding_temperature,
            soft_border_C_windings=10.0,
            temp_limit_case=self.max_case_temperature,
            soft_border_C_case=10.0,
        )

        # CAN communication
        if not self.is_offline:
            self._canman = CANManagerServo()
        self._canman.add_motor(self)

        LOGGER.info(f"Initialized TMotor servo actuator: {self.device_info_string()}")

    @property
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        """Get control mode configurations."""
        return TMOTOR_SERVO_CONTROL_MODE_CONFIGS

    def device_info_string(self) -> str:
        """Get device information string."""
        return f"{self.motor_type} ID: {self.ID}"

    def __enter__(self) -> 'TMotorServoActuator':
        """Used to safely power the motor on and begin the log file."""
        LOGGER.info(f'Turning on control for device: {self.device_info_string()}')

        if self.csv_file_name is not None:
            with open(self.csv_file_name, 'w') as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"] + self.log_vars)
            self.csv_file = open(self.csv_file_name, 'a').__enter__()
            self.csv_writer = csv.writer(self.csv_file)

        self.start()
        return self

    def __exit__(self, etype: Any, value: Any, tb: Any) -> None:
        """Used to safely power the motor off and close the log file."""
        LOGGER.info(f'Turning off control for device: {self.device_info_string()}')
        self.stop()

        if self.csv_file_name is not None:
            self.csv_file.__exit__(etype, value, tb)

        if not (etype is None):
            traceback.print_exception(etype, value, tb)

    @check_actuator_connection
    def start(self) -> None:
        """Start the actuator and enable control."""
        LOGGER.info(f"Starting control for {self.device_info_string()}")

        if not self.is_offline:
            self._canman.power_on(self.ID)
        self._send_command()

            # Verify connection
        if not self.check_can_connection():
                raise RuntimeError(f"Device not connected: {self.device_info_string()}")

        self._entered = True
        self._is_open = True
        self._is_streaming = True

    @check_actuator_stream
    @check_actuator_open
    def stop(self) -> None:
        """Stop the actuator and disable control."""
        LOGGER.info(f"Stopping control for {self.device_info_string()}")

        if not self.is_offline:
            self._canman.power_off(self.ID)

        self._is_open = False
        self._is_streaming = False

    def update(self) -> None:
        """Update motor state and send current command."""
        if not self._entered:
            raise RuntimeError(f"Motor not started: {self.device_info_string()}")

        # Temperature check
        if self.case_temperature > self.max_temperature:
            raise RuntimeError(f"Temperature {self.case_temperature}°C exceeds limit "
                             f"{self.max_temperature}°C for {self.device_info_string()}")

        # Communication timeout check
        now = time.time()
        if (self._last_command_time is not None and
            (now - self._last_command_time) < 0.25 and
            (now - self._last_update_time) > 0.1):
            warnings.warn(f"No data from motor. Check connection: {self.device_info_string()}",
                         RuntimeWarning, stacklevel=2)
        else:
            self._command_sent = False

        # Update state
        self._motor_state.set_state_obj(self._motor_state_async)
        self._motor_state.position = self._motor_state.position / self._params['GEAR_RATIO']

        # Send command
        if not self.is_offline:
            self._send_command()

        # Writing to log file
        if self.csv_file_name is not None:
            timestamp = self._last_update_time - self._start_time
            log_data = [self.LOG_FUNCTIONS[var]() for var in self.log_vars]
            self.csv_writer.writerow([timestamp] + log_data)

        self._updated = False

    def _send_command(self) -> None:
        """Send command based on current control mode and servo mode."""
        if self.is_offline:
            return

        try:
            if self._servo_mode == ServoControlMode.POSITION:
                pos_deg = self._command.position / self.rad_per_degree
                self._canman.set_position(self.ID, pos_deg)
            elif self._servo_mode == ServoControlMode.POSITION_VELOCITY:
                pos_deg = self._command.position / self.rad_per_degree
                vel_erpm = self._command.velocity / self.radps_per_erpm
                acc_erpm = self._command.acceleration / self.radps_per_erpm
                self._canman.set_position_velocity(self.ID, pos_deg, vel_erpm, acc_erpm)
            elif self._servo_mode == ServoControlMode.CURRENT_LOOP:
                self._canman.set_current(self.ID, self._command.current)
            elif self._servo_mode == ServoControlMode.CURRENT_BRAKE:
                self._canman.set_current_brake(self.ID, self._command.current)
            elif self._servo_mode == ServoControlMode.VELOCITY:
                vel_erpm = self._command.velocity / self.radps_per_erpm
                self._canman.set_velocity(self.ID, vel_erpm)
            elif self._servo_mode == ServoControlMode.IDLE:
                self._canman.set_duty_cycle(self.ID, 0.0)
            else:
                self._canman.set_duty_cycle(self.ID, 0.0)

            self._last_command_time = time.time()
            self._command_sent = True

        except Exception as e:
            LOGGER.error(f"Failed to send command: {e}")
            raise

    def _update_state_async(self, servo_state: ServoMotorState) -> None:
        """Update motor state asynchronously from CAN messages."""
        if servo_state.error != 0:
            error_codes = SERVO_PARAMS['ERROR_CODES']
            if isinstance(error_codes, dict):
                error_msg = error_codes.get(servo_state.error, 'Unknown error')
            else:
                error_msg = 'Unknown error'
            raise RuntimeError(f"Motor error {servo_state.error}: {error_msg} "
                             f"for {self.device_info_string()}")

        now = time.time()
        dt = now - self._last_update_time
        self._last_update_time = now

        # Calculate acceleration
        if dt > 0:
            self._motor_state_async.acceleration = (
                servo_state.velocity - self._motor_state_async.velocity) / dt

        self._motor_state_async.set_state_obj(servo_state)
        self._updated = True

    def home(
        self,
        homing_voltage: int = 2000,
        homing_frequency: Optional[int] = None,
        homing_direction: int = -1,
        output_position_offset: float = 0.0,
        current_threshold: int = 5000,
        velocity_threshold: float = 0.001,
    ) -> None:
        """
        Home the actuator by setting current position as zero.

        For servo mode, this simply sets the current position as the origin.
        """
        if not self.is_offline:
            self._canman.set_origin(self.ID, 1)  # Set permanent zero point
            time.sleep(0.1)  # Allow command to process

        self._is_homed = True
        LOGGER.info(f"Homed actuator: {self.device_info_string()}")

    def check_can_connection(self) -> bool:
        """Check CAN connection by sending test messages."""
        if self.is_offline:
            return True

        if not self._entered:
            raise RuntimeError("Cannot check connection before starting motor control")

        listener = can.BufferedReader()
        self._canman.notifier.add_listener(listener)

        try:
            # Send test messages
            for _ in range(10):
                self._canman.power_on(self.ID)
                time.sleep(0.001)

            time.sleep(0.1)  # Wait for responses

            # Check for replies
            success = True
            for _ in range(10):
                if listener.get_message(timeout=0.1) is None:
                    success = False
                    break

            return success

        finally:
            self._canman.notifier.remove_listener(listener)

    # Additional utility methods from original implementation
    def qaxis_current_to_TMotor_current(self, iq: float) -> float:
        """Convert q-axis current to TMotor current."""
        gear_ratio = self._params['GEAR_RATIO']
        kt_tmotor = self._params['Kt_TMotor']
        current_factor = self._params['Current_Factor']
        return iq * (gear_ratio * kt_tmotor) / current_factor

    def get_temperature_celsius(self) -> float:
        """Get motor temperature in Celsius."""
        return self._motor_state.temperature

    def get_motor_error_code(self) -> int:
        """Get motor error code."""
        return self._motor_state.error

    def get_current_qaxis_amps(self) -> float:
        """Get q-axis current in amps."""
        return self._motor_state.current

    def get_output_angle_radians(self) -> float:
        """Get output angle in radians."""
        return self._motor_state.position * self.rad_per_degree

    def get_output_velocity_radians_per_second(self) -> float:
        """Get output velocity in radians per second."""
        return self._motor_state.velocity * self.radps_per_erpm

    def get_output_acceleration_radians_per_second_squared(self) -> float:
        """Get output acceleration in radians per second squared."""
        return self._motor_state.acceleration

    def get_output_torque_newton_meters(self) -> float:
        """Get output torque in Nm."""
        return (self.get_current_qaxis_amps() *
                self._params["Kt_actual"] *
                self._params["GEAR_RATIO"])

    def get_motor_angle_radians(self) -> float:
        """Get motor angle in radians."""
        return (self._motor_state.position *
                self.rad_per_degree *
                self._params["GEAR_RATIO"])

    def get_motor_velocity_radians_per_second(self) -> float:
        """Get motor velocity in radians per second."""
        return (self._motor_state.velocity *
                self.radps_per_erpm *
                self._params["GEAR_RATIO"])

    def get_motor_acceleration_radians_per_second_squared(self) -> float:
        """Get motor acceleration in radians per second squared."""
        return self._motor_state.acceleration * self._params["GEAR_RATIO"]

    def get_motor_torque_newton_meters(self) -> float:
        """Get motor torque in Nm."""
        return self.get_output_torque_newton_meters() / self._params["GEAR_RATIO"]

    # Control mode entry methods
    def enter_duty_cycle_control(self) -> None:
        """Enter duty cycle control mode."""
        self._servo_mode = ServoControlMode.DUTY_CYCLE

    def enter_current_control(self) -> None:
        """Enter current control mode."""
        self._servo_mode = ServoControlMode.CURRENT_LOOP

    def enter_current_brake_control(self) -> None:
        """Enter current brake control mode."""
        self._servo_mode = ServoControlMode.CURRENT_BRAKE

    def enter_velocity_control(self) -> None:
        """Enter velocity control mode."""
        self._servo_mode = ServoControlMode.VELOCITY

    def enter_position_control(self) -> None:
        """Enter position control mode."""
        self._servo_mode = ServoControlMode.POSITION

    def enter_position_velocity_control(self) -> None:
        """Enter position velocity control mode."""
        self._servo_mode = ServoControlMode.POSITION_VELOCITY

    def enter_idle_mode(self) -> None:
        """Enter idle mode."""
        self._servo_mode = ServoControlMode.IDLE

    # Command setting methods
    def set_output_angle_radians(self, pos: float, vel: float = 0.0, acc: float = 0.0) -> None:
        """Set output angle in radians with optional velocity and acceleration."""
        # Convert to degrees for range checking
        pos_degrees = pos * 180.0 / np.pi
        p_max_deg = self._params['P_max'] * 0.1  # Convert from protocol units to degrees
        if abs(pos_degrees) >= p_max_deg:
            raise RuntimeError(
                f"Cannot control position with magnitude greater than {p_max_deg} degrees!"
            )

        pos = (pos / self.rad_per_degree)  # Convert to degrees for protocol
        vel = (vel / self.radps_per_erpm)
        acc = (acc / self.radps_per_erpm)

        if self._servo_mode == ServoControlMode.POSITION_VELOCITY:
            self._command.position = pos
            self._command.velocity = vel
            self._command.acceleration = acc
        elif self._servo_mode == ServoControlMode.POSITION:
            self._command.position = pos
        else:
            raise RuntimeError(
                f"Attempted to send position command without entering position control {self.device_info_string()}"
            )

    def set_duty_cycle_percent(self, duty: float) -> None:
        """Set duty cycle (-1 to 1)."""
        if self._servo_mode != ServoControlMode.DUTY_CYCLE:
            raise RuntimeError(
                f"Attempted to send duty cycle command without entering duty cycle mode "
                f"for device {self.device_info_string()}"
            )

        if abs(duty) > 1:
            raise RuntimeError("Cannot control using duty cycle mode for duty cycles greater than 100%!")
        self._command.duty = duty

    def set_output_velocity_radians_per_second(self, vel: float) -> None:
        """Set output velocity in radians per second."""
        # Convert to ERPM for range checking
        vel_erpm = vel / self.radps_per_erpm
        v_max = self._params["V_max"]
        if abs(vel_erpm) >= v_max:
            raise RuntimeError(
                f"Cannot control using speed mode for velocities with magnitude greater than {v_max} ERPM!"
            )

        if self._servo_mode != ServoControlMode.VELOCITY:
            raise RuntimeError(
                f"Attempted to send speed command without entering velocity mode "
                f"for device {self.device_info_string()}"
            )

        self._command.velocity = vel_erpm

    def set_motor_current_qaxis_amps(self, current: float) -> None:
        """Set motor current in amps."""
        if self._servo_mode not in [ServoControlMode.CURRENT_LOOP, ServoControlMode.CURRENT_BRAKE]:
            raise RuntimeError(
                f"Attempted to send current command before entering current mode "
                f"for device {self.device_info_string()}"
            )

        # Check current limits based on control mode and motor parameters
        curr_min = self._params['Curr_min'] / 1000.0  # Convert from protocol units to amps
        curr_max = self._params['Curr_max'] / 1000.0  # Convert from protocol units to amps

        if self._servo_mode == ServoControlMode.CURRENT_BRAKE:
            # Current brake mode: 0 to max current only
            if current < 0 or current > curr_max:
                raise RuntimeError(f"Current brake mode requires current between 0-{curr_max}A, got {current}A")
        else:
            # Current loop mode: min to max current
            if current < curr_min or current > curr_max:
                raise RuntimeError(
                    f"Current loop mode requires current between {curr_min}A to {curr_max}A, got {current}A"
                )

        self._command.current = current

    def set_output_torque_newton_meters(self, torque: float) -> None:
        """Set output torque in Nm."""
        kt_actual = self._params["Kt_actual"]
        gear_ratio = self._params["GEAR_RATIO"]
        self.set_motor_current_qaxis_amps(torque / kt_actual / gear_ratio)

    def set_motor_torque_newton_meters(self, torque: float) -> None:
        """Set motor torque in Nm."""
        self.set_output_torque_newton_meters(torque * self._params["GEAR_RATIO"])

    def set_motor_angle_radians(self, pos: float, vel: float = 0.0, acc: float = 0.0) -> None:
        """Set motor angle in radians."""
        self.set_output_angle_radians(pos / self._params["GEAR_RATIO"], vel, acc)

    def set_motor_velocity_radians_per_second(self, vel: float) -> None:
        """Set motor velocity in radians per second."""
        self.set_output_velocity_radians_per_second(vel / self._params["GEAR_RATIO"])

    # OSL Interface Implementation
    def set_motor_voltage(self, value: float) -> None:
        """Set motor voltage (not directly supported in servo mode)."""
        # Convert voltage to approximate duty cycle
        nominal_voltage = 24.0  # Assume 24V nominal
        duty_cycle = np.clip(value / nominal_voltage, -1.0, 1.0)
        self._command.duty = duty_cycle
        self._servo_mode = ServoControlMode.DUTY_CYCLE

    def set_motor_current(self, value: float) -> None:
        """Set motor current command."""
        curr_max = self._params['Curr_max'] / 1000.0  # Convert from protocol units to amps
        if abs(value) > curr_max:
            raise ValueError(f"Current {value}A exceeds motor limits (±{curr_max}A)")
        self.set_motor_current_qaxis_amps(value)

    def set_motor_position(self, value: float) -> None:
        """Set motor position command in radians."""
        # Convert to degrees for range checking
        value_deg = value * 180.0 / np.pi
        p_max_deg = self._params['P_max'] * 0.1  # Convert from protocol units to degrees
        if abs(value_deg) >= p_max_deg:
            raise ValueError(
                f"Position {value_deg}° exceeds motor limits (±{p_max_deg}°)"
            )
        self._command.position = value_deg  # Store in degrees

    def set_output_position(self, value: float) -> None:
        """Set output position command in radians."""
        self.set_motor_position(value * self.gear_ratio)

    def set_motor_torque(self, value: float) -> None:
        """Set motor torque command in Nm."""
        current = value / self._params['Kt_actual']
        self.set_motor_current(current)

    def set_output_torque(self, value: float) -> None:
        """Set output torque command in Nm."""
        motor_torque = value * self.gear_ratio
        self.set_motor_torque(motor_torque)

    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """Set current control gains (stored for reference)."""
        self._current_gains = ControlGains(kp=kp, ki=ki, kd=kd, ff=ff)

    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """Set position control gains (stored for reference)."""
        self._position_gains = ControlGains(kp=kp, ki=ki, kd=kd, ff=ff)

    def set_impedance_gains(self, kp: float, ki: float, kd: float,
                          k: float, b: float, ff: float) -> None:
        """Set impedance control gains."""
        self._impedance_gains = ControlGains(kp=kp, ki=ki, kd=kd, k=k, b=b, ff=ff)
        # For impedance control, use position-velocity mode with gains
        self._servo_mode = ServoControlMode.POSITION_VELOCITY

    def set_velocity_gains(self, kd: float) -> None:
        """Set velocity control gains."""
        self._velocity_gains = ControlGains(kd=kd)

    def set_output_velocity(self, value: float) -> None:
        """Set output velocity command in rad/s."""
        motor_velocity = value * self.gear_ratio
        self.set_motor_velocity(motor_velocity)

    def set_motor_velocity(self, value: float) -> None:
        """Set motor velocity command in rad/s."""
        # Convert to ERPM for range checking
        value_erpm = value / self.radps_per_erpm
        v_max = self._params["V_max"]
        if abs(value_erpm) >= v_max:
            raise ValueError(f"Velocity {value_erpm} ERPM exceeds motor limits (±{v_max} ERPM)")
        self._command.velocity = value_erpm

    # Property implementations
    @property
    def motor_position(self) -> float:
        """Get motor position in radians."""
        return self._motor_state.position * self.rad_per_degree

    @property
    def output_position(self) -> float:
        """Get output position in radians."""
        return self.motor_position / self.gear_ratio

    @property
    def motor_velocity(self) -> float:
        """Get motor velocity in rad/s."""
        return self._motor_state.velocity * self.radps_per_erpm

    @property
    def output_velocity(self) -> float:
        """Get output velocity in rad/s."""
        return self.motor_velocity / self.gear_ratio

    @property
    def motor_voltage(self) -> float:
        """Get motor voltage (estimated from duty cycle)."""
        nominal_voltage = 24.0
        return self._command.duty * nominal_voltage

    @property
    def motor_current(self) -> float:
        """Get motor current in amps."""
        return self._motor_state.current

    @property
    def motor_torque(self) -> float:
        """Get motor torque in Nm."""
        return self.motor_current * self._params['Kt_actual']

    @property
    def output_torque(self) -> float:
        """Get output torque in Nm."""
        return self.motor_torque / self.gear_ratio

    @property
    def case_temperature(self) -> float:
        """Get case temperature in Celsius."""
        return self._motor_state.temperature

    @property
    def winding_temperature(self) -> float:
        """Get estimated winding temperature in Celsius."""
        return getattr(self._thermal_model, 'T_w', self.case_temperature)

    @property
    def motor_acceleration(self) -> float:
        """Get motor acceleration in rad/s²."""
        return self._motor_state.acceleration * self.radps_per_erpm

    @property
    def output_acceleration(self) -> float:
        """Get output acceleration in rad/s²."""
        return self.motor_acceleration / self.gear_ratio

    # Additional properties for compatibility
    @property
    def temperature(self) -> float:
        """Temperature in degrees Celsius."""
        return self.get_temperature_celsius()

    @property
    def error(self) -> int:
        """Motor error code. 0 means no error."""
        return self.get_motor_error_code()

    @property
    def current_qaxis(self) -> float:
        """Q-axis current in amps."""
        return self.get_current_qaxis_amps()

    @property
    def position(self) -> float:
        """Output angle in rad."""
        return self.get_output_angle_radians()

    @property
    def velocity(self) -> float:
        """Output velocity in rad/s."""
        return self.get_output_velocity_radians_per_second()

    @property
    def acceleration(self) -> float:
        """Output acceleration in rad/s/s."""
        return self.get_output_acceleration_radians_per_second_squared()

    @property
    def torque(self) -> float:
        """Output torque in Nm."""
        return self.get_output_torque_newton_meters()

    @property
    def angle_motorside(self) -> float:
        """Motor-side angle in rad."""
        return self.get_motor_angle_radians()

    @property
    def velocity_motorside(self) -> float:
        """Motor-side velocity in rad/s."""
        return self.get_motor_velocity_radians_per_second()

    @property
    def acceleration_motorside(self) -> float:
        """Motor-side acceleration in rad/s/s."""
        return self.get_motor_acceleration_radians_per_second_squared()

    @property
    def torque_motorside(self) -> float:
        """Motor-side torque in Nm."""
        return self.get_motor_torque_newton_meters()

    def __str__(self) -> str:
        """String representation of the actuator state."""
        return (f"{self.device_info_string()} | "
                f"Pos: {self.output_position:.3f} rad | "
                f"Vel: {self.output_velocity:.3f} rad/s | "
                f"Cur: {self.motor_current:.3f} A | "
                f"Torque: {self.output_torque:.3f} Nm | "
                f"Temp: {self.case_temperature:.1f}°C")


if __name__ == "__main__":
    # Current Loop Control Example
    # Focus: Send torque commands and read motor parameters

    import time
    from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop
    try:
        with TMotorServoActuator(motor_type="AK80-9", motor_ID=1, offline=True) as motor:
            print(f"Motor: {motor.device_info_string()}")

            # Initialize
            motor.home()
            motor.set_control_mode(CONTROL_MODES.CURRENT)
            print("Current loop mode activated")

            # Control loop: Send torque commands and read parameters
            loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)

            for t in loop:
                if t > 10.0:  # Run for 10 seconds
                    break

                # Send torque command (example: 5Nm sine wave)
                torque_command = 5.0 * np.sin(2 * np.pi * 0.5 * t)  # 5Nm, 0.5Hz
                motor.set_output_torque(torque_command)

                # Update motor state
                motor.update()

                # Read motor parameters
                position = motor.output_position  # rad
                velocity = motor.output_velocity  # rad/s
                current = motor.motor_current     # A
                temperature = motor.case_temperature  # °C

                # Display every 0.5 seconds
                if int(t * 2) % 1 == 0:
                    print(f"t={t:4.1f}s | Torque={torque_command:5.1f}Nm | "
                          f"Pos={position:6.3f}rad | Vel={velocity:6.2f}rad/s | "
                          f"Curr={current:5.1f}A | Temp={temperature:4.1f}°C")

            # Stop motor
            motor.set_output_torque(0.0)
            motor.update()
            print("Motor stopped")

    except Exception as e:
        print(f"Error: {e}")

    print("Example completed")
