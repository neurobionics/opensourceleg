import os
import time
import traceback
import warnings
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional, cast

import can
import numpy as np

from opensourceleg.actuators.base import (
    CONTROL_MODE_CONFIGS,
    CONTROL_MODES,
    MOTOR_CONSTANTS,
    ActuatorBase,
    ControlModeConfig,
)
from opensourceleg.actuators.decorators import (
    check_actuator_connection,
    check_actuator_open,
    check_actuator_stream,
)
from opensourceleg.logging.logger import LOGGER
from opensourceleg.math import ThermalModel
from opensourceleg.utilities import SoftRealtimeLoop

# TMotor servo mode parameters configuration
SERVO_PARAMS: dict[str, Any] = {
    "ERROR_CODES": {
        0: "No Error",
        1: "Over voltage fault",
        2: "Under voltage fault",
        3: "DRV fault",
        4: "Absolute over current fault",
        5: "Over temp FET fault",
        6: "Over temp motor fault",
        7: "Gate driver over voltage fault",
        8: "Gate driver under voltage fault",
        9: "MCU under voltage fault",
        10: "Booting from watchdog reset fault",
        11: "Encoder SPI fault",
        12: "Encoder sincos below min amplitude fault",
        13: "Encoder sincos above max amplitude fault",
        14: "Flash corruption fault",
        15: "High offset current sensor 1 fault",
        16: "High offset current sensor 2 fault",
        17: "High offset current sensor 3 fault",
        18: "Unbalanced currents fault",
    },
    "AK10-9": {
        "P_min": -32000,  # -3200 deg
        "P_max": 32000,  # 3200 deg
        "V_min": -100000,  # ERPM
        "V_max": 100000,  # ERPM
        "Curr_min": -60000,  # -60A (protocol units)
        "Curr_max": 60000,  # 60A (protocol units)
        "Kt_actual": 0.206,  # Nm/A
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,
    },
    "AK80-9": {
        "P_min": -32000,  # -3200 deg
        "P_max": 32000,  # 3200 deg
        "V_min": -32000,  # ERPM
        "V_max": 32000,  # ERPM
        "Curr_min": -60000,  # -60A (protocol units)
        "Curr_max": 60000,  # 60A (protocol units)
        "Kt_actual": 0.115,  # Nm/A
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,
    },
    "CAN_PACKET_ID": {
        "CAN_PACKET_SET_DUTY": 0,
        "CAN_PACKET_SET_CURRENT": 1,
        "CAN_PACKET_SET_CURRENT_BRAKE": 2,
        "CAN_PACKET_SET_RPM": 3,
        "CAN_PACKET_SET_POS": 4,
        "CAN_PACKET_SET_ORIGIN_HERE": 5,
        "CAN_PACKET_SET_POS_SPD": 6,
    },
}

# TMotor servo mode constants
TMOTOR_SERVO_CONSTANTS = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=3600,  # 360 degrees * 10
    NM_PER_AMP=0.115,  # AK80-9 approximate
    NM_PER_RAD_TO_K=0.0,  # servo mode not needed
    NM_S_PER_RAD_TO_B=0.0,  # servo mode not needed
    MAX_CASE_TEMPERATURE=80.0,
    MAX_WINDING_TEMPERATURE=110.0,
)


@dataclass
class ServoMotorState:
    """Motor state data structure"""

    position: float = 0.0  # degrees
    velocity: float = 0.0  # ERPM
    current: float = 0.0  # amps
    temperature: float = 0.0  # celsius
    error: int = 0
    acceleration: float = 0.0  # rad/s²


class ServoControlMode(Enum):
    """TMotor servo control modes"""

    POSITION = 4
    VELOCITY = 3
    CURRENT = 1
    IDLE = 7


class CANManagerServo:
    """TMotor servo mode CAN communication manager"""

    _instance: Optional["CANManagerServo"] = None
    debug: bool = False
    _initialized: bool = False

    def __new__(cls) -> "CANManagerServo":
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self) -> None:
        if hasattr(self, "_initialized") and self._initialized:
            return

        LOGGER.info("Initializing CAN Manager for TMotor Servo Mode")
        try:
            # Configure CAN interface
            os.system("sudo /sbin/ip link set can0 down")  # noqa: S605, S607
            # Configure CAN interface with bitrate
            os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")  # noqa: S605, S607
            os.system("sudo ifconfig can0 txqueuelen 1000")  # noqa: S605, S607

            self.bus = can.interface.Bus(channel="can0", bustype="socketcan")
            self.notifier = can.Notifier(bus=self.bus, listeners=[])

            LOGGER.info(f"CAN bus connected: {self.bus}")
            self._initialized = True

        except Exception as e:
            LOGGER.error(f"CAN bus initialization failed: {e}")
            raise RuntimeError("CAN bus initialization failed") from e

    def __del__(self) -> None:
        try:
            os.system("sudo /sbin/ip link set can0 down")  # noqa: S605, S607
        except Exception as e:
            LOGGER.warning(f"Error shutting down CAN interface: {e}")

    def send_message(self, motor_id: int, data: list, data_len: int) -> None:
        """Send CAN message"""
        if data_len > 8:
            raise ValueError(f"Data too long in message for motor {motor_id}")

        if self.debug:
            LOGGER.info(f'ID: {hex(motor_id)} Data: [{", ".join(hex(d) for d in data)}]')

        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=True)

        try:
            self.bus.send(message)
        except can.CanError as e:
            LOGGER.error(f"Failed to send CAN message: {e}")

    def power_on(self, motor_id: int) -> None:
        """Send power on command"""
        self.send_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], 8)

    def power_off(self, motor_id: int) -> None:
        """Send power off command"""
        self.send_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], 8)

    def set_current(self, controller_id: int, current: float) -> None:
        """Send current control command"""
        current_protocol = int(current * 1000.0)  # Convert to protocol units
        buffer = self._pack_int32(current_protocol)
        message_id = controller_id | (SERVO_PARAMS["CAN_PACKET_ID"]["CAN_PACKET_SET_CURRENT"] << 8)
        self.send_message(message_id, buffer, len(buffer))

    def set_velocity(self, controller_id: int, velocity: float) -> None:
        """Send velocity control command"""
        buffer = self._pack_int32(int(velocity))
        message_id = controller_id | (SERVO_PARAMS["CAN_PACKET_ID"]["CAN_PACKET_SET_RPM"] << 8)
        self.send_message(message_id, buffer, len(buffer))

    def set_position(self, controller_id: int, position: float) -> None:
        """Send position control command"""
        # NOTE(IMPORTANT): TMotor spec uses 1,000,000 scale (0.000001° resolution) per documentation
        # Current implementation uses 10 scale (0.1° resolution) for simplicity
        # To enable high-precision position control, change to: int(position * 1000000.0)
        buffer = self._pack_int32(int(position * 10.0))  # 0.1 degree resolution
        message_id = controller_id | (SERVO_PARAMS["CAN_PACKET_ID"]["CAN_PACKET_SET_POS"] << 8)
        self.send_message(message_id, buffer, len(buffer))

    def set_origin(self, controller_id: int, mode: int = 1) -> None:
        """Set motor origin"""
        buffer = [mode]
        message_id = controller_id | (SERVO_PARAMS["CAN_PACKET_ID"]["CAN_PACKET_SET_ORIGIN_HERE"] << 8)
        self.send_message(message_id, buffer, len(buffer))

    @staticmethod
    def _pack_int32(number: int) -> list:
        """Pack 32-bit integer to byte list (big-endian, MSB first)"""
        if number < 0:
            number = (1 << 32) + number
        # TMotor expects big-endian (MSB first) per documentation
        return [(number >> 24) & 0xFF, (number >> 16) & 0xFF, (number >> 8) & 0xFF, number & 0xFF]

    def parse_servo_message(self, data: bytes) -> ServoMotorState:
        """Parse servo message to motor state"""
        if len(data) < 8:
            raise ValueError(f"Invalid message length: {len(data)}")

        # Fix endian issue: TMotor packs as big-endian (Data[n]<<8 | Data[n+1])
        pos_int = int.from_bytes(data[0:2], byteorder="big", signed=True)
        spd_int = int.from_bytes(data[2:4], byteorder="big", signed=True)
        cur_int = int.from_bytes(data[4:6], byteorder="big", signed=True)

        motor_pos = float(pos_int * 0.1)  # position (degrees)
        motor_spd = float(spd_int * 10.0)  # velocity (ERPM)
        motor_cur = float(cur_int / 1000.0)  # current (amps)
        motor_temp = float(data[6])  # temperature (celsius)
        motor_error = int(data[7])  # error code

        return ServoMotorState(motor_pos, motor_spd, motor_cur, motor_temp, motor_error, 0.0)

    def add_motor_listener(self, motor: "TMotorServoActuator") -> None:
        """Add motor listener"""
        listener = MotorListener(self, motor)
        self.notifier.add_listener(listener)


class MotorListener(can.Listener):
    """CAN message listener"""

    def __init__(self, canman: CANManagerServo, motor: "TMotorServoActuator") -> None:
        self.canman = canman
        self.motor = motor

    def on_message_received(self, msg: can.Message) -> None:
        """Handle received CAN message"""
        data = bytes(msg.data)
        motor_id = msg.arbitration_id & 0x00000FF
        if motor_id == self.motor.motor_id:
            self.motor._update_state_async(self.canman.parse_servo_message(data))


# Simplified unit conversion functions
def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians"""
    return degrees * np.pi / 180.0


def radians_to_degrees(radians: float) -> float:
    """Convert radians to degrees"""
    return radians * 180.0 / np.pi


def erpm_to_rad_per_sec(erpm: float, num_pole_pairs: int) -> float:
    """Convert ERPM to rad/s"""
    return erpm * 2 * np.pi / (60 * num_pole_pairs)


def rad_per_sec_to_erpm(rad_per_sec: float, num_pole_pairs: int) -> float:
    """Convert rad/s to ERPM"""
    return rad_per_sec * 60 * num_pole_pairs / (2 * np.pi)


# Control mode configuration
def _servo_position_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.POSITION


def _servo_position_mode_exit(actuator: "TMotorServoActuator") -> None:
    pass  # servo mode handles automatically


def _servo_current_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.CURRENT


def _servo_current_mode_exit(actuator: "TMotorServoActuator") -> None:
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_current(actuator.motor_id, 0.0)


def _servo_velocity_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.VELOCITY


def _servo_velocity_mode_exit(actuator: "TMotorServoActuator") -> None:
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_velocity(actuator.motor_id, 0.0)


def _servo_idle_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.IDLE


def _servo_idle_mode_exit(actuator: "TMotorServoActuator") -> None:
    pass


TMOTOR_SERVO_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    POSITION=ControlModeConfig(
        entry_callback=_servo_position_mode_entry,
        exit_callback=_servo_position_mode_exit,
        has_gains=False,  # servo mode handles internally
        max_gains=None,
    ),
    CURRENT=ControlModeConfig(
        entry_callback=_servo_current_mode_entry,
        exit_callback=_servo_current_mode_exit,
        has_gains=False,  # servo mode handles internally
        max_gains=None,
    ),
    VELOCITY=ControlModeConfig(
        entry_callback=_servo_velocity_mode_entry,
        exit_callback=_servo_velocity_mode_exit,
        has_gains=False,  # servo mode handles internally
        max_gains=None,
    ),
    VOLTAGE=ControlModeConfig(
        entry_callback=lambda actuator: setattr(actuator, "_servo_mode", ServoControlMode.IDLE),
        exit_callback=lambda actuator: None,
        has_gains=False,
        max_gains=None,
    ),
    IDLE=ControlModeConfig(
        entry_callback=_servo_idle_mode_entry,
        exit_callback=_servo_idle_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
)


class TMotorServoActuator(ActuatorBase):
    """

    Example:
        >>> with TMotorServoActuator(motor_type="AK80-9", motor_id=1) as motor:
        ...     motor.set_control_mode(CONTROL_MODES.CURRENT)
        ...     motor.set_output_torque(2.5)
        ...     motor.update()
    """

    def __init__(
        self,
        tag: str = "TMotorServoActuator",
        motor_type: str = "AK80-9",
        motor_id: int = 1,
        gear_ratio: float = 1.0,
        frequency: int = 500,
        offline: bool = False,
        max_temperature: float = 80.0,
        **kwargs: Any,
    ) -> None:
        """
        Initialize TMotor servo actuator

        Args:
            tag: actuator identifier
            motor_type: motor model (AK80-9, AK10-9)
            motor_id: CAN ID
            gear_ratio: gear ratio
            frequency: control frequency Hz
            offline: offline mode
            max_temperature: maximum temperature
        """
        # Validate motor type
        if motor_type not in SERVO_PARAMS:
            raise ValueError(f"Unsupported motor type: {motor_type}")

        super().__init__(
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=TMOTOR_SERVO_CONSTANTS,
            frequency=frequency,
            offline=offline,
            **kwargs,
        )

        # Motor configuration
        self.motor_type = motor_type
        self.motor_id = motor_id
        self.max_temperature = max_temperature
        self._motor_params = SERVO_PARAMS[motor_type]

        # CAN communication
        self._canman: Optional[CANManagerServo] = None
        if not self.is_offline:
            self._canman = CANManagerServo()
            self._canman.add_motor_listener(self)

        # State management
        self._motor_state = ServoMotorState()
        self._motor_state_async = ServoMotorState()
        self._servo_mode = ServoControlMode.IDLE

        # Error handling
        self._error_code: Optional[int] = None
        self._error_message: Optional[str] = None

        # Time management
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._last_command_time: Optional[float] = None

        # Thermal management
        self._thermal_model = ThermalModel(
            temp_limit_windings=self.max_winding_temperature,
            soft_border_C_windings=10.0,
            temp_limit_case=self.max_case_temperature,
            soft_border_C_case=10.0,
        )

        LOGGER.info(f"Initialized TMotor servo: {self.motor_type} ID:{self.motor_id}")

    @property
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        return TMOTOR_SERVO_CONTROL_MODE_CONFIGS

    def device_info_string(self) -> str:
        return f"{self.motor_type} ID:{self.motor_id}"

    @check_actuator_connection
    def start(self) -> None:
        """Start motor"""
        LOGGER.info(f"Starting {self.device_info_string()}")

        if not self.is_offline and self._canman:
            self._canman.power_on(self.motor_id)

        self._is_open = True
        self._is_streaming = True

    @check_actuator_stream
    @check_actuator_open
    def stop(self) -> None:
        """Stop motor"""
        LOGGER.info(f"Stopping {self.device_info_string()}")

        if not self.is_offline and self._canman:
            self._canman.power_off(self.motor_id)

        self._is_open = False
        self._is_streaming = False

    def update(self) -> None:
        """Update motor state"""
        # Temperature check
        if self.case_temperature > self.max_temperature:
            raise RuntimeError(f"Temperature {self.case_temperature}°C exceeds limit")

        # Update state
        self._motor_state.position = self._motor_state_async.position
        self._motor_state.velocity = self._motor_state_async.velocity
        self._motor_state.current = self._motor_state_async.current
        self._motor_state.temperature = self._motor_state_async.temperature
        self._motor_state.error = self._motor_state_async.error

        # Communication timeout check
        now = time.time()
        if (
            self._last_command_time is not None
            and (now - self._last_command_time) < 0.25
            and (now - self._last_update_time) > 0.1
        ):
            warnings.warn(f"No data from motor: {self.device_info_string()}", RuntimeWarning, stacklevel=2)

        self._last_update_time = now

    def _update_state_async(self, servo_state: ServoMotorState) -> None:
        """Asynchronously update state"""
        # Error handling
        if servo_state.error != 0:
            error_codes = cast(dict[int, str], SERVO_PARAMS["ERROR_CODES"])
            error_msg = error_codes.get(servo_state.error, "Unknown error")
            self._error_code = servo_state.error
            self._error_message = error_msg
            LOGGER.warning(f"Motor error {servo_state.error}: {error_msg}")
        else:
            self._error_code = None
            self._error_message = None

        # Calculate acceleration
        now = time.time()
        dt = now - self._last_update_time
        if dt > 0:
            motor_params = cast(dict[str, Any], self._motor_params)
            old_vel_rad_s = erpm_to_rad_per_sec(self._motor_state_async.velocity, motor_params["NUM_POLE_PAIRS"])
            new_vel_rad_s = erpm_to_rad_per_sec(servo_state.velocity, motor_params["NUM_POLE_PAIRS"])
            servo_state.acceleration = (new_vel_rad_s - old_vel_rad_s) / dt

        self._motor_state_async = servo_state

    @property
    def error_info(self) -> Optional[tuple[int, str]]:
        """Get error information"""
        if self._error_code is not None and self._error_message is not None:
            return (self._error_code, self._error_message)
        return None

    def home(
        self,
        homing_voltage: int = 0,
        homing_frequency: Optional[int] = None,
        homing_direction: int = 0,
        output_position_offset: float = 0.0,
        current_threshold: int = 0,
        velocity_threshold: float = 0.0,
    ) -> None:
        """Home motor"""
        if not self.is_offline and self._canman:
            self._canman.set_origin(self.motor_id, 1)
            time.sleep(0.1)
        self._is_homed = True
        LOGGER.info(f"Homed {self.device_info_string()}")

    # ============ Control Interface ============

    def set_motor_voltage(self, value: float) -> None:
        """Set motor voltage (not directly supported in servo mode)"""
        LOGGER.warning("Voltage control not supported in servo mode")

    def set_motor_current(self, value: float) -> None:
        """Set motor current"""
        if not self.is_offline and self._canman:
            self._canman.set_current(self.motor_id, value)
            self._last_command_time = time.time()

    def set_motor_position(self, value: float) -> None:
        """Set motor position (radians)"""
        position_deg = radians_to_degrees(value)
        if not self.is_offline and self._canman:
            self._canman.set_position(self.motor_id, position_deg)
            self._last_command_time = time.time()

    def set_motor_torque(self, value: float) -> None:
        """Set motor torque (Nm) - core functionality as requested by user"""
        # Torque to current: T = I * Kt
        current = value / self._motor_params["Kt_actual"]
        self.set_motor_current(current)

    def set_output_torque(self, value: float) -> None:
        """Set output torque (Nm) - core functionality as requested by user"""
        # Output torque to motor torque: T_motor = T_output * gear_ratio
        motor_torque = value * self.gear_ratio
        self.set_motor_torque(motor_torque)

    def set_motor_velocity(self, value: float) -> None:
        """Set motor velocity (rad/s)"""
        motor_params = cast(dict[str, Any], self._motor_params)
        velocity_erpm = rad_per_sec_to_erpm(value, motor_params["NUM_POLE_PAIRS"])
        if not self.is_offline and self._canman:
            self._canman.set_velocity(self.motor_id, velocity_erpm)
            self._last_command_time = time.time()

    def set_output_velocity(self, value: float) -> None:
        """Set output velocity (rad/s)"""
        motor_velocity = value * self.gear_ratio
        self.set_motor_velocity(motor_velocity)

    # ============ Unsupported PID Functions - TMotor Servo Mode Handles All Control Loops Internally ============

    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """TMotor servo mode does not support external current PID gains - motor handles current control internally"""
        raise NotImplementedError(
            "TMotor servo mode does not support external current PID gains. "
            "The motor handles current control internally."
        )

    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """TMotor servo mode does not support external position PID gains - motor handles position control internally"""
        raise NotImplementedError(
            "TMotor servo mode does not support external position PID gains. "
            "The motor handles position control internally."
        )

    def set_impedance_gains(self, kp: float, ki: float, kd: float, k: float, b: float, ff: float) -> None:
        """TMotor servo mode does not support impedance control"""
        raise NotImplementedError(
            "TMotor servo mode does not support impedance control. "
            "Use position, velocity, or current control modes instead."
        )

    # ============ State Properties ============

    @property
    def motor_position(self) -> float:
        """Motor position (radians)"""
        return degrees_to_radians(self._motor_state.position)

    @property
    def output_position(self) -> float:
        """Output position (radians)"""
        return self.motor_position / self.gear_ratio

    @property
    def motor_velocity(self) -> float:
        """Motor velocity (rad/s)"""
        motor_params = cast(dict[str, Any], self._motor_params)
        return erpm_to_rad_per_sec(self._motor_state.velocity, motor_params["NUM_POLE_PAIRS"])

    @property
    def output_velocity(self) -> float:
        """Output velocity (rad/s)"""
        return self.motor_velocity / self.gear_ratio

    @property
    def motor_voltage(self) -> float:
        """Motor voltage (estimated)"""
        return 24.0  # Cannot get directly in servo mode

    @property
    def motor_current(self) -> float:
        """Motor current (A)"""
        return self._motor_state.current

    @property
    def motor_torque(self) -> float:
        """Motor torque (Nm)"""
        motor_params = cast(dict[str, Any], self._motor_params)
        return self.motor_current * cast(float, motor_params["Kt_actual"])

    @property
    def output_torque(self) -> float:
        """Output torque (Nm)"""
        return self.motor_torque / self.gear_ratio

    @property
    def case_temperature(self) -> float:
        """Case temperature (°C)"""
        return self._motor_state.temperature

    @property
    def winding_temperature(self) -> float:
        """Winding temperature (°C)"""
        return cast(float, getattr(self._thermal_model, "T_w", self.case_temperature))

    def __str__(self) -> str:
        """State string"""
        return (
            f"{self.device_info_string()} | "
            f"Pos: {self.output_position:.3f}rad | "
            f"Vel: {self.output_velocity:.3f}rad/s | "
            f"Torque: {self.output_torque:.3f}Nm | "
            f"Current: {self.motor_current:.3f}A | "
            f"Temp: {self.case_temperature:.1f}°C"
        )


if __name__ == "__main__":
    print("TMotor Current Loop Control Example")

    try:
        actuator = TMotorServoActuator(motor_type="AK80-9", motor_id=1, offline=True)

        with actuator as motor:
            # Explicit type: motor is actually TMotorServoActuator
            motor_actuator = cast(TMotorServoActuator, motor)

            print(f"Motor initialized: {motor_actuator.device_info_string()}")

            # Home and set current control mode
            motor_actuator.home()
            motor_actuator.set_control_mode(CONTROL_MODES.CURRENT)
            print("Current loop mode activated")

            # Send 15Nm torque command
            target_torque = 15.0  # Nm
            motor_actuator.set_output_torque(target_torque)
            print(f"Sending {target_torque}Nm torque command to motor")
            print()

            # Create real-time reading loop
            loop = SoftRealtimeLoop(dt=0.1, report=False, fade=0)  # 10Hz, slower for observation
            print("📊 Reading motor parameters...")

            for t in loop:
                if t > 5.0:  # Run for 5 seconds
                    break

                # Update motor state
                motor_actuator.update()

                # Read motor parameters
                motor_angle = motor_actuator.motor_position  # motor angle (rad)
                output_angle = motor_actuator.output_position  # output angle (rad)
                motor_velocity = motor_actuator.motor_velocity  # motor velocity (rad/s)
                output_velocity = motor_actuator.output_velocity  # output velocity (rad/s)
                motor_current = motor_actuator.motor_current  # motor current (A)
                motor_torque = motor_actuator.motor_torque  # motor torque (Nm)
                output_torque = motor_actuator.output_torque  # output torque (Nm)
                temperature = motor_actuator.case_temperature  # temperature (°C)
                motor_voltage = motor_actuator.motor_voltage  # voltage (V)

                # Check for errors
                error_status = "OK"
                if motor_actuator.error_info:
                    error_code, error_msg = motor_actuator.error_info
                    error_status = f"Error{error_code}: {error_msg}"

                # Display complete status - every 0.5 seconds
                if int(t * 2) % 1 == 0:
                    print(f"Time: {t:4.1f}s")
                    print(
                        f"  Torque: Cmd={target_torque:6.2f}Nm | "
                        f"Motor={motor_torque:6.2f}Nm | Output={output_torque:6.2f}Nm"
                    )
                    print(
                        f"  Angle:  Motor={motor_angle:8.4f}rad ({np.degrees(motor_angle):7.2f}°) | "
                        f"Output={output_angle:8.4f}rad ({np.degrees(output_angle):7.2f}°)"
                    )
                    print(f"  Speed:  Motor={motor_velocity:8.4f}rad/s | Output={output_velocity:8.4f}rad/s")
                    print(
                        f"  Current: {motor_current:6.2f}A | "
                        f"Voltage: {motor_voltage:5.1f}V | Temp: {temperature:4.1f}°C"
                    )
                    print(f"  Status: {error_status}")
                    print("-" * 80)

            # Safe stop
            motor_actuator.set_output_torque(0.0)
            motor_actuator.update()
            print("Motor safely stopped")
            print()

            # Display final state
            print("   Final Motor State:")
            print(
                f"  Final Position: {np.degrees(motor_actuator.output_position):.2f}° "
                f"({motor_actuator.output_position:.4f} rad)"
            )
            print(f"  Final Torque: {motor_actuator.output_torque:.2f} Nm")
            print(f"  Final Current: {motor_actuator.motor_current:.2f} A")
            print(f"  Final Temperature: {motor_actuator.case_temperature:.1f}°C")

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()

    print("Current loop control example completed!")
