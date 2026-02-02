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
from opensourceleg.logging import LOGGER
from opensourceleg.math import ThermalModel
from opensourceleg.utilities import SoftRealtimeLoop

# TMotor servo mode error codes
TMOTOR_ERROR_CODES: dict[int, str] = {
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
}

# TMotor CAN packet IDs
CAN_PACKET_ID = {
    "SET_DUTY": 0,
    "SET_CURRENT": 1,
    "SET_CURRENT_BRAKE": 2,
    "SET_RPM": 3,
    "SET_POS": 4,
    "SET_ORIGIN_HERE": 5,
    "SET_POS_SPD": 6,
}

# TMotor model specifications
TMOTOR_MODELS: dict[str, dict[str, Any]] = {
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
}

# TMotor servo mode constants (for ActuatorBase compatibility)
TMOTOR_SERVO_CONSTANTS = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=65536,  # Encoder counts per revolution (16-bit encoder)
    NM_PER_AMP=0.115,  # AK80-9 default
    NM_PER_RAD_TO_K=1e-9,  # small positive placeholder to satisfy validation
    NM_S_PER_RAD_TO_B=1e-9,  # small positive placeholder to satisfy validation
    MAX_CASE_TEMPERATURE=80.0,
    MAX_WINDING_TEMPERATURE=110.0,
    # Soft limits set 10°C below hard limits for safety margin
    WINDING_SOFT_LIMIT=100.0,
    CASE_SOFT_LIMIT=70.0,
)


@dataclass
class ServoMotorState:
    """Motor state data structure"""

    position: float = 0.0  # degrees
    velocity: float = 0.0  # ERPM
    current: float = 0.0  # amps
    temperature: float = 0.0  # celsius
    error: int = 0


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
            # NOTE: CAN interface must be configured before running this code.
            # Please run the following commands before using TMotor servo mode:
            # sudo /sbin/ip link set can0 down
            # sudo /sbin/ip link set can0 up type can bitrate 1000000
            # sudo ifconfig can0 txqueuelen 1000

            self.bus = can.interface.Bus(channel="can0", bustype="socketcan")
            self.notifier = can.Notifier(bus=self.bus, listeners=[])

            LOGGER.info(f"CAN bus connected: {self.bus}")
            self._initialized = True

        except Exception as e:
            LOGGER.error(f"CAN bus initialization failed: {e}")
            LOGGER.error("Please ensure CAN interface is configured. Run:")
            LOGGER.error("sudo /sbin/ip link set can0 down")
            LOGGER.error("sudo /sbin/ip link set can0 up type can bitrate 1000000")
            LOGGER.error("sudo ifconfig can0 txqueuelen 1000")
            raise RuntimeError("CAN bus initialization failed. Please configure CAN interface first.") from e

    def __del__(self) -> None:
        try:
            if hasattr(self, "bus"):
                self.bus.shutdown()
        except Exception as e:
            LOGGER.warning(f"Error shutting down CAN bus: {e}")

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
        message_id = (CAN_PACKET_ID["SET_CURRENT"] << 8) | controller_id
        self.send_message(message_id, buffer, len(buffer))

    def set_velocity(self, controller_id: int, velocity: float) -> None:
        """Send velocity control command"""
        buffer = self._pack_int32(int(velocity))
        message_id = (CAN_PACKET_ID["SET_RPM"] << 8) | controller_id
        self.send_message(message_id, buffer, len(buffer))

    def set_position(self, controller_id: int, position: float) -> None:
        """Send position control command"""
        # NOTE(IMPORTANT): TMotor spec uses 1,000,000 scale (0.000001° resolution) per documentation
        # Current implementation uses 10 scale (0.1° resolution) for simplicity
        # To enable high-precision position control, change to: int(position * 1000000.0)
        buffer = self._pack_int32(int(position * 10.0))  # 0.1 degree resolution
        message_id = (CAN_PACKET_ID["SET_POS"] << 8) | controller_id
        self.send_message(message_id, buffer, len(buffer))

    def set_origin(self, controller_id: int, mode: int = 1) -> None:
        """Set motor origin"""
        buffer = [mode]
        message_id = (CAN_PACKET_ID["SET_ORIGIN_HERE"] << 8) | controller_id
        self.send_message(message_id, buffer, len(buffer))

    def set_control_mode(self, controller_id: int, mode: int) -> None:
        """Send control mode switch command"""
        buffer = [mode]
        message_id = (7 << 8) | controller_id  # Assume packet_id=7 for mode switching
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
        motor_cur = float(cur_int * 0.01)  # current (amps)
        motor_temp = float(data[6])  # temperature (celsius)
        motor_error = int(data[7])  # error code

        return ServoMotorState(motor_pos, motor_spd, motor_cur, motor_temp, motor_error)

    def add_motor_listener(self, motor: "TMotorServoActuator") -> None:
        """Add motor listener"""
        listener = MotorListener(self, motor)
        self.notifier.add_listener(listener)

    def enable_debug(self, enable: bool = True) -> None:
        """Enable/disable debug mode"""
        self.debug = enable
        if enable:
            LOGGER.info("CAN debug mode enabled")


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


def _wait_for_mode_switch(actuator: "TMotorServoActuator", timeout: float = 0.2) -> None:
    """Wait for control mode switch confirmation with timeout"""
    start_time = time.time()
    poll_interval = 0.01  # 10ms polling

    while time.time() - start_time < timeout:
        actuator.update()  # Read status
        if actuator._motor_state.error == 0:
            LOGGER.debug(f"Mode switch confirmed after {time.time() - start_time:.3f} seconds")
            return
        time.sleep(poll_interval)

    # If we reach here, mode switch may not be confirmed but we continue
    LOGGER.warning(f"Mode switch confirmation timeout after {timeout} seconds")


# Control mode configuration
def _servo_position_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.POSITION
    # Send actual mode switch command to motor
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, ServoControlMode.POSITION.value)
        _wait_for_mode_switch(actuator)


def _servo_position_mode_exit(actuator: "TMotorServoActuator") -> None:
    pass  # servo mode handles automatically


def _servo_current_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.CURRENT
    # Send actual mode switch command to motor
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, ServoControlMode.CURRENT.value)
        _wait_for_mode_switch(actuator)


def _servo_current_mode_exit(actuator: "TMotorServoActuator") -> None:
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_current(actuator.motor_id, 0.0)


def _servo_velocity_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.VELOCITY
    # Send actual mode switch command to motor
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, ServoControlMode.VELOCITY.value)
        _wait_for_mode_switch(actuator)


def _servo_velocity_mode_exit(actuator: "TMotorServoActuator") -> None:
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_velocity(actuator.motor_id, 0.0)


def _servo_idle_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.IDLE
    # Send actual mode switch command to motor
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, ServoControlMode.IDLE.value)
        _wait_for_mode_switch(actuator)


def _servo_idle_mode_exit(actuator: "TMotorServoActuator") -> None:
    pass


def _impedance_not_supported(actuator: "TMotorServoActuator") -> None:
    """Log that impedance control is not supported"""
    LOGGER.error(
        "TMotor servo mode does not support impedance control. "
        "Use position, velocity, or current control modes instead."
    )
    raise NotImplementedError(
        "TMotor servo mode does not support impedance control. "
        "Use position, velocity, or current control modes instead."
    )


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
    # Explicitly define IMPEDANCE as not supported
    IMPEDANCE=ControlModeConfig(
        entry_callback=_impedance_not_supported,
        exit_callback=lambda actuator: None,
        has_gains=False,
        max_gains=None,
    ),
)


class TMotorServoActuator(ActuatorBase):
    """
    TMotor servo mode actuator for AK series motors.

    Important: Before using this actuator, the CAN interface must be configured:
        sudo /sbin/ip link set can0 down
        sudo /sbin/ip link set can0 up type can bitrate 1000000

    For detailed setup instructions, see docs/tutorials/actuators/tmotor_servo_setup.md

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
        motor_id: int = 104,
        gear_ratio: float = 1.0,
        frequency: int = 1000,
        offline: bool = False,
        max_temperature: float = 80.0,
        current_mode: str = "driver",
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
            current_mode: current convention ('driver', 'amplitude-invariant', 'power-invariant')
                - 'driver': use driver's native current (default)
                - 'amplitude-invariant': true phase current convention
                - 'power-invariant': power-invariant current convention
        """
        # Validate motor type
        if motor_type not in TMOTOR_MODELS:
            raise ValueError(f"Unsupported motor type: {motor_type}")

        # Validate current mode
        if current_mode not in ["driver", "amplitude-invariant", "power-invariant"]:
            raise ValueError(
                f"Invalid current_mode: {current_mode}. Must be 'driver', 'amplitude-invariant', or 'power-invariant'"
            )

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
        self._motor_params = TMOTOR_MODELS[motor_type]
        self.current_mode = current_mode

        # Current conversion factors
        # Physical: I_drv (line current) = K * I_user
        # Code: driver_current = user_current / self._current_scale
        # So: I_drv = I_user / scale, which means scale = I_user / I_drv = 1 / K
        # Kt relationship: τ = Kt_drv * I_drv = Kt_user * I_user
        # So: Kt_user = Kt_drv * (I_drv / I_user) = Kt_drv * K
        if current_mode == "amplitude-invariant":
            # I_drv (line) = √3 * I_phase, so scale = I_phase / I_drv = 1/√3
            # Kt_amp = Kt_drv * √3
            self._current_scale = 1.0 / np.sqrt(3.0)
            self._kt_scale = np.sqrt(3.0)
        elif current_mode == "power-invariant":
            # I_drv (line) = √2 * I_pwr, so scale = I_pwr / I_drv = 1/√2
            # Kt_pwr = Kt_drv * √2
            self._current_scale = 1.0 / np.sqrt(2.0)
            self._kt_scale = np.sqrt(2.0)
        else:  # driver
            self._current_scale = 1.0
            self._kt_scale = 1.0

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
            motor_constants=self._MOTOR_CONSTANTS,
            actuator_tag=self.tag,
        )

        LOGGER.info(f"Initialized TMotor servo: {self.motor_type} ID:{self.motor_id} (current_mode: {current_mode})")

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

            # Poll for motor readiness (max 1 second timeout)
            start_time = time.time()
            timeout = 1.0
            poll_interval = 0.01  # 10ms polling interval
            motor_ready = False

            while time.time() - start_time < timeout:
                # The motor should start sending status messages after power on
                # Check if we've received valid data by checking if position/velocity/current are non-zero
                # or if the motor_state has been updated (timestamp check could be added)
                if (
                    self._motor_state_async.position != 0.0
                    or self._motor_state_async.velocity != 0.0
                    or self._motor_state_async.current != 0.0
                    or self._motor_state_async.temperature > 0.0
                ):
                    motor_ready = True
                    LOGGER.debug(f"Motor ready after {time.time() - start_time:.3f} seconds")
                    break
                time.sleep(poll_interval)

            if not motor_ready:
                # Fall back to minimum wait time if no status received
                LOGGER.warning("No status received from motor, using fallback delay")
                time.sleep(0.1)  # Minimum wait time

            # Set initial control mode to IDLE
            self._canman.set_control_mode(self.motor_id, ServoControlMode.IDLE.value)

            # Poll for mode switch confirmation (max 200ms timeout)
            mode_start_time = time.time()
            mode_timeout = 0.2

            while time.time() - mode_start_time < mode_timeout:
                self.update()  # Read status
                if self._motor_state.error == 0:
                    LOGGER.debug(f"Mode switch confirmed after {time.time() - mode_start_time:.3f} seconds")
                    break
                time.sleep(poll_interval)

            # Final status check
            if self._motor_state.error != 0:
                raise RuntimeError(f"Motor startup failed with error: {self._motor_state.error}")

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
        # More detailed error handling
        if servo_state.error != 0:
            error_codes = TMOTOR_ERROR_CODES
            error_msg = error_codes.get(servo_state.error, f"Unknown error code: {servo_state.error}")
            self._error_code = servo_state.error
            self._error_message = error_msg

            # Take different actions based on error type
            if servo_state.error in [1, 2]:  # Voltage errors
                LOGGER.error(f"Voltage error {servo_state.error}: {error_msg}")
            elif servo_state.error in [4, 5, 6]:  # Overcurrent or overtemperature
                LOGGER.critical(f"Critical error {servo_state.error}: {error_msg}")
                # Auto-stop motor
                if self._canman:
                    self._canman.set_current(self.motor_id, 0.0)
            else:
                LOGGER.warning(f"Motor error {servo_state.error}: {error_msg}")
        else:
            self._error_code = None
            self._error_message = None

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
        """Set motor current with clamping to motor limits"""
        if not self.is_offline and self._canman:
            # Convert from desired current convention to driver current
            driver_current = value / self._current_scale

            # Get current limits from motor parameters (these are in driver convention)
            motor_params = cast(dict[str, Any], self._motor_params)
            max_current = motor_params["Curr_max"] / 1000.0  # Convert from protocol units to amps
            min_current = motor_params["Curr_min"] / 1000.0  # Convert from protocol units to amps

            # Clamp current to safe limits
            clamped_driver_current = np.clip(driver_current, min_current, max_current)

            # Log warning if clamping occurred (show in user's convention)
            if driver_current != clamped_driver_current:
                clamped_user_current = clamped_driver_current * self._current_scale
                LOGGER.warning(
                    f"Current command {value:.2f}A clamped to {clamped_user_current:.2f}A "
                    f"(limits: [{min_current * self._current_scale:.1f}, {max_current * self._current_scale:.1f}]A)"
                )

            self._canman.set_current(self.motor_id, clamped_driver_current)
            self._last_command_time = time.time()

    def set_motor_position(self, value: float) -> None:
        """Set motor position (radians) with clamping to motor limits"""
        position_deg = radians_to_degrees(value)

        if not self.is_offline and self._canman:
            # Get position limits from motor parameters
            motor_params = cast(dict[str, Any], self._motor_params)
            max_position = motor_params["P_max"] / 10.0  # Convert from protocol units to degrees
            min_position = motor_params["P_min"] / 10.0  # Convert from protocol units to degrees

            # Clamp position to safe limits
            clamped_position = np.clip(position_deg, min_position, max_position)

            # Log warning if clamping occurred
            if position_deg != clamped_position:
                LOGGER.warning(
                    f"Position command {position_deg:.1f}° clamped to {clamped_position:.1f}° "
                    f"(limits: [{min_position:.0f}, {max_position:.0f}]°)"
                )

            self._canman.set_position(self.motor_id, clamped_position)
            self._last_command_time = time.time()

    def set_motor_torque(self, value: float) -> None:
        """Set motor torque (Nm) - core functionality as requested by user"""
        # Torque to current: T = I * Kt
        # Kt_user = Kt_drv * kt_scale
        kt_user = self._motor_params["Kt_actual"] * self._kt_scale
        current = value / kt_user
        self.set_motor_current(current)

    def set_output_torque(self, value: float) -> None:
        """Set output torque (Nm) - core functionality as requested by user"""
        # Output torque to motor torque: T_motor = T_output * gear_ratio
        motor_torque = value * self.gear_ratio
        self.set_motor_torque(motor_torque)

    def set_motor_velocity(self, value: float) -> None:
        """Set motor velocity (rad/s) with clamping to motor limits"""
        motor_params = cast(dict[str, Any], self._motor_params)
        velocity_erpm = rad_per_sec_to_erpm(value, self.num_pole_pairs)

        if not self.is_offline and self._canman:
            # Get velocity limits from motor parameters
            max_velocity = motor_params["V_max"]  # Already in ERPM
            min_velocity = motor_params["V_min"]  # Already in ERPM

            # Clamp velocity to safe limits
            clamped_velocity = np.clip(velocity_erpm, min_velocity, max_velocity)

            # Log warning if clamping occurred
            if velocity_erpm != clamped_velocity:
                LOGGER.warning(
                    f"Velocity command {velocity_erpm:.0f} ERPM clamped to {clamped_velocity:.0f} ERPM "
                    f"(limits: [{min_velocity}, {max_velocity}] ERPM)"
                )

            self._canman.set_velocity(self.motor_id, clamped_velocity)
            self._last_command_time = time.time()

    def set_motor_impedance(self, position: float, velocity: float, kp: float, kd: float, torque_ff: float) -> None:
        """TMotor servo mode does not support impedance control"""
        LOGGER.error(
            "TMotor servo mode does not support impedance control. "
            "Use position, velocity, or current control modes instead."
        )
        raise NotImplementedError("TMotor servo mode does not support impedance control.")

    def set_output_impedance(self, position: float, velocity: float, kp: float, kd: float, torque_ff: float) -> None:
        """TMotor servo mode does not support impedance control"""
        LOGGER.error(
            "TMotor servo mode does not support impedance control. "
            "Use position, velocity, or current control modes instead."
        )
        raise NotImplementedError("TMotor servo mode does not support impedance control.")

    def set_output_velocity(self, value: float) -> None:
        """Set output velocity (rad/s)"""
        motor_velocity = value * self.gear_ratio
        self.set_motor_velocity(motor_velocity)

    # ============ Unsupported PID Functions - TMotor Servo Mode Handles All Control Loops Internally ============

    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """TMotor servo mode does not support external current PID gains - motor handles current control internally"""
        LOGGER.debug(
            "TMotor servo mode handles current control internally. " "External current PID gains are not used."
        )
        # Motor handles current control internally, no action needed

    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """TMotor servo mode does not support external position PID gains - motor handles position control internally"""
        LOGGER.debug(
            "TMotor servo mode handles position control internally. " "External position PID gains are not used."
        )
        # Motor handles position control internally, no action needed

    def set_impedance_gains(self, kp: float, ki: float, kd: float, k: float, b: float, ff: float) -> None:
        """TMotor servo mode does not support impedance control"""
        LOGGER.debug(
            "TMotor servo mode does not support impedance control. "
            "Use position, velocity, or current control modes instead."
        )
        # Impedance control not supported in servo mode, no action needed

    def _set_impedance_gains(self, k: float, b: float) -> None:
        """Internal method for impedance gains - not supported in TMotor servo mode"""
        LOGGER.debug("TMotor servo mode handles control internally. " "Impedance gains are not used.")
        # Motor handles control internally, no action needed

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
        return erpm_to_rad_per_sec(self._motor_state.velocity, self.num_pole_pairs)

    @property
    def output_velocity(self) -> float:
        """Output velocity (rad/s)"""
        return self.motor_velocity / self.gear_ratio

    @property
    def num_pole_pairs(self) -> int:
        """Number of motor pole pairs"""
        motor_params = cast(dict[str, Any], self._motor_params)
        return motor_params["NUM_POLE_PAIRS"]

    @property
    def motor_voltage(self) -> float:
        """Motor voltage - not available in servo mode"""
        raise NotImplementedError(
            "Motor voltage reading is not available in TMotor servo mode. "
            "The motor does not provide voltage feedback through the CAN protocol."
        )

    @property
    def motor_current(self) -> float:
        """Motor current (A) - converted to selected current convention"""
        return self._motor_state.current * self._current_scale

    @property
    def motor_torque(self) -> float:
        """Motor torque (Nm)"""
        motor_params = cast(dict[str, Any], self._motor_params)
        kt_user = cast(float, motor_params["Kt_actual"]) * self._kt_scale
        return self.motor_current * kt_user

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

        with actuator:
            print(f"Motor initialized: {actuator.device_info_string()}")

            # Home and set current control mode
            actuator.home()
            actuator.set_control_mode(CONTROL_MODES.CURRENT)
            print("Current loop mode activated")

            # Send 15Nm torque command
            target_torque = 15.0  # Nm
            actuator.set_output_torque(target_torque)
            print(f"Sending {target_torque}Nm torque command to motor")
            print()

            # Create real-time reading loop
            loop = SoftRealtimeLoop(dt=0.1, report=False, fade=0)  # 10Hz, slower for observation
            print("📊 Reading motor parameters...")

            for t in loop:
                if t > 5.0:  # Run for 5 seconds
                    break

                # Update motor state
                actuator.update()

                # Read motor parameters
                motor_angle = actuator.motor_position  # motor angle (rad)
                output_angle = actuator.output_position  # output angle (rad)
                motor_velocity = actuator.motor_velocity  # motor velocity (rad/s)
                output_velocity = actuator.output_velocity  # output velocity (rad/s)
                motor_current = actuator.motor_current  # motor current (A)
                motor_torque = actuator.motor_torque  # motor torque (Nm)
                output_torque = actuator.output_torque  # output torque (Nm)
                temperature = actuator.case_temperature  # temperature (°C)
                # motor_voltage = actuator.motor_voltage  # voltage (V) - Not available in servo mode

                # Check for errors
                error_status = "OK"
                if actuator.error_info:
                    error_code, error_msg = actuator.error_info
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
                    print(f"  Current: {motor_current:6.2f}A | " f"Temp: {temperature:4.1f}°C")
                    print(f"  Status: {error_status}")
                    print("-" * 80)

            # Safe stop
            actuator.set_output_torque(0.0)
            actuator.update()
            print("Motor safely stopped")
            print()

            # Display final state
            print("   Final Motor State:")
            print(
                f"  Final Position: {np.degrees(actuator.output_position):.2f}° "
                f"({actuator.output_position:.4f} rad)"
            )
            print(f"  Final Torque: {actuator.output_torque:.2f} Nm")
            print(f"  Final Current: {actuator.motor_current:.2f} A")
            print(f"  Final Temperature: {actuator.case_temperature:.1f}°C")

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()

    print("Current loop control example completed!")
