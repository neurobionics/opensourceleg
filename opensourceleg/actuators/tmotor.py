import time
import warnings
from dataclasses import dataclass
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
    "SET_MODE": 7,
}

# TMotor model specifications
TMOTOR_MODELS: dict[str, dict[str, Any]] = {
    "AK10-9": {
        "P_min": -3200,  # -3200 deg
        "P_max": 3200,  # 3200 deg
        "V_min": -100000,  # ERPM
        "V_max": 100000,  # ERPM
        "Curr_min": -60000,  # -60A
        "Curr_max": 60000,  # 60A
        "Kt_actual": 0.206,  # Nm/A
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,
    },
    "AK80-9": {
        "P_min": -3200,  # -3200 deg
        "P_max": 3200,  # 3200 deg
        "V_min": -32000,  # ERPM
        "V_max": 32000,  # ERPM
        "Curr_min": -60000,  # -60A
        "Curr_max": 60000,  # 60A
        "Kt_actual": 0.0952,  # Nm/A
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,
    },
}

# TMotor servo mode constants (for ActuatorBase compatibility)
TMOTOR_SERVO_CONSTANTS = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=65536,  # Encoder counts per revolution (16-bit encoder)
    NM_PER_AMP=0.095,  # Placeholder to satisfy validation
    MAX_CASE_TEMPERATURE=80.0,  # Temperature parameters are also set in the driver via R-Link
    MAX_WINDING_TEMPERATURE=110.0,  # Temperature parameters are also set in the driver via R-Link
    WINDING_SOFT_LIMIT=100.0,  # Soft limits set 10°C below hard limits for safety margin
    CASE_SOFT_LIMIT=70.0,
)


@dataclass
class ServoMotorState:
    """Motor state data structure"""

    position: float = 0.0  # degrees
    velocity: float = 0.0  # ERPM
    current: float = 0.0  # milliamps
    temperature: float = 0.0  # celsius
    error: int = 0


class CANManagerServo:
    """
    Manages CAN bus communication for T-Motor actuators operating in Servo Mode.

    This class implements the Singleton pattern to ensure a single point of control
    for the CAN interface. It handles the initialization of the socketcan bus, dispatches
    incoming messages to registered motor listeners, and provides
    utilities for packing and sending control commands (current, velocity, and position).

    Requirements:
        The CAN interface ('can0') must be configured in the OS before initializing
        this class. Typical configuration commands for Linux systems:

        $ sudo /sbin/ip link set can0 down
        $ sudo /sbin/ip link set can0 up type can bitrate 1000000
        $ sudo ifconfig can0 txqueuelen 1000

    Attributes:
        debug (bool): If true, logs detailed information about sent and received messages.
        bus (can.interface.Bus): The underlying python-can bus interface
        notifier (can.Notifier): Manages message listeners for asynchronous updates

    Examples:
        with CANManagerServo() as manager:
            manager.power_on(motor_id=1)
            manager.set_position(motor_id=1, position=90.0)
    """

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
            self.bus = can.interface.Bus(channel="can0", bustype="socketcan")
            self.notifier = can.Notifier(bus=self.bus, listeners=[])
            self._listeners = []  # Track active listeners
            LOGGER.info(f"CAN bus connected: {self.bus}")
            self._initialized = True

        except Exception as e:
            LOGGER.error(
                f"CAN bus initialization failed: {e}"
                "Please ensure CAN interface is configured. Run:"
                "sudo /sbin/ip link set can0 down"
                "sudo /sbin/ip link set can0 up type can bitrate 1000000"
                "sudo ifconfig can0 txqueuelen 1000"
            )
            raise RuntimeError("CAN bus initialization failed. Please configure CAN interface first.") from e

    def __enter__(self) -> None:
        """Context manager enter"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit"""
        self.close()

    def close(self) -> None:
        """
        Close CAN service
        """
        try:
            LOGGER.info("Closing CAN Manager...")

            # Stop and remove all listeners first
            if hasattr(self, "_listeners"):
                for listener in self._listeners:
                    try:
                        self.notifier.remove_listener(listener)
                    except Exception as e:
                        LOGGER.warning(f"Error removing listener: {e}")
                self._listeners.clear()

            # Stop the notifier
            if hasattr(self, "notifier"):
                self.notifier.stop()
                # Give notifier time to stop cleanly
                time.sleep(0.05)

            # Shutdown the bus
            if hasattr(self, "bus"):
                self.bus.shutdown()

            LOGGER.info("CAN Manager closed successfully")

        except Exception as e:
            LOGGER.warning(f"Error closing CAN manager: {e}")

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

    def set_current(self, motor_id: int, current: float) -> None:
        """
        Send current control command
        Args:
            motor_id (int): CAN motor ID
            current (float): motor current in mA
        Returns:
            None
        """
        buffer = self._pack_int32(int(current))
        message_id = (CAN_PACKET_ID["SET_CURRENT"] << 8) | motor_id
        self.send_message(message_id, buffer, len(buffer))

    def set_velocity(self, motor_id: int, velocity: float) -> None:
        """
        Send velocity control command
        Args:
            motor_id (int): CAN motor ID
            velocity (float): motor velocity in erpm
        Returns:
            None
        """
        buffer = self._pack_int32(int(velocity))
        message_id = (CAN_PACKET_ID["SET_RPM"] << 8) | motor_id
        self.send_message(message_id, buffer, len(buffer))

    def set_position(self, motor_id: int, position: float) -> None:
        """
        Send position control command
        Args:
            motor_id (int): CAN motor ID
            position (float): motor position in degrees
        Returns:
            None
        """
        buffer = self._pack_int32(int(position * 10000.0))
        message_id = (CAN_PACKET_ID["SET_POS"] << 8) | motor_id
        self.send_message(message_id, buffer, len(buffer))

    def set_origin(self, motor_id: int, mode: int = 1) -> None:
        """
        Set motor origin
        Args:
            motor_id (int): CAN motor ID
            mode (int): 0 represents setting a temporary origin (erased upon power loss)
                        1 represents setting a permanent origin (parameters are automatically saved)
        Returns:
            None
        """
        buffer = [mode]
        message_id = (CAN_PACKET_ID["SET_ORIGIN_HERE"] << 8) | motor_id
        self.send_message(message_id, buffer, len(buffer))

    def set_control_mode(self, motor_id: int, mode: int) -> None:
        """
        Send control mode switch command
        Args:
            motor_id (int): CAN motor ID
            mode (int): represents control mode (e.g., position, current, velocity, idle)
        Returns:
            None
        """
        buffer = [mode]
        message_id = (CAN_PACKET_ID["SET_MODE"] << 8) | motor_id
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
        motor_cur = float(cur_int * 10)  # current (mA)
        motor_temp = float(int.from_bytes(data[6:7], byteorder="big", signed=True))  # temperature (celsius)
        motor_error = int(data[7])  # error code

        return ServoMotorState(motor_pos, motor_spd, motor_cur, motor_temp, motor_error)

    def add_motor_listener(self, motor: "TMotorServoActuator") -> None:
        """Add motor listener"""
        listener = MotorListener(self, motor)
        self.notifier.add_listener(listener)
        self._listeners.append(listener)
        LOGGER.debug(f"Added listener for motor ID {motor.motor_id}")

    def remove_motor_listener(self, motor: "TMotorServoActuator") -> None:
        """Remove motor listener"""
        for listener in self._listeners[:]:  # Create a copy to iterate
            if listener.motor == motor:
                try:
                    self.notifier.remove_listener(listener)
                    self._listeners.remove(listener)
                    LOGGER.debug(f"Removed listener for motor ID {motor.motor_id}")
                except Exception as e:
                    LOGGER.warning(f"Error removing listener for motor {motor.motor_id}: {e}")

    def enable_debug(self, enable: bool = True) -> None:
        """Enable/disable debug mode"""
        self.debug = enable
        if enable:
            LOGGER.info("CAN debug mode enabled")


class MotorListener(can.Listener):
    """
    CAN message listener.

    This class monitors the CAN bus for messages addressed to a specific motor ID.
    When a relevant message is received, it parses the data using the CAN manager
    and updates the state of the associated motor instance asynchronously.

    Args:
        canman (CANManagerServo): The CAN manager instance responsible for parsing
            raw message data into servo states.
        motor (TMotorServoActuator): The specific motor instance that this
            listener monitors and updates.
    """

    def __init__(self, canman: CANManagerServo, motor: "TMotorServoActuator") -> None:
        self.canman = canman
        self.motor = motor

    def on_message_received(self, msg: can.Message) -> None:
        """
        Callback triggered when a CAN message is received.

        Checks if the message's arbitration ID matches the assigned motor's ID.
        If it matches, the message data is parsed and the motor's state is updated.

        Args:
            msg (can.Message): The received CAN message
        """
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
    start_time = time.monotonic()
    poll_interval = 0.01  # 10ms polling

    while time.monotonic() - start_time < timeout:
        actuator.update()  # Read status
        if actuator._motor_state.error == 0:
            LOGGER.debug(f"Mode switch confirmed after {time.monotonic() - start_time:.3f} seconds")
            return
        time.sleep(poll_interval)

    # If we reach here, mode switch may not be confirmed but we continue
    LOGGER.warning(f"Mode switch confirmation timeout after {timeout} seconds")


# ============ Control mode configuration ============


def _servo_position_mode_entry(actuator: "TMotorServoActuator") -> None:
    LOGGER.debug(msg=f"[{actuator.__str__()}] Entering Position control mode.")
    mode_id = 4
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, mode_id)
        _wait_for_mode_switch(actuator)


def _servo_position_mode_exit(actuator: "TMotorServoActuator") -> None:
    LOGGER.debug(msg=f"[{actuator.__str__()}] Exiting Position control mode.")


def _servo_current_mode_entry(actuator: "TMotorServoActuator") -> None:
    LOGGER.debug(msg=f"[{actuator.__str__()}] Entering Current control mode.")
    mode_id = 1
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, mode_id)
        _wait_for_mode_switch(actuator)


def _servo_current_mode_exit(actuator: "TMotorServoActuator") -> None:
    LOGGER.debug(msg=f"[{actuator.__str__()}] Exiting Current control mode.")
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_current(actuator.motor_id, 0.0)


def _servo_velocity_mode_entry(actuator: "TMotorServoActuator") -> None:
    LOGGER.debug(msg=f"[{actuator.__str__()}] Entering Velocity control mode.")
    mode_id = 3
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, mode_id)
        _wait_for_mode_switch(actuator)


def _servo_velocity_mode_exit(actuator: "TMotorServoActuator") -> None:
    LOGGER.debug(msg=f"[{actuator.__str__()}] Exiting Velocity control mode.")
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_velocity(actuator.motor_id, 0.0)


def _servo_idle_mode_entry(actuator: "TMotorServoActuator") -> None:
    LOGGER.debug(msg=f"[{actuator.__str__()}] Entering Idle control mode.")
    mode_id = 7
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, mode_id)
        _wait_for_mode_switch(actuator)


def _servo_idle_mode_exit(actuator: "TMotorServoActuator") -> None:
    LOGGER.debug(msg=f"[{actuator.__str__()}] Exiting Idle control mode.")


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
    IDLE=ControlModeConfig(
        entry_callback=_servo_idle_mode_entry,
        exit_callback=_servo_idle_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
    IMPEDANCE=None,  # IMPEDANCE mode not supported
    VOLTAGE=None,  # VOLTAGE mode not supported
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
        current_mode: str = "driver",
        thermal_current_scale: float = 1.0,
        **kwargs: Any,
    ) -> None:
        """
        Initialize TMotor servo actuator

        Args:
            tag: actuator identifier
            motor_type: motor model ("AK80-9", "AK10-9")
            motor_id: CAN ID
            gear_ratio: gear ratio
            frequency: control frequency (Hz)
            offline: offline mode
            current_mode: current convention ('driver', 'amplitude-invariant', 'power-invariant')
                - 'driver': use driver's native current (default)
                - 'amplitude-invariant': true phase current convention
                - 'power-invariant': power-invariant current convention
            thermal_current_scale: compensates for discrepancy between firmware k_t and effective k_t
        """
        # Validate motor type
        if motor_type not in TMOTOR_MODELS:
            raise ValueError(f"Unsupported motor type: {motor_type}")
        # Validate gear ratio matches tmotor model
        if TMOTOR_MODELS[motor_type]["GEAR_RATIO"] != gear_ratio:
            raise ValueError(f"Gear ratio {gear_ratio} does not match {motor_type}")
        # Validate current mode
        if current_mode not in ["driver", "amplitude-invariant", "power-invariant"]:
            raise ValueError(
                f"Invalid current_mode: {current_mode}. Must be 'driver', 'amplitude-invariant', or 'power-invariant'"
            )

        ActuatorBase.__init__(
            self,
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=TMOTOR_SERVO_CONSTANTS,
            frequency=frequency,
            offline=offline,
        )

        # Override motor constants to ensure setter is called
        self.MOTOR_CONSTANTS = TMOTOR_SERVO_CONSTANTS

        # Motor configuration
        self.motor_type = motor_type
        self.motor_id = motor_id
        self._motor_params = TMOTOR_MODELS[motor_type]
        self.current_mode = current_mode
        self._thermal_current_scale = thermal_current_scale

        # Current conversion factors
        # Physical: I_drv (line current) = K * I_user
        # Code: driver_current = user_current / self._current_scale
        # So: I_drv = I_user / scale, which means scale = I_user / I_drv = 1 / K
        # Kt relationship: τ = Kt_drv * I_drv = Kt_user * I_user
        # So: Kt_user = Kt_drv * (I_drv / I_user) = Kt_drv * K
        if self.current_mode == "amplitude-invariant":
            # I_drv (line) = √3 * I_phase, so scale = I_phase / I_drv = 1/√3
            # Kt_amp = Kt_drv * √3
            self._current_scale = 1.0 / np.sqrt(3.0)
            self._kt_scale = np.sqrt(3.0)
        elif self.current_mode == "power-invariant":
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

        # Error handling
        self._error_code: Optional[int] = None
        self._error_message: Optional[str] = None

        # Time management
        self._start_time = time.monotonic()
        self._last_update_time = self._start_time
        self._last_command_time: Optional[float] = None

        # Thermal management
        self._thermal_model = ThermalModel(
            motor_constants=self._MOTOR_CONSTANTS,
            actuator_tag=self.tag,
        )

        LOGGER.info(f"Initialized TMotor servo: {self.motor_type} ID:{self.motor_id}")

    def __str__(self) -> str:
        return f"{self.motor_type} ID:{self.motor_id}"

    @property
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        return TMOTOR_SERVO_CONTROL_MODE_CONFIGS

    @check_actuator_connection
    def start(self) -> None:
        """
        Starts the actuator by powering on the device via CAN, setting control mode
        to IDLE, and checking error status.

        Returns:
            None
        """
        LOGGER.info(f"Starting {self.__str__()}")

        if not self.is_offline and self._canman:
            self._canman.power_on(self.motor_id)

            # Poll for motor readiness (max 1 second timeout)
            start_time = time.monotonic()
            timeout = 1.0
            poll_interval = 0.01  # 10ms polling interval
            motor_ready = False

            while time.monotonic() - start_time < timeout:
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
                    LOGGER.debug(f"Motor ready after {time.monotonic() - start_time:.3f} seconds")
                    break
                time.sleep(poll_interval)

            if not motor_ready:
                # Fall back to minimum wait time if no status received
                LOGGER.warning("No status received from motor, using fallback delay")
                time.sleep(0.1)  # Minimum wait time

            # Set initial control mode to IDLE
            self.set_control_mode(CONTROL_MODES.IDLE)

            # Poll for mode switch confirmation (max 200ms timeout)
            mode_start_time = time.monotonic()
            mode_timeout = 0.2

            while time.monotonic() - mode_start_time < mode_timeout:
                self.update()  # Read status
                if self._motor_state.error == 0:
                    LOGGER.debug(f"Mode switch confirmed after {time.monotonic() - mode_start_time:.3f} seconds")
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
        """Stop motor and clean up CAN communication"""
        LOGGER.info(f"Stopping {self.__str__()}")

        if not self.is_offline and self._canman:
            # Set motor to idle mode first
            try:
                self._canman.set_current(self.motor_id, 0.0)
                time.sleep(0.01)
                self.set_control_mode(CONTROL_MODES.IDLE)
                time.sleep(0.05)  # Give motor time to process
            except Exception as e:
                LOGGER.warning(f"Error setting idle mode during stop: {e}")

            # Send power off command
            try:
                self._canman.power_off(self.motor_id)
                time.sleep(0.05)  # Give motor time to power off
            except Exception as e:
                LOGGER.warning(f"Error powering off motor: {e}")

            # Remove this motor's listener
            self._canman.remove_motor_listener(self)

            # Close CAN manager (will only actually close if this is the last motor)
            self._canman.close()

        self._is_open = False
        self._is_streaming = False

    def update(self) -> None:
        """
        Updates the actuator's data by reading new values and updating the thermal model.
        It raises exceptions if thermal limits are exceeded.

        Returns:
            None
        """
        # Update state
        self._motor_state.position = self._motor_state_async.position
        self._motor_state.velocity = self._motor_state_async.velocity
        self._motor_state.current = self._motor_state_async.current
        self._motor_state.temperature = self._motor_state_async.temperature
        self._motor_state.error = self._motor_state_async.error

        # Temperature check
        self._thermal_scale = self._thermal_model.update(
            dt=1 / self.frequency,
            motor_current=self.motor_current * self._thermal_current_scale,
            case_temperature=self.case_temperature,
        )

        # Communication timeout check
        now = time.monotonic()
        if (
            self._last_command_time is not None
            and (now - self._last_command_time) < 0.25
            and (now - self._last_update_time) > 0.1
        ):
            warnings.warn(f"No data from motor: {self.__str__()}", RuntimeWarning, stacklevel=2)

        self._last_update_time = now

    def _update_state_async(self, servo_state: ServoMotorState) -> None:
        """
        Asynchronously update state and interprets errors.

        Returns:
            None
        """
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

    def home(self) -> None:
        """
        This method homes the actuator and the corresponding joint by moving it to the zero position.
        The zero position is defined as the position where the joint is fully extended.
        Returns:
            None
        """
        raise NotImplementedError("Homing not implemented.")

    def set_origin(self) -> None:
        """
        Set encoder origin.
        This function automatically applies a "zero position" offset.
        """
        if not self.is_offline and self._canman:
            self._canman.set_origin(self.motor_id, 1)
            time.sleep(0.3)
        LOGGER.info(f"Set origin {self.__str__()}")

    @property
    def MOTOR_CONSTANTS(self) -> MOTOR_CONSTANTS:
        """
        Get the motor constants configuration.
        Redefines the property from the ABC so we can set a setter.

        Returns:
            MOTOR_CONSTANTS: The motor constants.

        Examples:
            >>> constants = actuator.MOTOR_CONSTANTS
            >>> constants.MAX_CASE_TEMPERATURE
            80.0
        """
        return self._MOTOR_CONSTANTS

    @MOTOR_CONSTANTS.setter
    def MOTOR_CONSTANTS(self, value: MOTOR_CONSTANTS) -> None:
        """
        Setter for MOTOR_CONSTANTS property.
        Updates the motor constants and recalculates derived conversion factors.

        Args:
            value (MOTOR_CONSTANTS): New motor constants to set.
        """

        if not isinstance(value, MOTOR_CONSTANTS):
            raise TypeError(f"Expected MOTOR_CONSTANTS, got {type(value)}")
        self._MOTOR_CONSTANTS = value

    # ============ Control Interface ============

    def set_motor_voltage(self, value: float) -> None:
        """Set motor voltage (not directly supported in servo mode)"""
        raise NotImplementedError("Voltage control not supported in servo mode")

    def set_motor_current(self, value: float) -> None:
        """
        Set motor current with clamping to motor limits
        Args:
            value (float): desired motor current in mA
        """
        if not self.is_offline and self._canman:
            driver_current = value

            # Get current limits from motor parameters
            max_current = self._motor_params["Curr_max"]
            min_current = self._motor_params["Curr_min"]

            # Clamp current to safe limits
            clamped_driver_current = np.clip(driver_current, min_current, max_current)

            # Log warning if clamping occurred
            if driver_current != clamped_driver_current:
                clamped_user_current = clamped_driver_current * self._current_scale
                LOGGER.warning(
                    f"Current command {value}mA clamped to {clamped_user_current}mA "
                    f"(limits: [{min_current * self._current_scale:.1f}, {max_current * self._current_scale:.1f}]mA)"
                )

            self._canman.set_current(self.motor_id, clamped_driver_current)
            self._last_command_time = time.monotonic()

    def set_motor_position(self, value: float) -> None:
        raise NotImplementedError(
            "Setting motor position not supported" "Recommended to use 'set_output_position' command instead."
        )

    def set_output_position(self, value: float) -> None:
        """
        Set motor position (radians) with clamping to motor limits
        Args:
            value (float): desired motor position
        Return:
            None
        """
        position_deg = radians_to_degrees(value)

        if not self.is_offline and self._canman:
            # Get position limits from motor parameters
            max_position = self._motor_params["P_max"]
            min_position = self._motor_params["P_min"]

            # Clamp position to safe limits
            clamped_position = np.clip(position_deg, min_position, max_position)

            # Log warning if clamping occurred
            if position_deg != clamped_position:
                LOGGER.warning(
                    f"Position command {position_deg:.1f}° clamped to {clamped_position:.1f}° "
                    f"(limits: [{min_position:.0f}, {max_position:.0f}]°)"
                )

            self._canman.set_position(self.motor_id, clamped_position)
            self._last_command_time = time.monotonic()

    def set_motor_torque(self, value: float) -> None:
        """
        Sets the motor torque in Nm.
        This is the torque that is applied to the motor rotor, not the joint or output.
        Args:
            value (float): The torque to set in Nm.
        Returns:
            None
        """
        # Torque to current: T = I * Kt
        # Kt_user = Kt_drv * kt_scale
        kt_user = self._motor_params["Kt_actual"] * self._kt_scale
        current = value / kt_user * 1000
        self.set_motor_current(current)  # Send current command mA

    def set_output_torque(self, value: float) -> None:
        """
        Set the output torque of the joint.
        This is the torque that is applied to the joint, not the motor.
        Args:
            value (float): torque in Nm
        Returns:
            None
        """
        # Output torque to motor torque: T_motor = T_output / gear_ratio
        motor_torque = value / self.gear_ratio
        self.set_motor_torque(motor_torque)

    def set_motor_velocity(self, value: float) -> None:
        """Set motor velocity (rad/s) with clamping to motor limits"""
        velocity_erpm = rad_per_sec_to_erpm(value, self.num_pole_pairs)

        if not self.is_offline and self._canman:
            # Get velocity limits from motor parameters
            max_velocity = self._motor_params["V_max"]  # Already in ERPM
            min_velocity = self._motor_params["V_min"]  # Already in ERPM

            # Clamp velocity to safe limits
            clamped_velocity = np.clip(velocity_erpm, min_velocity, max_velocity)

            # Log warning if clamping occurred
            if velocity_erpm != clamped_velocity:
                LOGGER.warning(
                    f"Velocity command {velocity_erpm:.0f} ERPM clamped to {clamped_velocity:.0f} ERPM "
                    f"(limits: [{min_velocity}, {max_velocity}] ERPM)"
                )

            self._canman.set_velocity(self.motor_id, clamped_velocity)
            self._last_command_time = time.monotonic()

    def set_output_velocity(self, value: float) -> None:
        """Set output velocity (rad/s)"""
        motor_velocity = value * self.gear_ratio
        self.set_motor_velocity(motor_velocity)

    # ============ Unsupported PID Functions - TMotor Servo Mode Handles All Control Loops Internally ============

    def set_current_gains(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0, ff: float = 0.0) -> None:
        """TMotor servo mode does not support external current PID gains - motor handles current control internally"""
        LOGGER.warning(
            "TMotor servo mode handles current control internally. " "External current PID gains are not used."
        )

    def set_position_gains(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0, ff: float = 0.0) -> None:
        """TMotor servo mode does not support external position PID gains - motor handles position control internally"""
        LOGGER.warning(
            "TMotor servo mode handles position control internally. " "External position PID gains are not used."
        )

    def _set_impedance_gains(self, k: float = 0.0, b: float = 0.0) -> None:
        """Internal method for impedance gains - not supported in TMotor servo mode"""
        LOGGER.warning("TMotor servo mode handles control internally. " "Impedance gains are not used.")

    # ============ State Properties ============

    @property
    def motor_position(self) -> float:
        """Motor position (radians) - not supported"""
        LOGGER.warning(
            "Motor position reading is not available. "
            "The position returned in the motor state corresponds to the output position."
        )
        return 0.0

    @property
    def output_position(self) -> float:
        """Output position (radians)"""
        return degrees_to_radians(self._motor_state.position)

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
        return self._motor_params["NUM_POLE_PAIRS"]

    @property
    def motor_voltage(self) -> float:
        """Motor voltage - not available in servo mode"""
        LOGGER.warning(
            "Motor voltage reading is not available in TMotor servo mode. "
            "The motor does not provide voltage feedback through the CAN protocol."
        )
        return 0.0

    @property
    def motor_current(self) -> float:
        """
        Motor current in mA

        Returns:
            float: Motor current in mA
        """
        return self._motor_state.current * self._current_scale

    @property
    def motor_torque(self) -> float:
        """
        Torque at the motor in Nm.
        This is calculated using motor current and k_t.
        """
        kt_user = cast(float, self._motor_params["Kt_actual"]) * self._kt_scale
        return self.motor_current * kt_user / 1000

    @property
    def output_torque(self) -> float:
        """
        Torque at the joint output in Nm.
        This is calculated using motor current, k_t, and the gear ratio.
        """
        return self.motor_torque * self.gear_ratio

    @property
    def case_temperature(self) -> float:
        """
        Case temperature in degrees Celsius. This is read during actuator update.

        Returns:
            float: Case temperature in degrees Celsius.
        """
        return self._motor_state.temperature

    @property
    def winding_temperature(self) -> float:
        """
        ESTIMATED temperature of the windings in Celsius.
        This is calculated based on the thermal model using motor current.

        Returns:
            float: Winding temperature in degrees Celsius.
        """
        return float(self._thermal_model.winding_temperature)

    @property
    def thermal_scaling_factor(self) -> float:
        """
        Scale factor to use in torque control, in [0,1].
        If you scale the torque command by this factor, the motor temperature will never
        exceed max allowable temperature. For a proof, see paper referenced in thermal model.

        Returns:
            float: Thermal scaling factor.
        """
        return self._thermal_scale


if __name__ == "__main__":
    pass
