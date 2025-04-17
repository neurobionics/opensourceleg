import os
import time
from ctypes import c_int
from typing import Optional

import numpy as np
from flexsea.device import Device

from opensourceleg.actuators.base import (
    CONTROL_MODE_CONFIGS,
    CONTROL_MODES,
    MOTOR_CONSTANTS,
    ActuatorBase,
    ControlGains,
    ControlModeConfig,
)
from opensourceleg.actuators.decorators import (
    check_actuator_connection,
    check_actuator_open,
    check_actuator_stream,
)
from opensourceleg.extras.safety import I2tLimitException, ThermalLimitException
from opensourceleg.logging import LOGGER
from opensourceleg.logging.decorators import (
    deprecated_with_routing,
)
from opensourceleg.logging.exceptions import ControlModeException
from opensourceleg.math import ThermalModel

DEFAULT_POSITION_GAINS = ControlGains(kp=30, ki=0, kd=0, k=0, b=0, ff=0)

DEFAULT_CURRENT_GAINS = ControlGains(kp=40, ki=400, kd=0, k=0, b=0, ff=128)

DEFAULT_IMPEDANCE_GAINS = ControlGains(kp=40, ki=400, kd=0, k=200, b=400, ff=128)

DEPHY_SLEEP_DURATION = 0.1

RAD_PER_DEG = np.pi / 180
RAD_PER_SEC_GYROLSB = np.pi / 180 / 32.8
M_PER_SEC_SQUARED_ACCLSB = 9.80665 / 8192
IMPEDANCE_A = 0.00028444
IMPEDANCE_C = 0.0007812


DEPHY_ACTUATOR_CONSTANTS = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=16384,
    NM_PER_AMP=0.1133,
    NM_PER_RAD_TO_K=((2 * np.pi / 16384) / IMPEDANCE_C * 1e3 / 0.1133),
    NM_S_PER_RAD_TO_B=((np.pi / 180) / IMPEDANCE_A * 1e3 / 0.1133),
    MAX_CASE_TEMPERATURE=80,
    MAX_WINDING_TEMPERATURE=110,
)


def _dephy_voltage_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}] Entering Voltage control mode.")


def _dephy_voltage_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}]  Exiting Voltage control mode.")
    dephy_actuator.stop_motor()
    time.sleep(DEPHY_SLEEP_DURATION)


def _dephy_current_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}]  Entering Current control mode.")


def _dephy_current_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}]  Exiting Current control mode.")
    dephy_actuator.stop_motor()
    time.sleep(DEPHY_SLEEP_DURATION)


def _dephy_position_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}]  Entering Position control mode.")


def _dephy_position_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}]  Exiting Position control mode.")
    dephy_actuator.stop_motor()
    time.sleep(DEPHY_SLEEP_DURATION)


def _dephy_impedance_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}]  Entering Impedance control mode.")


def _dephy_impedance_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}]  Exiting Impedance control mode.")
    dephy_actuator.stop_motor()
    time.sleep(DEPHY_SLEEP_DURATION)


DEPHY_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    POSITION=ControlModeConfig(
        entry_callback=_dephy_position_mode_entry,
        exit_callback=_dephy_position_mode_exit,
        has_gains=False,
        max_gains=ControlGains(kp=1000, ki=1000, kd=1000, k=0, b=0, ff=0),
    ),
    CURRENT=ControlModeConfig(
        entry_callback=_dephy_current_mode_entry,
        exit_callback=_dephy_current_mode_exit,
        has_gains=False,
        max_gains=ControlGains(kp=80, ki=800, kd=0, k=0, b=0, ff=128),
    ),
    VOLTAGE=ControlModeConfig(
        entry_callback=_dephy_voltage_mode_entry,
        exit_callback=_dephy_voltage_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
    IMPEDANCE=ControlModeConfig(
        entry_callback=_dephy_impedance_mode_entry,
        exit_callback=_dephy_impedance_mode_exit,
        has_gains=False,
        max_gains=ControlGains(kp=80, ki=800, kd=0, k=1000, b=1000, ff=128),
    ),
)


class DephyActuator(Device, ActuatorBase):  # type: ignore[no-any-unimported]
    """
    Interface to a Dephy actuator device.

    Examples:
        >>> actuator = DephyActuator(port='/dev/ttyACM0', gear_ratio=2.0)
        >>> actuator.start()
        >>> actuator.set_motor_voltage(1500)
        >>> print(f"Output position: {actuator.output_position:.2f} rad")
    """

    def __init__(
        self,
        tag: str = "DephyActuator",
        firmware_version: str = "7.2.0",
        port: str = "/dev/ttyACM0",
        gear_ratio: float = 1.0,
        baud_rate: int = 230400,
        frequency: int = 500,
        debug_level: int = 4,
        dephy_log: bool = False,
        offline: bool = False,
        stop_motor_on_disconnect: bool = True,
    ) -> None:
        ActuatorBase.__init__(
            self,
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=DEPHY_ACTUATOR_CONSTANTS,
            frequency=frequency,
            offline=offline,
        )

        self._debug_level: int = debug_level if dephy_log else 6
        self._dephy_log: bool = dephy_log

        if self.is_offline:
            self.port = port
            self._is_streaming: bool = False
            self._is_open: bool = False
        else:
            Device.__init__(
                self,
                firmwareVersion=firmware_version,
                port=port,
                baudRate=baud_rate,
                stopMotorOnDisconnect=stop_motor_on_disconnect,
            )

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=self.max_winding_temperature,
            soft_border_C_windings=10,
            temp_limit_case=self.max_case_temperature,
            soft_border_C_case=10,
        )
        self._thermal_scale: float = 1.0

        self._mode = CONTROL_MODES.IDLE

    def __repr__(self) -> str:
        return f"{self.tag}[DephyActuator]"

    @property
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        return DEPHY_CONTROL_MODE_CONFIGS

    @check_actuator_connection
    def start(self) -> None:
        """
        Starts the actuator by opening the port, starting data streaming,
        reading initial data, and setting the control mode to VOLTAGE.

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
        """
        try:
            self.open()
            self._is_open = True
        except OSError:
            print("\n")
            LOGGER.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port '{self.port}'. \n\n \
                    Please run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        self.start_streaming(self._frequency)
        time.sleep(0.2)
        self._is_streaming = True

        self._data = self.read()
        self.set_control_mode(CONTROL_MODES.VOLTAGE)

    @check_actuator_stream
    @check_actuator_open
    def stop(self) -> None:
        """
        Stops the actuator by stopping the motor, switching to IDLE mode,
        stopping data streaming, and closing the connection.

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> # ... perform control tasks ...
            >>> actuator.stop()
        """
        self.stop_motor()
        self.set_control_mode(mode=CONTROL_MODES.IDLE)
        self._is_streaming = False
        self._is_open = False
        self.stop_streaming()
        self.close()

    def update(self) -> None:
        """
        Updates the actuator's data by reading new values and updating the thermal model.
        It raises exceptions if thermal limits are exceeded.

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator()
            >>> actuator.start()
            >>> actuator.update()
            >>> print(f"Motor current: {actuator.motor_current} mA")
        """
        self._data = self.read()

        self._thermal_model.T_c = self.case_temperature
        self._thermal_scale = self._thermal_model.update_and_get_scale(
            dt=1 / self.frequency,
            motor_current=self.motor_current,
        )
        if self.case_temperature >= self.max_case_temperature:
            LOGGER.error(
                msg=f"[{str.upper(self.tag)}] Case thermal limit {self.max_case_temperature} reached. "
                f"Current Case Temperature: {self.case_temperature} C. Exiting."
            )
            raise ThermalLimitException()

        if self.winding_temperature >= self.max_winding_temperature:
            LOGGER.error(
                msg=f"[{str.upper(self.tag)}] Winding thermal limit {self.max_winding_temperature} reached."
                f"Current Winding Temperature: {self.winding_temperature} C. Exiting."
            )
            raise ThermalLimitException()
        # Check for thermal fault, bit 2 of the execute status byte
        if self._data["status_ex"] & 0b00000010 == 0b00000010:
            # "Maximum Average Current" limit exceeded for "time at current limit,
            # review physical setup to ensure excessive torque is not normally applied
            # If issue persists, review "Maximum Average Current", "Current Limit", and
            # "Time at current limit" settings for the Dephy ActPack Firmware using the Plan GUI software
            LOGGER.error(msg=f"[{str.upper(self.tag)}] I2t limit exceeded. " f"Current: {self.motor_current} mA. ")
            raise I2tLimitException()

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

        This method homes the actuator and the corresponding joint by moving it to the zero position.
        The zero position is defined as the position where the joint is fully extended. This method will
        also load the encoder map if it exists. The encoder map is a polynomial that maps the encoder counts
        to joint position in radians. This is useful for more accurate joint position estimation.

        Args:
            homing_voltage (int): Voltage in mV to use for homing. Default is 2000 mV.
            homing_frequency (int): Frequency in Hz to use for homing. Default is the actuator's frequency.
            homing_direction (int): Direction to move the actuator during homing. Default is -1.
            output_position_offset (float): Offset in radians to add to the output position. Default is 0.0.
            current_threshold (int): Current threshold in mA to stop homing the joint or actuator.
                This is used to detect if the actuator or joint has hit a hard stop. Default is 5000 mA.
            velocity_threshold (float): Velocity threshold in rad/s to stop homing the joint or actuator.
                This is also used to detect if the actuator or joint has hit a hard stop. Default is 0.001 rad/s.
        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.home(homing_voltage=2000, homing_direction=-1)

        """
        is_homing = True
        homing_frequency = homing_frequency if homing_frequency is not None else self.frequency

        LOGGER.info(
            f"[{str.upper(self.tag)}] Homing {self.tag} joint. "
            "Please make sure the joint is free to move and press Enter to continue."
        )
        input()

        self.set_control_mode(mode=CONTROL_MODES.VOLTAGE)

        self.set_motor_voltage(value=homing_direction * homing_voltage)  # mV, negative for counterclockwise

        time.sleep(0.1)

        try:
            while is_homing:
                self.update()
                time.sleep(1 / homing_frequency)

                if abs(self.output_velocity) <= velocity_threshold or abs(self.motor_current) >= current_threshold:
                    self.set_motor_voltage(value=0)
                    is_homing = False

        except KeyboardInterrupt:
            self.set_motor_voltage(value=0)
            LOGGER.info(msg=f"[{str.upper(self.tag)}] Homing interrupted.")
            return
        except Exception as e:
            self.set_motor_voltage(value=0)
            LOGGER.error(msg=f"[{str.upper(self.tag)}] Homing failed: {e}")
            return

        self.set_motor_zero_position(value=self.motor_position + output_position_offset * self.gear_ratio)

        time.sleep(0.1)

        self._is_homed = True
        LOGGER.info(f"[{str.upper(self.tag)}] Homing complete.")

    def set_motor_torque(self, value: float) -> None:
        """
        Sets the motor torque in Nm. This is the torque that is applied to the motor rotor, not the joint or output.
        Args:
            value (float): The torque to set in Nm.
        Returns:
            None
        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.set_motor_torque(0.1)
        """
        self.set_motor_current(
            value / self.MOTOR_CONSTANTS.NM_PER_MILLIAMP,
        )

    def set_output_torque(self, value: float) -> None:
        """
        Set the output torque of the joint.
        This is the torque that is applied to the joint, not the motor.

        Args:
            value (float): torque in N_m

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.set_output_torque(0.1)
        """
        self.set_motor_torque(value=value / self.gear_ratio)

    def set_motor_current(
        self,
        value: float,
    ) -> None:
        """
        Sets the motor current in mA.

        Args:
            value (float): The current to set in mA.
        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.set_motor_current(1000)
        """
        self.command_motor_current(value=int(value))

    @deprecated_with_routing(alternative_func=set_motor_current)
    def set_current(self, value: float) -> None:
        self.command_motor_current(value=int(value))

    def set_motor_voltage(self, value: float) -> None:
        """
        Sets the motor voltage in mV.

        Args:
            value (float): The voltage to set in mV.

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.set_motor_voltage(100) TODO: Validate number
        """
        self.command_motor_voltage(value=int(value))

    @deprecated_with_routing(alternative_func=set_motor_voltage)
    def set_voltage(self, value: float) -> None:
        self.command_motor_voltage(value=int(value))

    def set_motor_position(self, value: float) -> None:
        """
        Sets the motor position in radians.
        If in impedance mode, this sets the equilibrium angle in radians.

        Args:
            value (float): The position to set

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.set_motor_position(0.1)
        """
        # TODO: New Dephy API splits impedance equilibrium position and position control into separate methods
        if self.mode == CONTROL_MODES.POSITION:
            self.command_motor_position(
                value=int((value + self.motor_zero_position) / self.MOTOR_CONSTANTS.RAD_PER_COUNT),
            )
        elif self.mode == CONTROL_MODES.IMPEDANCE:
            self.command_motor_impedance(
                value=int((value + self.motor_zero_position) / self.MOTOR_CONSTANTS.RAD_PER_COUNT),
            )
        else:
            raise ControlModeException(tag=self._tag, attribute="set_motor_position", mode=self._mode.name)

    def set_position_gains(
        self,
        kp: float = DEFAULT_POSITION_GAINS.kp,
        ki: float = DEFAULT_POSITION_GAINS.ki,
        kd: float = DEFAULT_POSITION_GAINS.kd,
        ff: float = DEFAULT_POSITION_GAINS.ff,
    ) -> None:
        """
        Sets the position gains in arbitrary Dephy units.

        Args:
            kp (float): The proportional gain
            ki (float): The integral gain
            kd (float): The derivative gain
            ff (float): The feedforward gain

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.set_position_gains(kp=30, ki=0, kd=0, ff=0)
        """
        self.set_gains(
            kp=int(kp),
            ki=int(ki),
            kd=int(kd),
            k=0,
            b=0,
            ff=int(ff),
        )

    def set_current_gains(
        self,
        kp: float = DEFAULT_CURRENT_GAINS.kp,
        ki: float = DEFAULT_CURRENT_GAINS.ki,
        kd: float = DEFAULT_CURRENT_GAINS.kd,
        ff: float = DEFAULT_CURRENT_GAINS.ff,
    ) -> None:
        """
        Sets the current gains in arbitrary Dephy units.

        Args:
            kp (float): The proportional gain
            ki (float): The integral gain
            kd (float): The derivative gain
            ff (float): The feedforward gain

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.set_current_gains(kp=40, ki=400, kd=0, ff=128)
        """
        self.set_gains(
            kp=int(kp),
            ki=int(ki),
            kd=int(kd),
            k=0,
            b=0,
            ff=int(ff),
        )

    def set_output_impedance(
        self,
        kp: float = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: float = DEFAULT_IMPEDANCE_GAINS.ki,
        kd: float = DEFAULT_IMPEDANCE_GAINS.kd,
        k: float = 100.0,
        b: float = 3.0,
        ff: float = 128,
    ) -> None:
        """
        Set the impedance gains of the joint in real units: Nm/rad and Nm/rad/s.
        This sets the impedance at the output and automatically scales based on gear raitos.

        Conversion:
            K_motor = K_joint / (gear_ratio ** 2)
            B_motor = B_joint / (gear_ratio ** 2)

        Args:
            kp (float): Proportional gain. Defaults to 40.
            ki (float): Integral gain. Defaults to 400.
            kd (float): Derivative gain. Defaults to 0.
            k (float): Spring constant. Defaults to 100 Nm/rad.
            b (float): Damping constant. Defaults to 3.0 Nm/rad/s.
            ff (float): Feedforward gain. Defaults to 128.

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.set_output_impedance(kp=40, ki=400, kd=0, k=100, b=3, ff=128)
        """
        self.set_motor_impedance(
            kp=kp,
            ki=ki,
            kd=kd,
            k=k / (self.gear_ratio**2),
            b=b / (self.gear_ratio**2),
            ff=ff,
        )

    def set_impedance_gains(
        self,
        kp: float = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: float = DEFAULT_IMPEDANCE_GAINS.ki,
        kd: float = DEFAULT_IMPEDANCE_GAINS.kd,
        k: float = DEFAULT_IMPEDANCE_GAINS.k,
        b: float = DEFAULT_IMPEDANCE_GAINS.b,
        ff: float = DEFAULT_IMPEDANCE_GAINS.ff,
    ) -> None:
        """
        Sets the impedance gains in arbitrary actpack units.
        See Dephy's webpage for conversions or use other library methods that handle conversion for you.

        Args:
            kp (float): The proportional gain
            ki (float): The integral gain
            kd (float): The derivative gain
            k (float): The spring constant
            b (float): The damping constant
            ff (float): The feedforward gain

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.set_impedance_gains(kp=40, ki=400, kd=0, k=200, b=400, ff=128)
        """
        self.set_gains(
            kp=int(kp),
            ki=int(ki),
            kd=int(kd),
            k=int(k),
            b=int(b),
            ff=int(ff),
        )

    def set_motor_impedance(
        self,
        kp: float = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: float = DEFAULT_IMPEDANCE_GAINS.ki,
        kd: float = DEFAULT_IMPEDANCE_GAINS.kd,
        k: float = 0.08922,
        b: float = 0.0038070,
        ff: float = DEFAULT_IMPEDANCE_GAINS.ff,
    ) -> None:
        """
        Set the impedance gains of the motor in real units: Nm/rad and Nm/rad/s.

        Args:
            kp (float): Proportional gain. Defaults to 40.
            ki (float): Integral gain. Defaults to 400.
            kd (float): Derivative gain. Defaults to 0.
            k (float): Spring constant. Defaults to 0.08922 Nm/rad.
            b (float): Damping constant. Defaults to 0.0038070 Nm/rad/s.
            ff (float): Feedforward gain. Defaults to 128.

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.set_motor_impedance(kp=40, ki=400, kd=0, k=0.08922, b=0.0038070, ff=128) TODO: Validate numbers
        """
        self.set_impedance_gains(
            kp=kp,
            ki=ki,
            kd=kd,
            k=int(k * self.MOTOR_CONSTANTS.NM_PER_RAD_TO_K),
            b=int(b * self.MOTOR_CONSTANTS.NM_S_PER_RAD_TO_B),
            ff=ff,
        )

    @property
    def motor_voltage(self) -> float:
        """
        Q-axis motor voltage in mV.

        Returns:
            float: Motor voltage in mV.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Motor voltage: {actuator.motor_voltage} mV")
        """
        if self._data is not None:
            return float(self._data["mot_volt"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_current(self) -> float:
        """
        Motor current in mA.

        Returns:
            float: Motor current in mA.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Motor current: {actuator.motor_current} mA")
        """
        if self._data is not None:
            return float(self._data["mot_cur"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_torque(self) -> float:
        """
        Torque at the motor output in Nm.

        Returns:
            float: Motor torque in Nm.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Motor torque: {actuator.motor_torque} Nm")
        """
        if self._data is not None:
            return float(self._data["mot_cur"] * self.MOTOR_CONSTANTS.NM_PER_MILLIAMP)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_position(self) -> float:
        """
        Motor position in radians.

        Returns:
            float: Motor position in radians.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Motor position: {actuator.motor_position} rad")
        """
        if self._data is not None:
            return float(self._data["mot_ang"] * self.MOTOR_CONSTANTS.RAD_PER_COUNT) - self.motor_zero_position
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_encoder_counts(self) -> int:
        """
        Raw reading from motor encoder in counts.

        Returns:
            int: Motor encoder counts.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Motor encoder counts: {actuator.motor_encoder_counts}")
        """
        if self._data is not None:
            return int(self._data["mot_ang"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0."
            )
            return 0

    @property
    def motor_velocity(self) -> float:
        """
        Motor velocity in rad/s.

        Returns:
            float: Motor velocity in rad/s.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Motor velocity: {actuator.motor_velocity} rad/s")
        """

        if self._data is not None:
            return int(self._data["mot_vel"]) * RAD_PER_DEG
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_acceleration(self) -> float:
        """
        Motor acceleration in rad/s^2.

        Returns:
            float: Motor acceleration in rad/s^2.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Motor acceleration: {actuator.motor_acceleration} rad/s^2")
        """

        if self._data is not None:
            return float(self._data["mot_acc"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def battery_voltage(self) -> float:
        """
        Battery voltage in mV.

        Returns:
            float: Battery voltage in mV.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Battery voltage: {actuator.battery_voltage} mV")
        """
        if self._data is not None:
            return float(self._data["batt_volt"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def battery_current(self) -> float:
        """
        Battery current in mA.

        Returns:
            float: Battery current in mA.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Battery current: {actuator.battery_current} mA")
        """
        if self._data is not None:
            return float(self._data["batt_curr"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def output_torque(self) -> float:
        """
        Torque at the joint output in Nm. This is calculated using motor current, k_t, and the gear ratio.

        Returns:
            float: Output torque in Nm.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Output torque: {actuator.output_torque} Nm")
        """
        return self.motor_torque * self.gear_ratio

    @property
    def case_temperature(self) -> float:
        """
        Case temperature in degrees celsius. This is read during actuator update.

        Returns:
            float: Case temperature in degrees celsius.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Case temperature: {actuator.case_temperature} C")
        """
        if self._data is not None:
            return float(self._data["temperature"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def winding_temperature(self) -> float:
        """
        ESTIMATED temperature of the windings in celsius.
        This is calculated based on the thermal model using motor current.

        Returns:
            float: Winding temperature in degrees celsius.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Winding temperature: {actuator.winding_temperature} C")
        """
        if self._data is not None:
            return float(self._thermal_model.T_w)
        else:
            return 0.0

    @property
    def genvars(self) -> np.ndarray:
        """Dephy's 'genvars' object."""
        if self._data is not None:
            return np.array(
                object=[
                    self._data["genvar_0"],
                    self._data["genvar_1"],
                    self._data["genvar_2"],
                    self._data["genvar_3"],
                    self._data["genvar_4"],
                    self._data["genvar_5"],
                ]
            )
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning zeros"
            )
            return np.zeros(shape=6)

    @property
    def accelx(self) -> float:
        """
        Acceleration in x direction in m/s^2.
        Measured using actpack's onboard IMU.

        Returns:
            float: Acceleration in x direction in m/s^2.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Acceleration in x direction: {actuator.accelx} m/s^2")
        """
        if self._data is not None:
            return float(self._data["accelx"] * M_PER_SEC_SQUARED_ACCLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def accely(self) -> float:
        """
        Acceleration in y direction in m/s^2.
        Measured using actpack's onboard IMU.

        Returns:
            float: Acceleration in y direction in m/s^2.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Acceleration in y direction: {actuator.accely} m/s^2")
        """
        if self._data is not None:
            return float(self._data["accely"] * M_PER_SEC_SQUARED_ACCLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def accelz(self) -> float:
        """
        Acceleration in z direction in m/s^2.
        Measured using actpack's onboard IMU.

        Returns:
            float: Acceleration in z direction in m/s^2.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Acceleration in z direction: {actuator.accelz} m/s^2")
        """
        if self._data is not None:
            return float(self._data["accelz"] * M_PER_SEC_SQUARED_ACCLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def gyrox(self) -> float:
        """
        Angular velocity in x direction in rad/s.
        Measured using actpack's onboard IMU.

        Returns:
            float: Angular velocity in x direction in rad/s.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Angular velocity in x direction: {actuator.gyrox} rad/s")
        """
        if self._data is not None:
            return float(self._data["gyrox"] * RAD_PER_SEC_GYROLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def gyroy(self) -> float:
        """
        Angular velocity in y direction in rad/s.
        Measured using actpack's onboard IMU.

        Returns:
            float: Angular velocity in y direction in rad/s.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Angular velocity in y direction: {actuator.gyroy} rad/s")
        """
        if self._data is not None:
            return float(self._data["gyroy"] * RAD_PER_SEC_GYROLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def gyroz(self) -> float:
        """
        Angular velocity in z direction in rad/s.
        Measured using actpack's onboard IMU.

        Returns:
            float: Angular velocity in z direction in rad/s.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Angular velocity in z direction: {actuator.gyroz} rad/s")
        """
        if self._data is not None:
            return float(self._data["gyroz"] * RAD_PER_SEC_GYROLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def thermal_scaling_factor(self) -> float:
        """
        Scale factor to use in torque control, in [0,1].
        If you scale the torque command by this factor, the motor temperature will never
        exceed max allowable temperature. For a proof, see paper referenced in thermal model.

        Returns:
            float: Thermal scaling factor.

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> print(f"Thermal scaling factor: {actuator.thermal_scaling_factor}")
            >>> actuator.update()
            >>> # This will update the thermal model and return the new scaling factor.
            >>> print(f"Thermal scaling factor: {actuator.thermal_scaling_factor}")
        """
        return self._thermal_scale


def _dephy_legacy_voltage_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}] Entering {CONTROL_MODES.VOLTAGE.name} control mode.")


def _dephy_legacy_current_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}] Entering {CONTROL_MODES.CURRENT.name} control mode.")


def _dephy_legacy_position_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}] Entering {CONTROL_MODES.POSITION.name} control mode.")


def _dephy_legacy_impedance_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}] Entering {CONTROL_MODES.IMPEDANCE.name} control mode.")


def _dephy_legacy_voltage_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}] Exiting {CONTROL_MODES.VOLTAGE.name} control mode.")


def _dephy_legacy_current_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}] Exiting {CONTROL_MODES.CURRENT.name} control mode.")


def _dephy_legacy_position_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}] Exiting {CONTROL_MODES.POSITION.name} control mode.")


def _dephy_legacy_impedance_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[{dephy_actuator.tag}] Exiting {CONTROL_MODES.IMPEDANCE.name} control mode.")


DEPHY_LEGACY_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    POSITION=ControlModeConfig(
        entry_callback=_dephy_legacy_position_mode_entry,
        exit_callback=_dephy_legacy_position_mode_exit,
        has_gains=False,
        max_gains=ControlGains(kp=1000, ki=1000, kd=1000, k=0, b=0, ff=0),
    ),
    CURRENT=ControlModeConfig(
        entry_callback=_dephy_legacy_current_mode_entry,
        exit_callback=_dephy_legacy_current_mode_exit,
        has_gains=False,
        max_gains=ControlGains(kp=80, ki=800, kd=0, k=0, b=0, ff=128),
    ),
    VOLTAGE=ControlModeConfig(
        entry_callback=_dephy_legacy_voltage_mode_entry,
        exit_callback=_dephy_legacy_voltage_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
    IMPEDANCE=ControlModeConfig(
        entry_callback=_dephy_legacy_impedance_mode_entry,
        exit_callback=_dephy_legacy_impedance_mode_exit,
        has_gains=False,
        max_gains=ControlGains(kp=80, ki=800, kd=0, k=1000, b=1000, ff=128),
    ),
)


class DephyLegacyActuator(DephyActuator):
    def __init__(
        self,
        tag: str = "DephyActuator",
        port: str = "/dev/ttyACM0",
        gear_ratio: float = 1.0,
        baud_rate: int = 230400,
        frequency: int = 500,
        debug_level: int = 4,
        dephy_log: bool = False,
        offline: bool = False,
    ) -> None:
        ActuatorBase.__init__(
            self,
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=DEPHY_ACTUATOR_CONSTANTS,
            frequency=frequency,
            offline=offline,
        )

        self._debug_level: int = debug_level if dephy_log else 6
        self._dephy_log: bool = dephy_log

        if self.is_offline:
            self.port = port
            self._is_streaming: bool = False
            self._is_open: bool = False
        else:
            # def set_is_streaming(self, value):
            #     self._is_streaming = value

            # def set_is_open(self, value):
            #     self._is_open = value

            # type(self).is_streaming = property(fset=set_is_streaming)
            # type(self).is_open = property(fset=set_is_open)

            Device.__init__(self, port=port, baud_rate=baud_rate)

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=self.max_winding_temperature,
            soft_border_C_windings=10,
            temp_limit_case=self.max_case_temperature,
            soft_border_C_case=10,
        )
        self._thermal_scale: float = 1.0

        self._mode = CONTROL_MODES.IDLE

    def __repr__(self) -> str:
        return f"{self.tag}[DephyLegacyActuator]"

    @property
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        return DEPHY_LEGACY_CONTROL_MODE_CONFIGS

    @check_actuator_connection
    def start(self) -> None:
        try:
            self.open(
                freq=self._frequency,
                log_level=self._debug_level,
                log_enabled=self._dephy_log,
            )
        except OSError:
            print("\n")
            LOGGER.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port '{self.port}'. \n\n \
                    Please run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        self._data = self.read()

        # TODO: Verify if we need this sleep here
        time.sleep(0.1)
        self.set_control_mode(CONTROL_MODES.VOLTAGE)

    @check_actuator_stream
    @check_actuator_open
    def stop(self) -> None:
        self.set_control_mode(mode=CONTROL_MODES.VOLTAGE)
        self.set_motor_voltage(value=0)

        self.set_control_mode(mode=CONTROL_MODES.IDLE)
        time.sleep(0.1)
        self._is_streaming = False
        self._is_open = False
        self.stop_streaming()
        self.close()

    def update(self) -> None:
        self._data = self.read()

        self._thermal_model.T_c = self.case_temperature
        self._thermal_scale = self._thermal_model.update_and_get_scale(
            dt=1 / self.frequency,
            motor_current=self.motor_current,
        )
        if self.case_temperature >= self.max_case_temperature:
            LOGGER.error(
                f"[{str.upper(self.tag)}] Case thermal limit {self.max_case_temperature} reached. "
                f"Current case temperature: {self.case_temperature}. Stopping motor."
            )
            raise ThermalLimitException()

        if self.winding_temperature >= self.max_winding_temperature:
            LOGGER.error(
                f"[{str.upper(self.tag)}] Winding thermal limit {self.max_winding_temperature} reached. "
                f"Current winding temperature: {self.winding_temperature}. Stopping motor."
            )
            raise ThermalLimitException()
        # Check for thermal fault, bit 2 of the execute status byte

        if self._data.status_ex & 0b00000010 == 0b00000010:
            LOGGER.error(
                f"[{str.upper(self.tag)}] Thermal Fault: Winding temperature: {self.winding_temperature}; "
                f"Case temperature: {self.case_temperature}."
            )
            raise ThermalLimitException("Internal thermal limit tripped.")

    def set_motor_current(
        self,
        value: float,
    ) -> None:
        """
        Sets the motor current in mA.

        Args:
            value (float): The current to set in mA.
        """
        self.send_motor_command(ctrl_mode=c_int(self.mode.value), value=int(value))

    @deprecated_with_routing(alternative_func=set_motor_current)
    def set_current(self, value: float) -> None:
        self.send_motor_command(ctrl_mode=c_int(self.mode.value), value=int(value))

    def set_motor_voltage(self, value: float) -> None:
        """
        Sets the motor voltage in mV.

        Args:
            value (float): The voltage to set in mV.
        """
        self.send_motor_command(ctrl_mode=c_int(self.mode.value), value=int(value))

    @deprecated_with_routing(alternative_func=set_motor_voltage)
    def set_voltage(self, value: float) -> None:
        self.send_motor_command(ctrl_mode=c_int(self.mode.value), value=int(value))

    def set_motor_position(self, value: float) -> None:
        """
        Sets the motor position in radians.
        If in impedance mode, this sets the equilibrium angle in radians.

        Args:
            value (float): The position to set
        """
        self.send_motor_command(
            ctrl_mode=c_int(self.mode.value),
            value=int((value + self.motor_zero_position) / self.MOTOR_CONSTANTS.RAD_PER_COUNT),
        )

    @property
    def motor_voltage(self) -> float:
        """Q-axis motor voltage in mV."""
        if self._data is not None:
            return float(self._data.mot_volt)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_current(self) -> float:
        if self._data is not None:
            return float(self._data.mot_cur)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_torque(self) -> float:
        """
        Torque at the motor output in Nm.
        """
        if self._data is not None:
            return float(self._data.mot_cur * self.MOTOR_CONSTANTS.NM_PER_MILLIAMP)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_position(self) -> float:
        """
        Motor position in radians.
        """
        if self._data is not None:
            return float(self._data.mot_ang * self.MOTOR_CONSTANTS.RAD_PER_COUNT) - self.motor_zero_position
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_encoder_counts(self) -> int:
        """Raw reading from motor encoder in counts."""
        if self._data is not None:
            return int(self._data.mot_ang)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0."
            )
            return 0

    @property
    def motor_velocity(self) -> float:
        """
        Motor velocity in rad/s.
        """
        if self._data is not None:
            return int(self._data.mot_vel) * RAD_PER_DEG
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_acceleration(self) -> float:
        """
        Motor acceleration in rad/s^2.
        """
        if self._data is not None:
            return float(self._data.mot_acc)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def battery_voltage(self) -> float:
        """Battery voltage in mV."""
        if self._data is not None:
            return float(self._data.batt_volt)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def battery_current(self) -> float:
        """Battery current in mA."""
        if self._data is not None:
            return float(self._data.batt_curr)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

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
        Case temperature of the actuator in celsius.
        """
        if self._data is not None:
            return float(self._data.temperature)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def winding_temperature(self) -> float:
        """
        ESTIMATED temperature of the windings in celsius.
        This is calculated based on the thermal model using motor current.
        """
        if self._data is not None:
            return float(self._thermal_model.T_w)
        else:
            return 0.0

    @property
    def genvars(self) -> np.ndarray:
        """Dephy's 'genvars' object."""
        if self._data is not None:
            return np.array(
                object=[
                    self._data.genvar_0,
                    self._data.genvar_1,
                    self._data.genvar_2,
                    self._data.genvar_3,
                    self._data.genvar_4,
                    self._data.genvar_5,
                ]
            )
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning zeros"
            )
            return np.zeros(shape=6)

    @property
    def accelx(self) -> float:
        """
        Acceleration in x direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.accelx * M_PER_SEC_SQUARED_ACCLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def accely(self) -> float:
        """
        Acceleration in y direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.accely * M_PER_SEC_SQUARED_ACCLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def accelz(self) -> float:
        """
        Acceleration in z direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.accelz * M_PER_SEC_SQUARED_ACCLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def gyrox(self) -> float:
        """
        Angular velocity in x direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.gyrox * RAD_PER_SEC_GYROLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def gyroy(self) -> float:
        """
        Angular velocity in y direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.gyroy * RAD_PER_SEC_GYROLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def is_streaming(self) -> bool:
        return self._is_streaming

    @is_streaming.setter
    def is_streaming(self, value: bool) -> None:
        self._is_streaming = value

    @property
    def is_open(self) -> bool:
        return self._is_open

    @is_open.setter
    def is_open(self, value: bool) -> None:
        self._is_open = value

    @property
    def gyroz(self) -> float:
        """
        Angular velocity in z direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.gyroz * RAD_PER_SEC_GYROLSB)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0


if __name__ == "__main__":
    pass
