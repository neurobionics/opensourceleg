from typing import Any, Callable, Union, overload

import ctypes
import os
import time
from ctypes import c_int
from dataclasses import dataclass
from unittest.mock import Mock

# import flexsea.fx_enums as fxe
import numpy as np
from flexsea.device import Device

from opensourceleg.actuators.base import (
    ActuatorBase,
    ControlGains,
    ControlModeBase,
    ControlModesBase,
    ControlModesMapping,
    MotorConstants,
)
from opensourceleg.actuators.decorators import (
    check_actuator_connection,
    check_actuator_open,
    check_actuator_stream,
)
from opensourceleg.actuators.exceptions import (
    ActuatorIsNoneException,
    ControlModeException,
)
from opensourceleg.logging import LOGGER
from opensourceleg.logging.decorators import (
    deprecated,
    deprecated_with_routing,
    deprecated_with_suggestion,
)
from opensourceleg.math import ThermalModel
from opensourceleg.safety import ThermalLimitException

DEFAULT_POSITION_GAINS = ControlGains(kp=50, ki=0, kd=0, k=0, b=0, ff=0)

DEFAULT_CURRENT_GAINS = ControlGains(kp=40, ki=400, kd=0, k=0, b=0, ff=128)

DEFAULT_IMPEDANCE_GAINS = ControlGains(kp=40, ki=400, kd=0, k=200, b=400, ff=128)

DEPHY_SLEEP_DURATION = 0.1


class DephyVoltageMode(ControlModeBase):
    def __init__(self, actuator: Union["DephyActpack", None] = None) -> None:
        super().__init__(
            control_mode_map=ControlModesMapping.VOLTAGE,
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
        )

    def __repr__(self) -> str:
        return f"DephyControlMode[{self.name}]"

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[DephyControlMode] Entering {self.name} control mode.")

        if self.actuator is None:
            raise ActuatorIsNoneException(mode=self.name)

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[DephyControlMode] Exiting {self.name} control mode.")
        self.actuator.stop_motor()
        time.sleep(DEPHY_SLEEP_DURATION)

    def set_gains(self, gains: ControlGains) -> None:
        LOGGER.info(
            msg=f"[{self._actuator.__repr__()}] {self.name} mode does not have gains."
        )

    def set_voltage(self, value: Union[float, int]):
        self.actuator.command_motor_voltage(value=int(value))

    def set_current(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.CURRENT),
            mode=self.name,
        )

    def set_position(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.POSITION),
            mode=self.name,
        )


class DephyCurrentMode(ControlModeBase):
    def __init__(self, actuator: Union["DephyActpack", None] = None) -> None:
        super().__init__(
            control_mode_map=ControlModesMapping.CURRENT,
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
            max_gains=ControlGains(kp=80, ki=800, kd=0, k=0, b=0, ff=128),
        )

    def __repr__(self) -> str:
        return f"DephyControlMode[{self.name}]"

    def _entry(self) -> None:

        LOGGER.debug(msg=f"[DephyControlMode] Entering {self.name} mode.")

        if self.actuator is None:
            raise ActuatorIsNoneException(mode=self.name)

        if not self.has_gains:
            self.set_gains()

        self.set_current(value=0)

    def _exit(self) -> None:

        LOGGER.debug(msg=f"[DephyControlMode] Exiting {self.name} mode.")

        # Is this necessary? This was a required step for older flexsea but not sure if it is needed anymore
        self.actuator.stop_motor()
        time.sleep(1 / self._actuator.frequency)

    def set_gains(self, gains: ControlGains = DEFAULT_CURRENT_GAINS) -> None:
        super().set_gains(gains)
        self.actuator.set_gains(
            kp=int(gains.kp),
            ki=int(gains.ki),
            kd=0,
            k=0,
            b=0,
            ff=int(gains.ff),
        )

    def set_current(self, value: Union[float, int]):
        self.actuator.command_motor_current(value=int(value))

    def set_voltage(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.VOLTAGE),
            mode=self.name,
        )

    def set_position(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.POSITION),
            mode=self.name,
        )


class DephyPositionMode(ControlModeBase):
    def __init__(self, actuator: Union["DephyActpack", None] = None) -> None:
        super().__init__(
            control_mode_map=ControlModesMapping.POSITION,
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
            max_gains=ControlGains(kp=1000, ki=1000, kd=1000, k=0, b=0, ff=0),
        )

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[DephyControlMode] Entering {self.name} mode.")

        if self.actuator is None:
            raise ActuatorIsNoneException(mode=self.name)

        if not self.has_gains:
            self.set_gains()

        self.set_position(value=self.actuator.motor_position)

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[DephyControlMode] Exiting {self.name} mode.")

        # Is this necessary? This was a required step for older flexsea but not sure if it is needed anymore
        self._actuator.stop_motor()
        time.sleep(0.1)

    def set_gains(
        self,
        gains: ControlGains = DEFAULT_POSITION_GAINS,
    ) -> None:
        super().set_gains(gains)
        self.actuator.set_gains(
            kp=int(gains.kp),
            ki=int(gains.ki),
            kd=int(gains.kd),
            k=0,
            b=0,
            ff=int(gains.ff),
        )

    def set_position(self, value: Union[float, int]):
        self.actuator.command_motor_position(
            value=int(
                (
                    value
                    + self.actuator.motor_zero_position
                    + self.actuator.motor_position_offset
                )
                / self.actuator.MOTOR_CONSTANTS.RAD_PER_COUNT
            ),
        )

    def set_current(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.CURRENT),
            mode=self.name,
        )

    def set_voltage(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.VOLTAGE),
            mode=self.name,
        )


class DephyImpedanceMode(ControlModeBase):
    def __init__(self, actuator: Union["DephyActpack", None] = None) -> None:
        super().__init__(
            control_mode_map=ControlModesMapping.IMPEDANCE,
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
            max_gains=ControlGains(kp=80, ki=800, kd=0, k=1000, b=1000, ff=128),
        )

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[DephyControlMode] Entering {self.name} mode.")

        if self.actuator is None:
            raise ActuatorIsNoneException(mode=self.name)

        if not self.has_gains:
            self.set_gains()

        self.set_position(self._actuator.motor_position)

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[DephyControlMode] Exiting {self.name} mode.")

        # Is this necessary? This was a required step for older flexsea but not sure if it is needed anymore
        self._actuator.stop_motor()
        time.sleep(1 / self._actuator.frequency)

    def set_gains(self, gains: ControlGains = DEFAULT_IMPEDANCE_GAINS):
        super().set_gains(gains)
        self.actuator.set_gains(
            kp=int(gains.kp),
            ki=int(gains.ki),
            kd=0,
            k=int(gains.k),
            b=int(gains.b),
            ff=int(gains.ff),
        )

    def set_position(self, value: Union[float, int]):
        self.actuator.command_motor_position(
            value=int(
                (
                    value
                    + self.actuator.motor_zero_position
                    + self.actuator.motor_position_offset
                )
                / self.actuator.MOTOR_CONSTANTS.RAD_PER_COUNT
            ),
        )

    def set_current(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.CURRENT),
            mode=self.name,
        )

    def set_voltage(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.VOLTAGE),
            mode=self.name,
        )


@dataclass(init=False)
class DephyActpackControlModes(ControlModesBase):
    def __init__(self, actuator: "DephyActpack") -> None:

        self.VOLTAGE = DephyVoltageMode(actuator=actuator)
        self.CURRENT = DephyCurrentMode(actuator=actuator)
        self.POSITION = DephyPositionMode(actuator=actuator)
        self.IMPEDANCE = DephyImpedanceMode(actuator=actuator)


class DephyActpack(ActuatorBase, Device):
    def __init__(
        self,
        tag: str = "DephyActpack",
        firmware_version: str = "7.2.0",
        port: str = "/dev/ttyACM0",
        gear_ratio: float = 1.0,
        baud_rate: int = 230400,
        frequency: int = 500,
        debug_level: int = 0,
        dephy_log: bool = False,
        offline: bool = False,
        stop_motor_on_disconnect: bool = False,
    ) -> None:
        dephy_control_modes = DephyActpackControlModes(self)
        ActuatorBase.__init__(
            self,
            tag=tag,
            control_modes=dephy_control_modes,
            default_control_mode=dephy_control_modes.VOLTAGE,
            gear_ratio=gear_ratio,
            motor_constants=MotorConstants(
                MOTOR_COUNT_PER_REV=16384,
                NM_PER_AMP=0.1133,
                IMPEDANCE_A=0.00028444,
                IMPEDANCE_C=0.0007812,
                MAX_CASE_TEMPERATURE=80,
                MAX_WINDING_TEMPERATURE=110,
                M_PER_SEC_SQUARED_ACCLSB=9.80665 / 8192,
            ),
            frequency=frequency,
            offline=offline,
        )

        if self.is_offline:
            self.port = port
            self.is_streaming: bool = False
            self.is_open: bool = False
        else:
            Device.__init__(
                self,
                firmwareVersion=firmware_version,
                port=port,
                baudRate=baud_rate,
                stopMotorOnDisconnect=stop_motor_on_disconnect,
            )

        self._debug_level: int = debug_level
        self._dephy_log: bool = dephy_log

        self._encoder_map = None
        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0
        self._motor_position_offset = 0.0
        self._joint_position_offset = 0.0
        self._joint_direction = 1

        self._is_homed: bool = False

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=self.max_winding_temperature,
            soft_border_C_windings=10,
            temp_limit_case=self.max_case_temperature,
            soft_border_C_case=10,
        )
        self._thermal_scale: float = 1.0

    def __repr__(self) -> str:
        return f"{self.tag}[DephyActpack]"

    @check_actuator_connection
    def start(self) -> None:
        try:
            self.open()
        except OSError as e:
            print("\n")
            self._log.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port '{self.port}'. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        self.start_streaming(self._frequency)
        self._data = self.read()

        time.sleep(0.1)
        self.mode.enter()

    @check_actuator_stream
    @check_actuator_open
    def stop(self) -> None:
        self.set_control_mode(mode=self.CONTROL_MODES.VOLTAGE)
        self.stop_motor()

        time.sleep(0.1)
        self.close()

    def update(self) -> None:

        self._data = self.read()

        self._thermal_model.T_c = self.case_temperature
        self._thermal_scale = self._thermal_model.update_and_get_scale(
            dt=1 / self.frequency,
            motor_current=self.motor_current,
        )
        if self.case_temperature >= self.max_case_temperature:
            self._log.error(
                msg=f"[{str.upper(self._name)}] Case thermal limit {self.max_case_temperature} reached. Stopping motor."
            )
            raise ThermalLimitException()

        if self.winding_temperature >= self.max_winding_temperature:
            self._log.error(
                msg=f"[{str.upper(self._name)}] Winding thermal limit {self.max_winding_temperature} reached. Stopping motor."
            )
            raise ThermalLimitException()
        # Check for thermal fault, bit 2 of the execute status byte
        if self._data["status_ex"] & 0b00000010 == 0b00000010:
            raise RuntimeError("Actpack Thermal Limit Tripped")

    def home(
        self,
        homing_voltage: int = 2000,
        homing_frequency: int = None,
        homing_direction: int = -1,
        joint_direction: int = -1,
        joint_position_offset: float = 0.0,
        motor_position_offset: float = 0.0,
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
            current_threshold (int): Current threshold in mA to stop homing the joint or actuator. This is used to detect if the actuator or joint has hit a hard stop. Default is 5000 mA.
            velocity_threshold (float): Velocity threshold in rad/s to stop homing the joint or actuator. This is also used to detect if the actuator or joint has hit a hard stop. Default is 0.001 rad/s.
        """
        is_homing = True

        if homing_frequency is None:
            homing_frequency = self.frequency

        self.set_control_mode(mode=self.CONTROL_MODES.VOLTAGE)

        self.set_motor_voltage(
            value=homing_direction * homing_voltage
        )  # mV, negative for counterclockwise

        _motor_encoder_array = []
        _joint_encoder_array = []

        time.sleep(0.1)

        try:
            while is_homing:
                self.update()
                time.sleep(1 / homing_frequency)

                _motor_encoder_array.append(self.motor_position)
                _joint_encoder_array.append(self.joint_position)

                if (
                    abs(self.output_velocity) <= velocity_threshold
                    or abs(self.motor_current) >= current_threshold
                ):
                    self.set_motor_voltage(value=0)
                    is_homing = False

        except KeyboardInterrupt:
            self.set_motor_voltage(value=0)
            LOGGER.info(msg=f"[{self.__repr__()}] Homing interrupted.")
            return
        except Exception as e:
            self.set_motor_voltage(value=0)
            LOGGER.error(msg=f"[{self.__repr__()}] Homing failed: {e}")
            return

        self.set_motor_zero_position(position=self.motor_position)
        self.set_joint_zero_position(position=self.joint_position)

        time.sleep(0.1)
        self.set_joint_direction(joint_direction)
        self.set_motor_position_offset(motor_position_offset)
        self.set_joint_position_offset(joint_position_offset)

        self._is_homed = True
        LOGGER.info(f"[{self.__repr__()}] Homing complete.")

        if os.path.isfile(path=f"./{self.tag}_encoder_map.npy"):
            coefficients = np.load(file=f"./{self.tag}_encoder_map.npy")
            self.set_encoder_map(np.polynomial.polynomial.Polynomial(coef=coefficients))
        else:
            LOGGER.debug(
                msg=f"[{self.__repr__()}] No encoder map found. Please call the make_encoder_map method to create one. The encoder map is used to estimate joint position more accurately."
            )

    def make_encoder_map(self, overwrite=False) -> None:
        """
        This method makes a lookup table to calculate the position measured by the joint encoder.
        This is necessary because the magnetic output encoders are nonlinear.
        By making the map while the joint is unloaded, joint position calculated by motor position * gear ratio
        should be the same as the true joint position.

        Output from this function is a file containing a_i values parameterizing the map

        Eqn: position = sum from i=0^5 (a_i*counts^i)

        Author: Kevin Best
                U-M Locolab | Neurobionics Lab
                Gitub: tkevinbest, https://github.com/tkevinbest
        """

        if not self.is_homed:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Please home the {self.tag} joint before making the encoder map."
            )
            return

        if os.path.exists(f"./{self.tag}_encoder_map.npy") and not overwrite:
            LOGGER.info(
                msg=f"[{self.__repr__()}] Encoder map exists. Skipping encoder map creation."
            )
            return

        self.set_control_mode(mode=self.CONTROL_MODES.CURRENT)
        self.set_current_gains()
        time.sleep(0.1)
        self.set_current_gains()

        self.set_joint_torque(value=0.0)
        time.sleep(0.1)
        self.set_joint_torque(value=0.0)

        _joint_encoder_array = []
        _output_position_array = []

        LOGGER.info(
            msg=f"[{self.__repr__()}] Please manually move the {self.tag} joint numerous times through its full range of motion for 10 seconds. \n{input('Press any key when you are ready to start.')}"
        )

        _start_time: float = time.time()

        try:
            while time.time() - _start_time < 10:
                LOGGER.info(
                    msg=f"[{self.__repr__()}] Mapping the {self.tag} joint encoder: {10 - time.time() + _start_time} seconds left."
                )
                self.update()
                _joint_encoder_array.append(self.joint_encoder_counts)
                _output_position_array.append(self.output_position)
                time.sleep(1 / self.frequency)

        except KeyboardInterrupt:
            LOGGER.warning(msg="Encoder map interrupted.")
            return

        LOGGER.info(
            msg=f"[{self.__repr__()}] You may now stop moving the {self.tag} joint."
        )

        _power = np.arange(4.0)
        _a_mat = np.array(_joint_encoder_array).reshape(-1, 1) ** _power
        _beta = np.linalg.lstsq(_a_mat, _output_position_array, rcond=None)
        _coeffs = _beta[0]

        self.set_encoder_map(np.polynomial.polynomial.Polynomial(coef=_coeffs))

        np.save(file=f"./{self.tag}_encoder_map.npy", arr=_coeffs)
        LOGGER.info(
            msg=f"[{self.__repr__()}] Encoder map saved to './{self.tag}_encoder_map.npy'."
        )

    def set_control_mode(self, mode: ControlModeBase) -> None:
        """
        Sets the control mode of the actuator.

        Args:
            mode (ControlModeBase): The control mode to set. This is a ControlModeBase object.
        """
        super().set_control_mode(mode)

    def set_motor_torque(self, value: float) -> None:
        """
        Sets the motor torque in Nm.

        Args:
            value (float): The torque to set in Nm.
        """
        self.set_motor_current(
            value / self.MOTOR_CONSTANTS.NM_PER_MILLIAMP,
        )

    def set_joint_torque(self, value: float) -> None:
        """
        Set the joint torque of the joint.
        This is the torque that is applied to the joint, not the motor.

        Args:
            value (float): torque in N_m
        """
        self.set_motor_torque(value=value / self.gear_ratio)

    @deprecated_with_routing(alternative_func=set_joint_torque)
    def set_output_torque(self, value: float) -> None:
        """
        Set the output torque of the joint.
        This is the torque that is applied to the joint, not the motor.

        Args:
            value (float): torque in N_m
        """
        self.set_motor_torque(value=value / self.gear_ratio)

    def set_output_position(self, value: float) -> None:
        """
        Set the output position of the joint.
        This is the desired position of the joint, not the motor.
        This method automatically handles scaling by the gear raito.

        Args:
            value (float): position in radians
        """
        self.set_motor_position(value=value * self.gear_ratio)

    def set_motor_current(
        self,
        value: float,
    ):
        """
        Sets the motor current in mA.

        Args:
            value (float): The current to set in mA.
        """
        self.mode.set_current(value=value)

    @deprecated_with_routing(alternative_func=set_motor_current)
    def set_current(self, value: float) -> None:

        self.mode.set_current(value=value)

    def set_motor_voltage(self, value: float) -> None:
        """
        Sets the motor voltage in mV.

        Args:
            voltage_value (float): The voltage to set in mV.
        """
        self.mode.set_voltage(
            value,
        )

    @deprecated_with_routing(alternative_func=set_motor_voltage)
    def set_voltage(self, value: float) -> None:

        self.mode.set_voltage(
            value,
        )

    def set_motor_position(self, value: float) -> None:
        """
        Sets the motor position in radians.
        If in impedance mode, this sets the equilibrium angle in radians.

        Args:
            position (float): The position to set
        """
        self.mode.set_position(value)

    def set_position_gains(
        self,
        kp: int = DEFAULT_POSITION_GAINS.kp,
        ki: int = DEFAULT_POSITION_GAINS.ki,
        kd: int = DEFAULT_POSITION_GAINS.kd,
        ff: int = DEFAULT_POSITION_GAINS.ff,
    ) -> None:
        """
        Sets the position gains in arbitrary Dephy units.

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            kd (int): The derivative gain
            ff (int): The feedforward gain
        """
        self.mode.set_gains(ControlGains(kp=kp, ki=ki, kd=kd, k=0, b=0, ff=ff))  # type: ignore

    def set_current_gains(
        self,
        kp: int = DEFAULT_CURRENT_GAINS.kp,
        ki: int = DEFAULT_CURRENT_GAINS.ki,
        ff: int = DEFAULT_CURRENT_GAINS.ff,
    ) -> None:
        """
        Sets the current gains in arbitrary Dephy units.

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            ff (int): The feedforward gain
        """
        self.mode.set_gains(ControlGains(kp=kp, ki=ki, kd=0, k=0, b=0, ff=ff))  # type: ignore

    def set_motor_impedance(
        self,
        kp: int = 40,
        ki: int = 400,
        K: float = 0.08922,
        B: float = 0.0038070,
        ff: int = 128,
    ) -> None:
        """
        Set the impedance gains of the motor in real units: Nm/rad and Nm/rad/s.

        Args:
            kp (int): Proportional gain. Defaults to 40.
            ki (int): Integral gain. Defaults to 400.
            K (float): Spring constant. Defaults to 0.08922 Nm/rad.
            B (float): Damping constant. Defaults to 0.0038070 Nm/rad/s.
            ff (int): Feedforward gain. Defaults to 128.
        """
        self.set_impedance_gains(
            kp=kp,
            ki=ki,
            K=int(K * self.MOTOR_CONSTANTS.NM_PER_RAD_TO_K),
            B=int(B * self.MOTOR_CONSTANTS.NM_S_PER_RAD_TO_B),
            ff=ff,
        )

    def set_joint_impedance(
        self,
        kp: int = 40,
        ki: int = 400,
        K: float = 100.0,
        B: float = 3.0,
        ff: int = 128,
    ) -> None:
        """
        Set the impedance gains of the joint in real units: Nm/rad and Nm/rad/s.
        This sets the impedance at the output and automatically scales based on gear raitos.

        Conversion:
            K_motor = K_joint / (gear_ratio ** 2)
            B_motor = B_joint / (gear_ratio ** 2)

        Args:
            kp (int): Proportional gain. Defaults to 40.
            ki (int): Integral gain. Defaults to 400.
            K (float): Spring constant. Defaults to 100 Nm/rad.
            B (float): Damping constant. Defaults to 3.0 Nm/rad/s.
            ff (int): Feedforward gain. Defaults to 128.
        """
        self.set_motor_impedance(
            kp=kp,
            ki=ki,
            K=K / (self.gear_ratio**2),
            B=B / (self.gear_ratio**2),
            ff=ff,
        )

    def set_impedance_gains(
        self,
        kp: int = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: int = DEFAULT_IMPEDANCE_GAINS.ki,
        k: int = DEFAULT_IMPEDANCE_GAINS.k,
        b: int = DEFAULT_IMPEDANCE_GAINS.b,
        ff: int = DEFAULT_IMPEDANCE_GAINS.ff,
    ) -> None:
        """
        Sets the impedance gains in arbitrary actpack units.
        See Dephy's webpage for conversions or use other library methods that handle conversion for you.

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            k (int): The spring constant
            b (int): The damping constant
            ff (int): The feedforward gain
        """
        self.mode.set_gains(ControlGains(kp=kp, ki=ki, kd=0, k=k, b=b, ff=ff))  # type: ignore

    def set_encoder_map(self, encoder_map) -> None:
        """Sets the joint encoder map"""
        self._encoder_map = encoder_map

    def set_motor_zero_position(self, position: float) -> None:
        """Sets motor zero position in radians"""
        self._motor_zero_position = position

    def set_motor_position_offset(self, position: float) -> None:
        """Sets joint offset position in radians"""
        self._motor_position_offset = position

    def set_joint_zero_position(self, position: float) -> None:
        """Sets joint zero position in radians"""
        self._joint_zero_position = position

    def set_joint_position_offset(self, position: float) -> None:
        """Sets joint offset position in radians"""
        self._joint_position_offset = position

    def set_joint_direction(self, direction: float) -> None:
        """Sets joint direction to 1 or -1"""
        self._joint_direction = direction

    @property
    def joint_zero_position(self) -> float:
        """Joint encoder zero position in radians."""
        return self._joint_zero_position

    @property
    def joint_position_offset(self) -> float:
        """Joint encoder offset in radians."""
        return self._joint_position_offset

    @property
    def joint_direction(self) -> float:
        """Joint direction: 1 or -1"""
        return self._joint_direction

    @property
    def encoder_map(self):
        """Polynomial coefficients defining the joint encoder map from counts to radians."""
        return self._encoder_map

    @property
    def motor_zero_position(self) -> float:
        """Motor encoder zero position in radians."""
        return self._motor_zero_position

    @property
    def motor_position_offset(self) -> float:
        """Motor encoder offset in radians."""
        return self._motor_position_offset

    @property
    def motor_voltage(self) -> float:
        """Q-axis motor voltage in mV."""
        if self._data is not None:
            return float(self._data["mot_volt"])
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_current(self) -> float:
        if self._data is not None:
            return float(self._data["mot_cur"])
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_torque(self) -> float:
        if self._data is not None:
            return float(self._data["mot_cur"] * self.MOTOR_CONSTANTS.NM_PER_MILLIAMP)
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_position(self) -> float:
        if self._data is not None:
            return (
                float(self._data["mot_ang"] * self.MOTOR_CONSTANTS.RAD_PER_COUNT)
                - self.motor_zero_position
                - self.motor_position_offset
            )
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_encoder_counts(self) -> int:
        """Raw reading from motor encoder in counts."""
        if self._data is not None:
            return int(self._data["mot_ang"])
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0."
            )
            return 0

    @property
    def joint_encoder_counts(self) -> int:
        """Raw reading from joint encoder in counts."""
        if self._data is not None:
            return int(self._data["ank_ang"])
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0."
            )
            return 0

    @property
    def motor_velocity(self) -> float:
        if self._data is not None:
            return int(self._data["mot_vel"]) * self.MOTOR_CONSTANTS.RAD_PER_DEG
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_acceleration(self) -> float:
        if self._data is not None:
            return float(self._data["mot_acc"])
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def battery_voltage(self) -> float:
        """Battery voltage in mV."""
        if self._data is not None:
            return float(self._data["batt_volt"])
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def battery_current(self) -> float:
        """Battery current in mA."""
        if self._data is not None:
            return float(self._data["batt_curr"])
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def joint_position(self) -> float:
        if self._data is not None:
            return (
                float(self._data["ank_ang"] * self.MOTOR_CONSTANTS.RAD_PER_COUNT)
                - self.joint_zero_position
                - self.joint_position_offset
            ) * self.joint_direction
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def joint_velocity(self) -> float:
        if self._data is not None:
            return (
                float(self._data["ank_vel"] * self.MOTOR_CONSTANTS.RAD_PER_DEG)
                * self.joint_direction
            )
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def output_position(self) -> float:
        """
        Position of the output in radians.
        This is calculated by scaling the motor angle with the gear ratio.
        Note that this method does not consider compliance from an SEA.
        """
        return self.motor_position / self.gear_ratio

    @property
    def output_velocity(self) -> float:
        """
        Velocity of the output in radians.
        This is calculated by scaling the motor angle with the gear ratio.
        Note that this method does not consider compliance from an SEA.
        """
        return self.motor_velocity / self.gear_ratio

    @property
    def joint_torque(self) -> float:
        """
        Torque at the joint output in Nm.
        This is calculated using motor current, k_t, and the gear ratio.
        """
        return self.motor_torque * self.gear_ratio

    @property
    def case_temperature(self) -> float:
        if self._data is not None:
            return float(self._data["temperature"])
        else:
            LOGGER.warning(
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
    def genvars(self):
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
            LOGGER.warning(
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
            return float(
                self._data["accelx"] * self.MOTOR_CONSTANTS.M_PER_SEC_SQUARED_ACCLSB
            )
        else:
            LOGGER.warning(
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
            return float(
                self._data["accely"] * self.MOTOR_CONSTANTS.M_PER_SEC_SQUARED_ACCLSB
            )
        else:
            LOGGER.warning(
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
            return float(
                self._data["accelz"] * self.MOTOR_CONSTANTS.M_PER_SEC_SQUARED_ACCLSB
            )
        else:
            LOGGER.warning(
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
            return float(self._data["gyrox"] * self.MOTOR_CONSTANTS.RAD_PER_SEC_GYROLSB)
        else:
            LOGGER.warning(
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
            return float(self._data["gyroy"] * self.MOTOR_CONSTANTS.RAD_PER_SEC_GYROLSB)
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def gyroz(self) -> float:
        """
        Angular velocity in z direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data["gyroz"] * self.MOTOR_CONSTANTS.RAD_PER_SEC_GYROLSB)
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def thermal_scaling_factor(self) -> float:
        """
        Scale factor to use in torque control, in [0,1].
        If you scale the torque command by this factor, the motor temperature will never exceed max allowable temperature.
        For a proof, see paper referenced in thermal model.
        """
        return self._thermal_scale

    @property
    def is_streaming(self) -> bool:

        return self.streaming

    @property
    def is_open(self) -> bool:

        return self.connected


if __name__ == "__main__":
    knee = DephyActpack(
        tag="knee",
        firmware_version="7.2.0",
        port="/dev/ttyACM0",
        gear_ratio=1.0,
        baud_rate=230400,
        frequency=500,
        debug_level=0,
        dephy_log=False,
        offline=True,
        stop_motor_on_disconnect=False,
    )
