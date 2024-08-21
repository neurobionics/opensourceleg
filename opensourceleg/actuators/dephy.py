from typing import Any, Callable, Union, overload

import os
import time
from ctypes import c_int
from dataclasses import dataclass
from unittest.mock import Mock

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


def _dephy_voltage_mode_exit(dephy_actuator: "DephyActuator") -> None:
    dephy_actuator.stop_motor()
    time.sleep(DEPHY_SLEEP_DURATION)


def _dephy_current_mode_exit(dephy_actuator: "DephyActuator") -> None:
    dephy_actuator.stop_motor()
    time.sleep(DEPHY_SLEEP_DURATION)


def _dephy_position_mode_exit(dephy_actuator: "DephyActuator") -> None:
    dephy_actuator.stop_motor()
    time.sleep(DEPHY_SLEEP_DURATION)


def _dephy_impedance_mode_exit(dephy_actuator: "DephyActuator") -> None:
    dephy_actuator.stop_motor()
    time.sleep(DEPHY_SLEEP_DURATION)


DEPHY_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    POSITION=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=_dephy_position_mode_exit,
        has_gains=False,
        max_gains=ControlGains(kp=1000, ki=1000, kd=1000, k=0, b=0, ff=0),
    ),
    CURRENT=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=_dephy_current_mode_exit,
        has_gains=False,
        max_gains=ControlGains(kp=80, ki=800, kd=0, k=0, b=0, ff=128),
    ),
    VOLTAGE=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=_dephy_voltage_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
    IMPEDANCE=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=_dephy_impedance_mode_exit,
        has_gains=False,
        max_gains=ControlGains(kp=80, ki=800, kd=0, k=1000, b=1000, ff=128),
    ),
)


class DephyActuator(ActuatorBase, Device):
    def __init__(
        self,
        tag: str = "DephyActuator",
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
        ActuatorBase.__init__(
            self,
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=DEPHY_ACTUATOR_CONSTANTS,
            frequency=frequency,
            offline=offline,
        )

        if self.is_offline:
            self.port = port
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

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=self.max_winding_temperature,
            soft_border_C_windings=10,
            temp_limit_case=self.max_case_temperature,
            soft_border_C_case=10,
        )
        self._thermal_scale: float = 1.0

        self._mode = CONTROL_MODES.VOLTAGE

    def __repr__(self) -> str:
        return f"{self.tag}[DephyActuator]"

    @property
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        return DEPHY_CONTROL_MODE_CONFIGS

    @check_actuator_connection
    def start(self) -> None:
        try:
            self.open()
        except OSError as e:
            print("\n")
            LOGGER.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port '{self.port}'. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        self.start_streaming(self._frequency)
        self._data = self.read()

        time.sleep(0.1)
        # self._get_control_mode_config(self._mode).entry_callback(self)

        default_mode_config = self._get_control_mode_config(self._mode)
        if default_mode_config:
            default_mode_config.entry_callback(self)

    @check_actuator_stream
    @check_actuator_open
    def stop(self) -> None:
        self.set_control_mode(mode=CONTROL_MODES.VOLTAGE)
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
            LOGGER.error(
                msg=f"[{str.upper(self.tag)}] Case thermal limit {self.max_case_temperature} reached. Stopping motor."
            )
            raise ThermalLimitException()

        if self.winding_temperature >= self.max_winding_temperature:
            LOGGER.error(
                msg=f"[{str.upper(self.tag)}] Winding thermal limit {self.max_winding_temperature} reached. Stopping motor."
            )
            raise ThermalLimitException()
        # Check for thermal fault, bit 2 of the execute status byte
        if self._data["status_ex"] & 0b00000010 == 0b00000010:
            raise RuntimeError("Actpack Thermal Limit Tripped")

    def home(
        self,
        homing_frequency: int = None,
        homing_voltage: int = 2000,
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
        homing_frequency = (
            homing_frequency if homing_frequency is not None else self.frequency
        )

        self.set_control_mode(mode=CONTROL_MODES.VOLTAGE)

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

        self.set_motor_zero_position(value=self.motor_position)
        self.set_joint_zero_position(value=self.joint_position)

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

        self.set_control_mode(mode=CONTROL_MODES.CURRENT)
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

    def set_motor_current(
        self,
        value=float,
    ):
        """
        Sets the motor current in mA.

        Args:
            value (float): The current to set in mA.
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
        """
        self.command_motor_voltage(value=int(value))

    @deprecated_with_routing(alternative_func=set_motor_voltage)
    def set_voltage(self, value: float) -> None:

        self.command_motor_voltage(value=int(value))

    def set_motor_position(self, value=float) -> None:
        """
        Sets the motor position in radians.
        If in impedance mode, this sets the equilibrium angle in radians.

        Args:
            value (float): The position to set
        """
        self.command_motor_position(
            value=int(
                (value + self.motor_zero_position + self.motor_position_offset)
                / self.MOTOR_CONSTANTS.RAD_PER_COUNT
            ),
        )

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
        """
        self.set_gains(
            kp=int(kp),
            ki=int(ki),
            kd=int(kd),
            k=0,
            b=0,
            ff=int(ff),
        )

    def set_motor_impedance(
        self,
        kp: float = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: float = DEFAULT_IMPEDANCE_GAINS.ki,
        kd: float = DEFAULT_IMPEDANCE_GAINS.kd,
        k: float = 0.08922,
        b: float = 0.0038070,
        ff: float = 128,
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
        """
        self.set_impedance_gains(
            kp=kp,
            ki=ki,
            kd=kd,
            k=int(k * self.MOTOR_CONSTANTS.NM_PER_RAD_TO_K),
            b=int(b * self.MOTOR_CONSTANTS.NM_S_PER_RAD_TO_B),
            ff=ff,
        )

    def set_joint_impedance(
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
        """
        self.set_gains(
            kp=int(kp),
            ki=int(ki),
            kd=int(kd),
            k=int(k),
            b=int(b),
            ff=int(ff),
        )

    def set_encoder_map(self, encoder_map) -> None:
        """Sets the joint encoder map"""
        self._encoder_map = encoder_map

    @property
    def encoder_map(self):
        """Polynomial coefficients defining the joint encoder map from counts to radians."""
        if getattr(self, "_encoder_map", None) is not None:
            return self._encoder_map
        else:
            LOGGER.debug(
                msg="Encoder map is not set. Please call the make_encoder_map method to create one."
            )
            return None

    @property
    def motor_voltage(self) -> float:
        """Q-axis motor voltage in mV."""
        if self._data is not None:
            return float(self._data["mot_volt"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_current(self) -> float:
        if self._data is not None:
            return float(self._data["mot_cur"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_torque(self) -> float:
        if self._data is not None:
            return float(self._data["mot_cur"] * self.MOTOR_CONSTANTS.NM_PER_MILLIAMP)
        else:
            LOGGER.debug(
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
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_encoder_counts(self) -> int:
        """Raw reading from motor encoder in counts."""
        if self._data is not None:
            return int(self._data["mot_ang"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0."
            )
            return 0

    @property
    def joint_encoder_counts(self) -> int:
        """Raw reading from joint encoder in counts."""
        if self._data is not None:
            return int(self._data["ank_ang"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0."
            )
            return 0

    @property
    def motor_velocity(self) -> float:
        if self._data is not None:
            return int(self._data["mot_vel"]) * RAD_PER_DEG
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_acceleration(self) -> float:
        if self._data is not None:
            return float(self._data["mot_acc"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def battery_voltage(self) -> float:
        """Battery voltage in mV."""
        if self._data is not None:
            return float(self._data["batt_volt"])
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def battery_current(self) -> float:
        """Battery current in mA."""
        if self._data is not None:
            return float(self._data["batt_curr"])
        else:
            LOGGER.debug(
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
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def joint_velocity(self) -> float:
        if self._data is not None:
            return float(self._data["ank_vel"] * RAD_PER_DEG) * self.joint_direction
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

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
        If you scale the torque command by this factor, the motor temperature will never exceed max allowable temperature.
        For a proof, see paper referenced in thermal model.
        """
        return self._thermal_scale


def _dephy_legacy_voltage_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[DephyControlMode] Entering Voltage control mode.")


def _dephy_legacy_voltage_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[DephyControlMode] Exiting Voltage control mode.")
    dephy_actuator.set_motor_voltage(value=0)
    time.sleep(DEPHY_SLEEP_DURATION)


def _dephy_legacy_current_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[DephyControlMode] Entering Current control mode.")


def _dephy_legacy_current_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[DephyControlMode] Exiting Current control mode.")
    dephy_actuator.set_control_mode(mode=CONTROL_MODES.VOLTAGE)
    dephy_actuator.set_motor_voltage(value=0)
    time.sleep(DEPHY_SLEEP_DURATION)


def _dephy_legacy_position_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[DephyControlMode] Entering Position control mode.")


def _dephy_legacy_position_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[DephyControlMode] Exiting Position control mode.")
    dephy_actuator.set_control_mode(mode=CONTROL_MODES.VOLTAGE)
    dephy_actuator.set_motor_voltage(value=0)
    time.sleep(DEPHY_SLEEP_DURATION)


def _dephy_legacy_impedance_mode_entry(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[DephyControlMode] Entering Impedance control mode.")


def _dephy_legacy_impedance_mode_exit(dephy_actuator: "DephyActuator") -> None:
    LOGGER.debug(msg=f"[DephyControlMode] Exiting Impedance control mode.")
    dephy_actuator.set_control_mode(mode=CONTROL_MODES.VOLTAGE)
    dephy_actuator.set_motor_voltage(value=0)
    time.sleep(DEPHY_SLEEP_DURATION)


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


class DephyLegacyActuator(ActuatorBase, Device):
    def __init__(
        self,
        tag: str = "DephyActuator",
        port: str = "/dev/ttyACM0",
        gear_ratio: float = 1.0,
        baud_rate: int = 230400,
        frequency: int = 500,
        debug_level: int = 0,
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

        if self.is_offline:
            self.port = port
            self.is_streaming: bool = False
            self.is_open: bool = False
        else:
            Device.__init__(self, port=port, baud_rate=baud_rate)

        self._debug_level: int = debug_level
        self._dephy_log: bool = dephy_log

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=self.max_winding_temperature,
            soft_border_C_windings=10,
            temp_limit_case=self.max_case_temperature,
            soft_border_C_case=10,
        )
        self._thermal_scale: float = 1.0

    def __repr__(self) -> str:
        return f"{self.tag}[DephyLegacyActuator]"

    @check_actuator_connection
    def start(self) -> None:
        try:
            self.open(
                freq=self._frequency,
                log_level=self._debug_level,
                log_enabled=self._dephy_log,
            )
        except OSError as e:
            print("\n")
            LOGGER.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port '{self.port}'. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        self._data = self.read()

        time.sleep(0.1)
        default_mode_config = self._get_control_mode_config(self._mode)
        if default_mode_config:
            default_mode_config.entry_callback(self)

    @check_actuator_stream
    @check_actuator_open
    def stop(self) -> None:
        self.set_control_mode(mode=CONTROL_MODES.VOLTAGE)
        self.set_motor_voltage(value=0)

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
            LOGGER.error(
                msg=f"[{str.upper(self.tag)}] Case thermal limit {self.max_case_temperature} reached. Stopping motor."
            )
            raise ThermalLimitException()

        if self.winding_temperature >= self.max_winding_temperature:
            LOGGER.error(
                msg=f"[{str.upper(self.tag)}] Winding thermal limit {self.max_winding_temperature} reached. Stopping motor."
            )
            raise ThermalLimitException()
        # Check for thermal fault, bit 2 of the execute status byte
        if self._data.status_ex & 0b00000010 == 0b00000010:
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
            homing_direction (int): Direction to move the actuator during homing. Default is -1.
            joint_direction (int): Direction to move the joint during homing. Default is -1.
            joint_position_offset (float): Offset in radians to add to the joint position. Default is 0.0.
            motor_position_offset (float): Offset in radians to add to the motor position. Default is 0.0.
            current_threshold (int): Current threshold in mA to stop homing the joint or actuator. This is used to detect if the actuator or joint has hit a hard stop. Default is 5000 mA.
            velocity_threshold (float): Velocity threshold in rad/s to stop homing the joint or actuator. This is also used to detect if the actuator or joint has hit a hard stop. Default is 0.001 rad/s.

        """
        is_homing = True
        homing_frequency = (
            homing_frequency if homing_frequency is not None else self.frequency
        )

        self.set_control_mode(mode=CONTROL_MODES.VOLTAGE)

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

        self.set_motor_zero_position(value=self.motor_position)
        self.set_joint_zero_position(value=self.joint_position)

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

        self.set_control_mode(mode=CONTROL_MODES.CURRENT)
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

    def set_motor_current(
        self,
        value: float,
    ):
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
            value=int(
                (value + self.motor_zero_position + self.motor_position_offset)
                / self.MOTOR_CONSTANTS.RAD_PER_COUNT
            ),
        )

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
        """
        self.set_gains(
            kp=int(kp),
            ki=int(ki),
            kd=int(kd),
            k=0,
            b=0,
            ff=int(ff),
        )

    def set_joint_impedance(
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
        """
        self.set_impedance_gains(
            kp=kp,
            ki=ki,
            kd=kd,
            k=int(k * self.MOTOR_CONSTANTS.NM_PER_RAD_TO_K),
            b=int(b * self.MOTOR_CONSTANTS.NM_S_PER_RAD_TO_B),
            ff=ff,
        )

    def set_encoder_map(self, encoder_map) -> None:
        """Sets the joint encoder map"""
        self._encoder_map = encoder_map

    @property
    def encoder_map(self):
        """Polynomial coefficients defining the joint encoder map from counts to radians."""
        if getattr(self, "_encoder_map", None) is not None:
            return self._encoder_map
        else:
            LOGGER.warning(
                msg="Encoder map is not set. Please call the make_encoder_map method to create one."
            )
            return None

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
        if self._data is not None:
            return float(self._data.mot_cur * self.MOTOR_CONSTANTS.NM_PER_MILLIAMP)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_position(self) -> float:
        if self._data is not None:
            return (
                float(self._data.mot_ang * self.MOTOR_CONSTANTS.RAD_PER_COUNT)
                - self.motor_zero_position
                - self.motor_position_offset
            )
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
    def joint_encoder_counts(self) -> int:
        """Raw reading from joint encoder in counts."""
        if self._data is not None:
            return int(self._data.ank_ang)
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0."
            )
            return 0

    @property
    def motor_velocity(self) -> float:
        if self._data is not None:
            return int(self._data.mot_vel) * RAD_PER_DEG
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_acceleration(self) -> float:
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
    def joint_position(self) -> float:
        if self._data is not None:
            return (
                float(self._data.ank_ang * self.MOTOR_CONSTANTS.RAD_PER_COUNT)
                - self.joint_zero_position
                - self.joint_position_offset
            ) * self.joint_direction
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def joint_velocity(self) -> float:
        if self._data is not None:
            return float(self._data.ank_vel * RAD_PER_DEG) * self.joint_direction
        else:
            LOGGER.debug(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

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
    def genvars(self):
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

    @property
    def thermal_scaling_factor(self) -> float:
        """
        Scale factor to use in torque control, in [0,1].
        If you scale the torque command by this factor, the motor temperature will never exceed max allowable temperature.
        For a proof, see paper referenced in thermal model.
        """
        return self._thermal_scale


if __name__ == "__main__":
    knee = DephyActuator(
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
