"""
Actuators Interface Generalized
05/2024
"""

from typing import Any, Callable, Protocol, Union, overload

import ctypes
import os
import time
from abc import ABC, abstractmethod
from ctypes import c_int
from curses.ascii import ctrl
from dataclasses import dataclass

# To be removed after Generalization
import flexsea.fx_enums as fxe
import numpy as np

# To be removed after Generalization
from flexsea.device import Device

from ..tools.logger import Logger
from .thermal import ThermalModel

"""
    User Guide to opensourceleg.hardware.actuators
    
    ```
    ```
    
    Returns:
        _type_: _description_
"""


@dataclass
class ControlModes:
    """
    Control modes definition.

    All members are in ctypes.c_int type. Will Change later?

    Available modes are voltage, current, position, impedance.
    """

    mode_voltage: ctypes.c_int
    mode_current: ctypes.c_int
    mode_position: ctypes.c_int
    mode_impedance: ctypes.c_int

    def __repr__(self) -> str:
        return f"ControlModes"


@dataclass
class ControlGains:
    """
    Dataclass for controller gains

    Args:
        kp (int): Proportional gain
        ki (int): Integral gain
        kd (int): Derivative gain
        K (int): Stiffness of the impedance controller
        B (int): Damping of the impedance controller
        ff (int): Feedforward gain
    """

    kp: int = 0
    ki: int = 0
    kd: int = 0
    K: int = 0
    B: int = 0
    ff: int = 0

    def __repr__(self) -> str:
        return f"kp={self.kp}, ki={self.ki}, kd={self.kd}, K={self.K}, B={self.B}, ff={self.ff}"


@dataclass
class MecheConsts:
    """
    Imports necessary constants, compute the rest and protects them by @property

    Returns:
        _type_: _description_
    """

    def __repr__(self) -> str:
        return f"MecheConsts"

    MOTOR_COUNT_PER_REV: float = 16384
    NM_PER_AMP: float = 0.1133
    IMPEDANCE_A: float = 0.00028444
    IMPEDANCE_C: float = 0.0007812
    MAX_CASE_TEMPERATURE: float = 80
    M_PER_SEC_SQUARED_ACCLSB: float = 9.80665 / 8192

    # Should be only containing the members above. Others directly implemented in the Actuators class
    @property
    def NM_PER_MILLIAMP(self) -> float:
        return self.NM_PER_AMP / 1000

    @property
    def RAD_PER_COUNT(self) -> float:
        return 2 * np.pi / self.MOTOR_COUNT_PER_REV

    @property
    def RAD_PER_DEG(self) -> float:
        return np.pi / 180

    @property
    def RAD_PER_SEC_GYROLSB(self) -> float:
        return np.pi / 180 / 32.8

    @property
    def NM_PER_RAD_TO_K(self) -> float:
        return self.RAD_PER_COUNT / self.IMPEDANCE_C * 1e3 / self.NM_PER_AMP

    @property
    def NM_S_PER_RAD_TO_B(self) -> float:
        return self.RAD_PER_DEG / self.IMPEDANCE_A * 1e3 / self.NM_PER_AMP


@dataclass
class ActuatorComm:
    """
    Configuration of Actuator Definition & Communication.

    Args:
        ABC (_type_): _description_
    """

    name: str = "DephyActpack"
    port: str = "/dev/ttyACM0"
    baud_rate: int = 230400
    frequency: int = 500
    logger: Logger = Logger()

    debug_level: int = 0
    log: bool = False


@dataclass
class SafetySpec:
    MAX_VOLTAGE = 0
    MAX_CURRENT = 0
    MAX_POSITION_COUNT = 0
    MAX_IMPEDANCE_KP = 0
    MAX_IMPEDANCE_KI = 0
    MAX_IMPEDANCE_KD = 0
    MAX_IMPEDANCE_K = 0
    MAX_IMPEDANCE_B = 0
    MAX_IMPEDANCE_FF = 0
    MAX_POSITION_KP = 0
    MAX_POSITION_KI = 0
    MAX_POSITION_KD = 0
    MAX_CURRENT_KP = 0
    MAX_CURRENT_KI = 0
    MAX_CURRENT_KD = 0


class ActuatorMode:

    def __init__(self, device: "Actuator") -> None:

        # self._control_mode: c_int = control_mode
        self._which_mode: str = "voltage"
        self._control_mode: c_int
        self._device: Actuator = device
        self._entry_callback: Callable[[], None] = lambda: None
        self._exit_callback: Callable[[], None] = lambda: None
        self._has_gains = False
        """state machine design?
        """

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, ActuatorMode):
            return self._control_mode == __o._control_mode
        return False

    def __str__(self) -> str:
        return str(object=self._control_mode)

    def __repr__(self) -> str:
        return f"ActpackMode[{self._control_mode}]"

    @property
    def mode(self) -> c_int:
        """
        Control mode

        Returns:
            c_int: Control mode
        """
        return self._control_mode

    # @property
    # def has_gains(self) -> bool:
    #     """
    #     Whether the mode has gains

    #     Returns:
    #         bool: True if the mode has gains, False otherwise
    #     """
    #     return self._has_gains

    def enter(self) -> None:
        """
        Calls the entry callback
        """
        self._entry_callback()

    def exit(self) -> None:
        """
        Calls the exit callback
        """
        self._exit_callback()

    # To be modified
    def transition(self, to_which_mode: str, to_mode_pass: c_int) -> None:
        """
        Transition to another mode. Calls the exit callback of the current mode
        and the entry callback of the new mode.

        Args:
            to_state (ActpackMode): Mode to transition to
        """
        self.exit()
        # to_state.enter()
        self._which_mode = to_which_mode
        self._control_mode = to_mode_pass
        self.enter()

    # def _in_which_mode(self):
    #     return self._which_mode


class ActuatorAction(SafetySpec):
    """
    This class contains all the methods a

    Args:
        control_mode (c_int): Control mode
        device (DephyActpack): Dephy Actpack
    """

    def __init__(
        self,
        device: "Actuator",
        # Mode: ActuatorMode
    ) -> None:
        # super().__init__()
        self._device: Actuator = device

    def _set_voltage(self, which_mode: str, voltage_value: float):
        if which_mode == "voltage":
            if self.safety_check("voltage", voltage_value) == True:
                pass
            else:
                # should raise an error here
                pass
        else:
            # should raise an error here
            pass

    def _set_current(
        self, which_mode: str, current_value: float, CurrentGain: ControlGains
    ):
        if which_mode == "current":
            if self.safety_check("current", current_value, CurrentGain) == True:
                pass
            else:
                # should raise an error here
                pass
        else:
            # should raise an error here
            pass

    def _set_impedance(self, which_mode: str, ImpedanceGains: ControlGains):
        if which_mode == "impedance":
            if self.safety_check("impedance", ImpedanceGains) == True:
                pass
            else:
                # should raise an error here
                pass

        else:
            # should raise an error here
            pass

    def _set_position(self, which_mode: str, PositionGains: ControlGains):
        if which_mode == "position":
            if self.safety_check("impedance", PositionGains) == True:
                pass
            else:
                # should raise an error here
                pass
        else:
            # should raise an error here
            pass

    # @abstractmethod
    def safety_check(self, which_mode: str, *argv) -> bool:
        if which_mode == "voltage":
            if type(argv[1]) == float:
                return True
            else:
                # should raise an error here
                return False
        elif which_mode == "current":
            if type(argv[1]) == float & type(argv[2] == ControlGains):
                assert 0 <= argv[2].kp <= 80, "kp must be between 0 and 80"
                assert 0 <= argv[2].ki <= 800, "ki must be between 0 and 800"
                assert 0 <= argv[2].ff <= 128, "ff must be between 0 and 128"
                # check for max current
                return True
            else:
                # should raise an error here
                return False
        elif which_mode == "impedance":
            if type(argv[1]) == ControlGains:
                assert 0 <= argv[2].kp <= 80, "kp must be between 0 and 80"
                assert 0 <= argv[2].ki <= 800, "ki must be between 0 and 800"
                assert 0 <= argv[2].ff <= 128, "ff must be between 0 and 128"
                assert 0 <= argv[2].K, "K must be greater than 0"
                assert 0 <= argv[2].B, "B must be greater than 0"
                return True
            else:
                # should raise an error here
                return False
        elif which_mode == "position":
            if type(argv[1]) == ControlGains:
                assert 0 <= argv[2].kp <= 1000, "kp must be between 0 and 1000"
                assert 0 <= argv[2].ki <= 1000, "ki must be between 0 and 1000"
                assert 0 <= argv[2].kd <= 1000, "kd must be between 0 and 1000"
                return True
            else:
                # should raise an error here
                return False
        else:
            # error: control mode error
            return False


class Actuator(ActuatorAction, ABC):

    def __init__(
        self,
        Gains: ControlGains,
        MecheSpecs: MecheConsts,
        Communication: ActuatorComm,
        Safety: SafetySpec,
        *args,
        **kwargs,
    ) -> None:
        # super().__init__()
        ActuatorAction.__init__(self, device=self)
        self._Gains: ControlGains = Gains
        self._MecheConsts: MecheConsts = MecheSpecs
        self._Communication: ActuatorComm = Communication
        self._frequency = self._Communication.frequency
        self._mode: ActuatorMode = ActuatorMode(device=self)

        self._which_mode: str = self._mode._which_mode
        self._mode_pass: c_int
        self._safety: SafetySpec = Safety
        self._log: Logger = Logger()

        self._sensor: Any = None

        self._encoder_map = None

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0

        self._joint_offset = 0.0
        self._motor_offset = 0.0

        self._joint_direction = 1.0

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=80,
            soft_border_C_windings=10,
            temp_limit_case=70,
            soft_border_C_case=10,
        )
        self._thermal_scale: float = 1.0

    @abstractmethod
    def start(self) -> None:
        self.set_mode(which_mode="voltage", mode_pass=self._mode_pass)
        self.set_voltage(voltage_value=0, mode_pass=self._mode_pass)
        # try:
        #     self.open(
        #         freq=self._frequency,
        #         log_level=self._debug_level,
        #         log_enabled=self._dephy_log,
        #     )
        # except OSError as e:
        #     print("\n")
        #     self._log.error(
        #         msg=f"[{self.__repr__()}] Need admin previleges to open the port '{self.port}'. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
        #     )
        #     os._exit(status=1)
        # time.sleep(0.1)
        # # self._sensor = self.read()
        self._mode.enter()
        pass

    @abstractmethod
    def stop(self) -> None:
        self.set_mode(which_mode="voltage", mode_pass=self._mode_pass)
        self.set_voltage(voltage_value=0, mode_pass=self._mode_pass)

        time.sleep(0.1)
        # self.close()
        pass

    @abstractmethod
    def update(self) -> None:
        pass

    def set_mode(self, which_mode: str, mode_pass: c_int) -> None:
        if which_mode in ["voltage", "current", "position", "impedance"]:
            self._which_mode = which_mode
            self._mode.transition(which_mode, mode_pass)
        else:
            # Should raise an error here
            self._log.warning(msg=f"[{self.__repr__()}] Mode {which_mode} not found")
            return

    @property
    def frequency(self) -> int:
        return self._frequency

    @property
    def mode(self) -> ActuatorMode:
        return self._mode

    @property
    def encoder_map(self):
        """Polynomial coefficients defining the joint encoder map from counts to radians."""
        return self._encoder_map

    @property
    def motor_zero_position(self) -> float:
        """Motor encoder zero position in radians."""
        return self._motor_zero_position

    @property
    def joint_zero_position(self) -> float:
        """Joint encoder zero position in radians."""
        return self._joint_zero_position

    @property
    def joint_offset(self) -> float:
        """Joint encoder offset in radians."""
        return self._joint_offset

    @property
    def motor_offset(self) -> float:
        """Motor encoder offset in radians."""
        return self._motor_offset

    @property
    def joint_direction(self) -> float:
        """Joint direction: 1 or -1"""
        return self._joint_direction

    @property
    def battery_current(self) -> float:
        """Battery current in mA."""
        if self._sensor is not None:
            return float(self._sensor.batt_curr)
        else:
            return 0.0

    @property
    def motor_voltage(self) -> float:
        """Q-axis motor voltage in mV."""
        if self._sensor is not None:
            return float(self._sensor.mot_volt)
        else:
            return 0.0

    @property
    def motor_current(self) -> float:
        """Q-axis motor current in mA."""
        if self._sensor is not None:
            return float(self._sensor.mot_cur)
        else:
            return 0.0

    @property
    def motor_torque(self) -> float:
        """
        Torque at motor output in Nm.
        This is calculated using the motor current and torque constant.
        """
        if self._sensor is not None:
            return float(self._sensor.mot_cur * self._MecheConsts.NM_PER_MILLIAMP)
        else:
            return 0.0

    @property
    def motor_position(self) -> float:
        """Angle of the motor in radians."""
        if self._sensor is not None:
            return (
                float(self._sensor.mot_ang * self._MecheConsts.RAD_PER_COUNT)
                - self._motor_zero_position
                - self.motor_offset
            )
        else:
            return 0.0

    @property
    def motor_encoder_counts(self) -> int:
        """Raw reading from motor encoder in counts."""
        return int(self._sensor.mot_ang)

    @property
    def joint_encoder_counts(self) -> int:
        """Raw reading from joint encoder in counts."""
        return int(self._sensor.ank_ang)

    @property
    def motor_velocity(self) -> float:
        """Motor velocity in rad/s."""
        if self._sensor is not None:
            return int(self._sensor.mot_vel) * self._MecheConsts.RAD_PER_DEG
        else:
            return 0.0

    @property
    def motor_acceleration(self) -> float:
        """Motor acceleration in rad/s^2."""
        if self._sensor is not None:
            return float(self._sensor.mot_acc)
        else:
            return 0.0

    @property
    def joint_position(self) -> float:
        """Measured angle from the joint encoder in radians."""
        if self._sensor is not None:
            if self.encoder_map is not None:
                return float(self.encoder_map(self._sensor.ank_ang))
            else:
                return (
                    float(self._sensor.ank_ang * self._MecheConsts.RAD_PER_COUNT)
                    - self.joint_zero_position
                    - self.joint_offset
                ) * self.joint_direction
        else:
            return 0.0

    @property
    def joint_velocity(self) -> float:
        """Measured velocity from the joint encoder in rad/s."""
        if self._sensor is not None:
            return float(self._sensor.ank_vel * self._MecheConsts.RAD_PER_COUNT)
        else:
            return 0.0

    @property
    def case_temperature(self) -> float:
        """Case temperature in celsius."""
        if self._sensor is not None:
            return float(self._sensor.temperature)
        else:
            return 0.0

    @property
    def winding_temperature(self) -> float:
        """
        ESTIMATED temperature of the windings in celsius.
        This is calculated based on the thermal model using motor current.
        """
        if self._sensor is not None:
            return float(self._thermal_model.T_w)
        else:
            return 0.0

    @property
    def thermal_scaling_factor(self) -> float:
        """
        Scale factor to use in torque control, in [0,1].
        If you scale the torque command by this factor, the motor temperature will never exceed max allowable temperature.
        For a proof, see paper referenced in thermal model.
        """
        return float(self._thermal_scale)

    @property
    def genvars(self):
        """Dephy's 'genvars' object."""
        if self._sensor is not None:
            return np.array(
                object=[
                    self._sensor.genvar_0,
                    self._sensor.genvar_1,
                    self._sensor.genvar_2,
                    self._sensor.genvar_3,
                    self._sensor.genvar_4,
                    self._sensor.genvar_5,
                ]
            )
        else:
            return np.zeros(shape=6)

    @property
    def accelx(self) -> float:
        """
        Acceleration in x direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._sensor is not None:
            return float(
                self._sensor.accelx * self._MecheConsts.M_PER_SEC_SQUARED_ACCLSB
            )
        else:
            return 0.0

    @property
    def accely(self) -> float:
        """
        Acceleration in y direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._sensor is not None:
            return float(
                self._sensor.accely * self._MecheConsts.M_PER_SEC_SQUARED_ACCLSB
            )
        else:
            return 0.0

    @property
    def accelz(self) -> float:
        """
        Acceleration in z direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._sensor is not None:
            return float(
                self._sensor.accelz * self._MecheConsts.M_PER_SEC_SQUARED_ACCLSB
            )
        else:
            return 0.0

    @property
    def gyrox(self) -> float:
        """
        Angular velocity in x direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._sensor is not None:
            return float(self._sensor.gyrox * self._MecheConsts.RAD_PER_SEC_GYROLSB)
        else:
            return 0.0

    @property
    def gyroy(self) -> float:
        """
        Angular velocity in y direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._sensor is not None:
            return float(self._sensor.gyroy * self._MecheConsts.RAD_PER_SEC_GYROLSB)
        else:
            return 0.0

    @property
    def gyroz(self) -> float:
        """
        Angular velocity in z direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._sensor is not None:
            return float(self._sensor.gyroz * self._MecheConsts.RAD_PER_SEC_GYROLSB)
        else:
            return 0.0

    @abstractmethod
    def set_voltage(self, voltage_value: float, mode_pass: c_int):
        super()._set_voltage(self._which_mode, voltage_value=voltage_value)
        # transfer value here
        pass

    @abstractmethod
    def set_current(
        self, current_value: float, CurrentGain: ControlGains, mode_pass: c_int
    ):
        super()._set_current(self._which_mode, current_value, CurrentGain)
        self._set_gains(Gains=CurrentGain)
        # transfer value here
        pass

    @abstractmethod
    def set_impedance(
        self, PositionCount: int, ImpedanceGains: ControlGains, mode_pass: c_int
    ):
        super()._set_impedance(self._which_mode, ImpedanceGains=ImpedanceGains)
        self._set_gains(Gains=ImpedanceGains)
        # transfer value here
        pass

    @abstractmethod
    def set_position(
        self, PositionCount: int, PositionGains: ControlGains, mode_pass: c_int
    ):
        super()._set_position(self._which_mode, PositionGains=PositionGains)
        self._set_gains(Gains=PositionGains)
        # transfer value here
        pass

    def _set_gains(self, Gains: ControlGains) -> None:
        if self._which_mode == "current":
            self._Gains = ControlGains(
                kp=Gains.kp, ki=Gains.ki, kd=0, K=0, B=0, ff=Gains.ff
            )
        if self._which_mode == "impedance":
            self._Gains = ControlGains(
                kp=Gains.kp, ki=Gains.ki, kd=0, K=Gains.K, B=Gains.B, ff=Gains.ff
            )
        if self._which_mode == "position":
            self._Gains = ControlGains(
                kp=Gains.kp, ki=Gains.ki, kd=Gains.kd, K=0, B=0, ff=Gains.ff
            )
        else:
            # should raise an error here
            pass


class DephyActpack(Actuator, Device):
    def __init__(
        self,
        Gains: ControlGains,
        MecheSpecs: MecheConsts,
        Communication: ActuatorComm,
        Safety: SafetySpec,
        *args,
        **kwargs,
    ) -> None:
        Actuator.__init__(self, Gains, MecheSpecs, Communication, Safety)
        Device.__init__(
            self, port=Communication.port, baud_rate=Communication.baud_rate
        )

    @abstractmethod
    def start(self) -> None:
        super().start()
        self.open(
            freq=self._Communication.frequency,
            log_level=self._Communication.debug_level,
            log_enabled=self._Communication.log,
        )

    def stop(self) -> None:
        self.set_mode(which_mode="voltage", mode_pass=self._mode_pass)
        self.set_voltage(voltage_value=0, mode_pass=self._mode_pass)

        time.sleep(0.1)
        self.close()

    @abstractmethod
    def update(self) -> None:
        if self.is_streaming:
            self._sensor = self.read()
            self._thermal_model.T_c = self.case_temperature
            self._thermal_scale = self._thermal_model.update_and_get_scale(
                dt=(1 / self._frequency),
                motor_current=self.motor_current,
            )

            if hasattr(self, "_safety_attributes"):
                for safety_attribute_name in self._safety_attributes:
                    self._log.debug(
                        msg=f"[{self.__repr__()}] Safety mechanism in-place for {safety_attribute_name}: {getattr(self, safety_attribute_name)}"
                    )
        else:
            self._log.warning(
                msg=f"[{self.__repr__()}] Please open() the device before streaming data."
            )

    def set_voltage(self, voltage_value: float, mode_pass: c_int):
        super()._set_voltage(self._which_mode, voltage_value=voltage_value)
        # transfer value here
        self.send_motor_command(ctrl_mode=mode_pass, value=voltage_value)

    def set_current(
        self, current_value: float, CurrentGain: ControlGains, mode_pass: c_int
    ):
        super()._set_current(self._which_mode, current_value, CurrentGain)
        # transfer value here
        self.send_motor_command(ctrl_mode=mode_pass, value=current_value)

    def set_impedance(
        self, PositionCount: int, ImpedanceGains: ControlGains, mode_pass: c_int
    ):
        super()._set_impedance(self._which_mode, ImpedanceGains=ImpedanceGains)
        self.send_motor_command(ctrl_mode=mode_pass, value=PositionCount)
        # transfer value here
        pass

    def set_position(
        self, PositionCount: int, PositionGains: ControlGains, mode_pass: c_int
    ):
        super()._set_position(self._which_mode, PositionGains=PositionGains)
        self._set_gains(Gains=PositionGains)
        self.send_motor_command(ctrl_mode=mode_pass, value=PositionCount)
        # transfer value here
        pass


# MockDephyActpack class definition for testing
# MockData class definition for testing without a data stream
class MockData:
    def __init__(
        self,
        batt_volt=30,
        batt_curr=0,
        mot_volt=0,
        mot_cur=0,
        mot_ang=0,
        ank_ang=0,
        mot_vel=0,
        mot_acc=0,
        ank_vel=0,
        temperature=25,
        genvar_0=0,
        genvar_1=0,
        genvar_2=0,
        genvar_3=0,
        genvar_4=0,
        genvar_5=0,
        accelx=0,
        accely=0,
        accelz=0,
        gyrox=0,
        gyroy=0,
        gyroz=0,
    ):
        self.batt_volt = batt_volt
        self.batt_curr = batt_curr
        self.mot_volt = mot_volt
        self.mot_cur = mot_cur
        self.mot_ang = mot_ang
        self.ank_ang = ank_ang
        self.mot_vel = mot_vel
        self.mot_acc = mot_acc
        self.ank_vel = ank_vel
        self.temperature = temperature
        self.genvar_0 = genvar_0
        self.genvar_1 = genvar_1
        self.genvar_2 = genvar_2
        self.genvar_3 = genvar_3
        self.genvar_4 = genvar_4
        self.genvar_5 = genvar_5
        self.accelx = accelx
        self.accely = accely
        self.accelz = accelz
        self.gyrox = gyrox
        self.gyroy = gyroy
        self.gyroz = gyroz

    def __repr__(self):
        return f"MockData"


class MockDephyActpack(DephyActpack):
    def __init__(
        self,
        Gains: ControlGains,
        MecheSpecs: MecheConsts,
        Communication: ActuatorComm,
        Safety: SafetySpec,
        *args,
        **kwargs,
    ) -> None:
        Actuator.__init__(self, Gains, MecheSpecs, Communication, Safety)
        # Device.__init__(
        #     self, port=Communication.port, baud_rate=Communication.baud_rate
        # )
        self._sensor = MockData()

    # Overrides the open method to function without a device
    def open(self, freq, log_level, log_enabled):
        self._log.debug(msg=f"[{self.__repr__()}] Opening Device at {self.port}")
        self.is_streaming = True

    # Overrides the send_motor_command method to set the new _motor_command attribute
    def send_motor_command(self, ctrl_mode, value):
        self._motor_command = (
            f"[{self.__repr__()}] Control Mode: {ctrl_mode}, Value: {value}"
        )

    # Overrides the set_gains method to set the gains in the new _gains attribute
    def set_gains(self, kp, ki, kd, k, b, ff):
        self._Gains["kp"] = kp
        self._Gains["ki"] = ki
        self._Gains["kd"] = kd
        self._Gains["k"] = k
        self._Gains["b"] = b
        self._Gains["ff"] = ff

    # Overrides the read method to modify the data incrementally instead of through a device data stream
    def read(self):
        small_noise = np.random.normal(0, 0.01)

        self._sensor.batt_volt += small_noise
        self._sensor.batt_curr += 0.0
        self._sensor.mot_volt += 0.0
        self._sensor.mot_cur += 0.0
        self._sensor.mot_ang += 0.0
        self._sensor.ank_ang += 0.0
        self._sensor.mot_vel += small_noise
        self._sensor.mot_acc += small_noise
        self._sensor.ank_vel += small_noise
        self._sensor.temperature += small_noise
        self._sensor.genvar_0 += 0.0
        self._sensor.genvar_1 += 0.0
        self._sensor.genvar_2 += 0.0
        self._sensor.genvar_3 += 0.0
        self._sensor.genvar_4 += 0.0
        self._sensor.genvar_5 += 0.0
        self._sensor.accelx += small_noise
        self._sensor.accely += small_noise
        self._sensor.accelz += small_noise
        self._sensor.gyrox += small_noise
        self._sensor.gyroy += small_noise
        self._sensor.gyroz += small_noise
        return self._sensor

    # Overrides the close method to do nothing
    def close(self):
        pass


if __name__ == "__main__":
    pass
