from typing import Any, Callable, Union, overload

import ctypes
import os
import time
from ctypes import c_int
from dataclasses import dataclass

import flexsea.fx_enums as fxe
import numpy as np
from flexsea.device import Device

from ..tools.logger import Logger
from .thermal import ThermalModel

"""
Module Overview:

This module defines classes related to controlling the Dephy Actpack, including control modes,
gains, and Actpack modes. It also provides a class for the Dephy Actpack itself.

Key Classes:

- `ControlModes`: Enumerates available control modes for the Dephy Actpack.
- `Gains`: Dataclass for controller gains.
- `ActpackMode`: Base class for Actpack modes, including `VoltageMode`, `CurrentMode`, `PositionMode`, and `ImpedanceMode`.
- `ActpackControlModes`: Enumerates available Actpack modes.
- `DephyActpack`: Class for interacting with the Dephy Actpack.

Usage Guide:

1. Create an instance of `DephyActpack` with appropriate parameters (e.g., port, baud_rate, frequency).
2. Start the actpack using the `start` method.
3. Set the desired control mode using the `set_mode` method.
4. Set gains for the selected control mode using methods like `set_position_gains`, `set_current_gains`, etc.
5. Optionally, update the actpack using the `update` method to query the latest values.
6. Stop the actpack using the `stop` method.

"""


@dataclass
class ControlModes:
    """
    Control modes for the Dephy Actpack.

    Available modes are Voltage, Current, Position, Impedance.
    """

    voltage: ctypes.c_int = fxe.FX_VOLTAGE
    current: ctypes.c_int = fxe.FX_CURRENT
    position: ctypes.c_int = fxe.FX_POSITION
    impedance: ctypes.c_int = fxe.FX_IMPEDANCE

    def __repr__(self) -> str:
        return f"ControlModes"


@dataclass
class Gains:
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


CONTROL_MODE = ControlModes()

MOTOR_COUNT_PER_REV: float = 16384
NM_PER_AMP: float = 0.1133
NM_PER_MILLIAMP: float = NM_PER_AMP / 1000
RAD_PER_COUNT: float = 2 * np.pi / MOTOR_COUNT_PER_REV
RAD_PER_DEG: float = np.pi / 180

RAD_PER_SEC_GYROLSB: float = np.pi / 180 / 32.8
M_PER_SEC_SQUARED_ACCLSB: float = 9.80665 / 8192

IMPEDANCE_A: float = 0.00028444
IMPEDANCE_C: float = 0.0007812

NM_PER_RAD_TO_K: float = RAD_PER_COUNT / IMPEDANCE_C * 1e3 / NM_PER_AMP
NM_S_PER_RAD_TO_B: float = RAD_PER_DEG / IMPEDANCE_A * 1e3 / NM_PER_AMP

MAX_CASE_TEMPERATURE: float = 80

DEFAULT_POSITION_GAINS = Gains(kp=50, ki=0, kd=0, K=0, B=0, ff=0)

DEFAULT_CURRENT_GAINS = Gains(kp=40, ki=400, kd=0, K=0, B=0, ff=128)

DEFAULT_IMPEDANCE_GAINS = Gains(kp=40, ki=400, kd=0, K=200, B=400, ff=128)


class ActpackMode:
    """
    Base class for Actpack modes

    Args:
        control_mode (c_int): Control mode
        device (DephyActpack): Dephy Actpack
    """

    def __init__(self, control_mode: c_int, device: "DephyActpack") -> None:

        self._control_mode: c_int = control_mode
        self._device: DephyActpack = device
        self._entry_callback: Callable[[], None] = lambda: None
        self._exit_callback: Callable[[], None] = lambda: None

        self._has_gains = False

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, ActpackMode):
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

    @property
    def has_gains(self) -> bool:
        """
        Whether the mode has gains

        Returns:
            bool: True if the mode has gains, False otherwise
        """
        return self._has_gains

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

    def transition(self, to_state: "ActpackMode") -> None:
        """
        Transition to another mode. Calls the exit callback of the current mode
        and the entry callback of the new mode.

        Args:
            to_state (ActpackMode): Mode to transition to
        """
        self.exit()
        to_state.enter()

    def _set_voltage(self, voltage: int) -> None:
        """
        This method should be implemented by the child class. It should set the q axis voltage.
        """
        pass

    def _set_current(self, current: int) -> None:
        """
        This method should be implemented by the child class. It should set the q axis current.
        """
        pass

    def _set_motor_position(self, counts: int) -> None:
        """
        This method should be implemented by the child class. It should set the motor position.
        """
        pass


class VoltageMode(ActpackMode):
    def __init__(self, device: "DephyActpack") -> None:
        super().__init__(control_mode=CONTROL_MODE.voltage, device=device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self) -> None:
        self._device._log.debug(msg=f"[Actpack] Entering Voltage mode.")

    def _exit(self) -> None:
        self._device._log.debug(msg=f"[Actpack] Exiting Voltage mode.")
        self._set_voltage(voltage=0)
        time.sleep(0.1)

    def _set_voltage(self, voltage: int) -> None:
        self._device.send_motor_command(
            ctrl_mode=self.mode,
            value=voltage,
        )


class CurrentMode(ActpackMode):
    def __init__(self, device: "DephyActpack") -> None:
        super().__init__(control_mode=CONTROL_MODE.current, device=device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self) -> None:
        self._device._log.debug(msg=f"[Actpack] Entering Current mode.")

        if not self.has_gains:
            self._set_gains()

        self._set_current(current=0)

    def _exit(self) -> None:
        self._device._log.debug(msg=f"[Actpack] Exiting Current mode.")
        self._device.send_motor_command(ctrl_mode=CONTROL_MODE.voltage, value=0)
        time.sleep(1 / self._device.frequency)

    def _set_gains(
        self,
        kp: int = DEFAULT_CURRENT_GAINS.kp,
        ki: int = DEFAULT_CURRENT_GAINS.ki,
        ff: int = DEFAULT_CURRENT_GAINS.ff,
    ) -> None:

        assert 0 <= kp <= 80, "kp must be between 0 and 80"
        assert 0 <= ki <= 800, "ki must be between 0 and 800"
        assert 0 <= ff <= 128, "ff must be between 0 and 128"

        self._device.set_gains(kp=kp, ki=ki, kd=0, k=0, b=0, ff=ff)
        self._has_gains = True

    def _set_current(self, current: int) -> None:
        """Sets the Q-axis current of the motor

        Args:
            current (int): _description_
        """
        self._device.send_motor_command(
            ctrl_mode=self.mode,
            value=current,
        )


class PositionMode(ActpackMode):
    def __init__(self, device: "DephyActpack") -> None:
        super().__init__(control_mode=CONTROL_MODE.position, device=device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self) -> None:
        self._device._log.debug(msg=f"[Actpack] Entering Position mode.")

        if not self.has_gains:
            self._set_gains()

        self._set_motor_position(
            counts=int(self._device.motor_position / RAD_PER_COUNT)
        )

    def _exit(self) -> None:
        self._device._log.debug(msg=f"[Actpack] Exiting Position mode.")
        self._device.send_motor_command(ctrl_mode=CONTROL_MODE.voltage, value=0)
        time.sleep(0.1)

    def _set_gains(
        self,
        kp: int = DEFAULT_POSITION_GAINS.kp,
        ki: int = DEFAULT_POSITION_GAINS.ki,
        kd: int = DEFAULT_POSITION_GAINS.kd,
        ff: int = DEFAULT_POSITION_GAINS.ff,
    ) -> None:

        assert 0 <= kp <= 1000, "kp must be between 0 and 1000"
        assert 0 <= ki <= 1000, "ki must be between 0 and 1000"
        assert 0 <= kd <= 1000, "kd must be between 0 and 1000"

        self._device.set_gains(kp=kp, ki=ki, kd=kd, k=0, b=0, ff=ff)
        self._has_gains = True

    def _set_motor_position(self, counts: int) -> None:
        """Sets the motor position

        Args:
            counts (int): position in counts
        """
        self._device.send_motor_command(
            ctrl_mode=self.mode,
            value=counts,
        )


class ImpedanceMode(ActpackMode):
    def __init__(self, device: "DephyActpack") -> None:
        super().__init__(control_mode=CONTROL_MODE.impedance, device=device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self) -> None:
        self._device._log.debug(msg=f"[Actpack] Entering Impedance mode.")
        if not self.has_gains:
            self._set_gains()

        self._set_motor_position(
            counts=int(self._device.motor_position / RAD_PER_COUNT)
        )

    def _exit(self) -> None:
        self._device._log.debug(msg=f"[Actpack] Exiting Impedance mode.")
        self._device.send_motor_command(ctrl_mode=CONTROL_MODE.voltage, value=0)
        time.sleep(1 / self._device.frequency)

    def _set_motor_position(self, counts: int) -> None:
        """Sets the motor position

        Args:
            counts (int): position in counts
        """
        self._device.send_motor_command(
            ctrl_mode=self.mode,
            value=counts,
        )

    def _set_gains(
        self,
        kp: int = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: int = DEFAULT_IMPEDANCE_GAINS.ki,
        K: int = DEFAULT_IMPEDANCE_GAINS.K,
        B: int = DEFAULT_IMPEDANCE_GAINS.B,
        ff: int = DEFAULT_IMPEDANCE_GAINS.ff,
    ) -> None:

        assert 0 <= kp <= 80, "kp must be between 0 and 80"
        assert 0 <= ki <= 800, "ki must be between 0 and 800"
        assert 0 <= ff <= 128, "ff must be between 0 and 128"
        assert 0 <= K, "K must be greater than 0"
        assert 0 <= B, "B must be greater than 0"

        self._device.set_gains(
            kp=int(kp), ki=int(ki), kd=int(0), k=int(K), b=int(B), ff=int(ff)
        )
        self._has_gains = True


@dataclass(init=False)
class ActpackControlModes:
    """
    Actpack modes

    Args:
        voltage (VoltageMode): Voltage mode
        current (CurrentMode): Current mode
        position (PositionMode): Position mode
        impedance (ImpedanceMode): Impedance mode
    """

    voltage: VoltageMode
    current: CurrentMode
    position: PositionMode
    impedance: ImpedanceMode

    def __init__(self, device: "DephyActpack") -> None:
        self.voltage = VoltageMode(device=device)
        self.current = CurrentMode(device=device)
        self.position = PositionMode(device=device)
        self.impedance = ImpedanceMode(device=device)

    def __repr__(self) -> str:
        return f"ActpackControlModes"


class DephyActpack(Device):
    """Class for the Dephy Actpack

    Args:
        Device (_type_): _description_

    Raises:
        KeyError: _description_
        ValueError: _description_
        KeyError: _description_

    Returns:
        _type_: _description_
    """

    def __init__(
        self,
        name: str = "DephyActpack",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        logger: Logger = Logger(),
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:
        """
        Initializes the Actpack class

        Args:
            name (str): _description_. Defaults to "DephyActpack".
            port (str): _description_
            baud_rate (int): _description_. Defaults to 230400.
            frequency (int): _description_. Defaults to 500.
            logger (Logger): _description_
            debug_level (int): _description_. Defaults to 0.
            dephy_log (bool): _description_. Defaults to False.
        """
        super().__init__(port=port, baud_rate=baud_rate)
        self._debug_level: int = debug_level
        self._dephy_log: bool = dephy_log
        self._frequency: int = frequency
        self._data: Any = None
        self._name: str = name

        self._log: Logger = logger
        self._state = None

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=80,
            soft_border_C_windings=10,
            temp_limit_case=70,
            soft_border_C_case=10,
        )
        self._thermal_scale: float = 1.0

        self.control_modes: ActpackControlModes = ActpackControlModes(device=self)

        self._mode: ActpackMode = self.control_modes.voltage

    def __repr__(self) -> str:
        return f"DephyActpack[{self._name}]"

    def start(self) -> None:
        try:
            self.open(
                freq=self._frequency,
                log_level=self._debug_level,
                log_enabled=self._dephy_log,
            )
        except OSError as e:
            print("\n")
            self._log.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port '{self.port}'. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        time.sleep(0.1)
        self._data = self.read()
        self._mode.enter()

    def stop(self) -> None:
        self.set_mode(mode=self.control_modes.voltage)
        self.set_voltage(value=0)

        time.sleep(0.1)
        self.close()

    def update(self) -> None:
        """
        Queries the latest values from the actpack.
        Also updates thermal model.
        """
        if self.is_streaming:
            self._data = self.read()
            self._thermal_model.T_c = self.case_temperature
            self._thermal_scale = self._thermal_model.update_and_get_scale(
                dt=(1 / self._frequency),
                motor_current=self.motor_current,
            )
        else:
            self._log.warning(
                msg=f"[{self.__repr__()}] Please open() the device before streaming data."
            )

    def set_mode(self, mode: ActpackMode) -> None:
        if type(mode) in [VoltageMode, CurrentMode, PositionMode, ImpedanceMode]:
            self._mode.transition(to_state=mode)
            self._mode = mode

        else:
            self._log.warning(msg=f"[{self.__repr__()}] Mode {mode} not found")
            return

    def set_motor_zero_position(self, position: float) -> None:
        """Sets motor zero position in counts"""
        self._motor_zero_position = position

    def set_joint_zero_position(self, position: float) -> None:
        """Sets joint zero position in counts"""
        self._joint_zero_position = position

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
        if self._mode != self.control_modes.position:
            self._log.warning(
                msg=f"[{self.__repr__()}] Cannot set position gains in mode {self._mode}"
            )
            return

        self._mode._set_gains(kp=kp, ki=ki, kd=kd, ff=ff)  # type: ignore

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
        if self._mode != self.control_modes.current:
            self._log.warning(
                f"[{self.__repr__()}] Cannot set current gains in mode {self._mode}"
            )
            return

        self._mode._set_gains(kp=kp, ki=ki, ff=ff)  # type: ignore

    def set_impedance_gains(
        self,
        kp: int = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: int = DEFAULT_IMPEDANCE_GAINS.ki,
        K: int = DEFAULT_IMPEDANCE_GAINS.K,
        B: int = DEFAULT_IMPEDANCE_GAINS.B,
        ff: int = DEFAULT_IMPEDANCE_GAINS.ff,
    ) -> None:
        """
        Sets the impedance gains in arbitrary actpack units.
        See Dephy's webpage for conversions or use other library methods that handle conversion for you.

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            K (int): The spring constant
            B (int): The damping constant
            ff (int): The feedforward gain
        """
        if self._mode != self.control_modes.impedance:
            self._log.warning(
                msg=f"[{self.__repr__()}] Cannot set impedance gains in mode {self._mode}"
            )
            return

        self._mode._set_gains(kp=kp, ki=ki, K=K, B=B, ff=ff)  # type: ignore

    def set_voltage(self, value: float) -> None:
        """
        Sets the q axis voltage in mV

        Args:
            value (float): The voltage to set in mv
        """
        if self._mode != self.control_modes.voltage:
            self._log.warning(
                msg=f"[{self.__repr__()}] Cannot set voltage in mode {self._mode}"
            )
            return

        self._mode._set_voltage(
            int(value),
        )

    def set_current(self, value: float) -> None:
        """
        Sets the q axis current in mA

        Args:
            value (float): The current to set in mA
        """
        if self._mode != self.control_modes.current:
            self._log.warning(
                msg=f"[{self.__repr__()}] Cannot set current in mode {self._mode}"
            )
            return

        self._mode._set_current(
            int(value),
        )

    def set_motor_torque(self, torque: float) -> None:
        """
        Sets the motor torque in Nm.

        Args:
            torque (float): The torque to set in Nm.
        """
        if self._mode != self.control_modes.current:
            self._log.warning(
                msg=f"[{self.__repr__()}] Cannot set motor_torque in mode {self._mode}"
            )
            return

        self._mode._set_current(
            int(torque / NM_PER_MILLIAMP),
        )

    def set_motor_position(self, position: float) -> None:
        """
        Sets the motor position in radians.
        If in impedance mode, this sets the equilibrium angle in radians.

        Args:
            position (float): The position to set
        """
        if self._mode not in [
            self.control_modes.position,
            self.control_modes.impedance,
        ]:
            self._log.warning(
                msg=f"[{self.__repr__()}] Cannot set motor position in mode {self._mode}"
            )
            return

        self._mode._set_motor_position(
            int((position / RAD_PER_COUNT) + self.motor_zero_position),
        )

    @property
    def frequency(self) -> int:
        return self._frequency

    @property
    def mode(self) -> ActpackMode:
        return self._mode

    @property
    def motor_zero_position(self) -> float:
        """Motor encoder offset in radians."""
        return self._motor_zero_position

    @property
    def joint_zero_position(self) -> float:
        """Joint encoder offset in radians."""
        return self._joint_zero_position

    @property
    def battery_voltage(self) -> float:
        """Battery voltage in mV."""
        if self._data is not None:
            return float(self._data.batt_volt)
        else:
            return 0.0

    @property
    def battery_current(self) -> float:
        """Battery current in mA."""
        if self._data is not None:
            return float(self._data.batt_curr)
        else:
            return 0.0

    @property
    def motor_voltage(self) -> float:
        """Q-axis motor voltage in mV."""
        if self._data is not None:
            return float(self._data.mot_volt)
        else:
            return 0.0

    @property
    def motor_current(self) -> float:
        """Q-axis motor current in mA."""
        if self._data is not None:
            return float(self._data.mot_cur)
        else:
            return 0.0

    @property
    def motor_torque(self) -> float:
        """
        Torque at motor output in Nm.
        This is calculated using the motor current and torque constant.
        """
        if self._data is not None:
            return float(self._data.mot_cur * NM_PER_MILLIAMP)
        else:
            return 0.0

    @property
    def motor_position(self) -> float:
        """Angle of the motor in radians."""
        if self._data is not None:
            return float(
                int(self._data.mot_ang - self.motor_zero_position) * RAD_PER_COUNT
            )
        else:
            return 0.0

    @property
    def motor_encoder_counts(self) -> int:
        """Raw reading from motor encoder in counts."""
        return int(self._data.mot_ang)

    @property
    def joint_encoder_counts(self) -> int:
        """Raw reading from joint encoder in counts."""
        return int(self._data.ank_ang)

    @property
    def motor_velocity(self) -> float:
        """Motor velocity in rad/s."""
        if self._data is not None:
            return int(self._data.mot_vel) * RAD_PER_DEG
        else:
            return 0.0

    @property
    def motor_acceleration(self) -> float:
        """Motor acceleration in rad/s^2."""
        if self._data is not None:
            return float(self._data.mot_acc)
        else:
            return 0.0

    @property
    def joint_position(self) -> float:
        """Measured angle from the joint encoder in radians."""
        if self._data is not None:
            return float(
                int(self._data.ank_ang - self._joint_zero_position) * RAD_PER_COUNT
            )
        else:
            return 0.0

    @property
    def joint_velocity(self) -> float:
        """Measured velocity from the joint encoder in rad/s."""
        if self._data is not None:
            return float(self._data.ank_vel * RAD_PER_COUNT)
        else:
            return 0.0

    @property
    def case_temperature(self) -> float:
        """Case temperature in celsius."""
        if self._data is not None:
            return float(self._data.temperature)
        else:
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
            return 0.0


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


# This class inherits everything from the DephyActpack class but deletes the super().__init__() call in the constructor so the constructor does not try to connect to a device. It also overrides some of the methods.
class MockDephyActpack(DephyActpack):
    """
    MockDephyActpack class definition for testing.\n
    This class inherits everything from the DephyActpack class but
    deletes the super().__init__() call in the constructor so the
    constructor does not try to connect to a device. It also overrides
    some of the methods to allow for testing without a device, and adds
    attributes used to determine if the methods were called properly.
    """

    def __init__(
        self,
        name: str = "MockDephyActpack",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        logger: Logger = Logger(),
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:
        """
        Initializes the MockDephyActpack class

        Args:
            name (str): _description_. Defaults to "MockDephyActpack".
            port (str): _description_
            baud_rate (int): _description_. Defaults to 230400.
            frequency (int): _description_. Defaults to 500.
            logger (Logger): _description_
            debug_level (int): _description_. Defaults to 0.
            dephy_log (bool): _description_. Defaults to False.
        """
        self._debug_level: int = debug_level
        self._dephy_log: bool = dephy_log
        self._frequency: int = frequency
        self._data: MockData = MockData()
        self._name: str = name

        self._log: Logger = logger
        self._state = None

        # New attributes to be used for testing

        # This is used in the open() method to display the port the device should be connected to
        self.port: str = port

        # This is used in the send_motor_command() method to display the motor command that was sent
        self._motor_command: str = "None"

        # This is used in the set_gains() method to display the gains that were set
        self._gains: dict[str, float] = {
            "kp": 0,
            "ki": 0,
            "kd": 0,
            "k": 0,
            "b": 0,
            "ff": 0,
        }

        # This is used in the read() method to indicate a data stream
        self.is_streaming: bool = False

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=80,
            soft_border_C_windings=10,
            temp_limit_case=70,
            soft_border_C_case=10,
        )

        self.control_modes: ActpackControlModes = ActpackControlModes(device=self)
        self._mode: ActpackMode = self.control_modes.voltage

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
        self._gains["kp"] = kp
        self._gains["ki"] = ki
        self._gains["kd"] = kd
        self._gains["k"] = k
        self._gains["b"] = b
        self._gains["ff"] = ff

    # Overrides the read method to modify the data incrementally instead of through a device data stream
    def read(self):
        small_noise = np.random.normal(0, 0.01)

        self._data.batt_volt += small_noise
        self._data.batt_curr += 0.0
        self._data.mot_volt += 0.0
        self._data.mot_cur += 0.0
        self._data.mot_ang += 0.0
        self._data.ank_ang += 0.0
        self._data.mot_vel += small_noise
        self._data.mot_acc += small_noise
        self._data.ank_vel += small_noise
        self._data.temperature += small_noise
        self._data.genvar_0 += 0.0
        self._data.genvar_1 += 0.0
        self._data.genvar_2 += 0.0
        self._data.genvar_3 += 0.0
        self._data.genvar_4 += 0.0
        self._data.genvar_5 += 0.0
        self._data.accelx += small_noise
        self._data.accely += small_noise
        self._data.accelz += small_noise
        self._data.gyrox += small_noise
        self._data.gyroy += small_noise
        self._data.gyroz += small_noise
        return self._data

    # Overrides the close method to do nothing
    def close(self):
        pass


if __name__ == "__main__":
    pass
