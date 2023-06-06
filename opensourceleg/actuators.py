from typing import Any, Callable

import os
import time
from ctypes import c_int
from dataclasses import dataclass

import flexsea.fx_enums as fxe
import numpy as np
from flexsea.device import Device

import opensourceleg.constants as constants
from opensourceleg.control import (
    DEFAULT_CURRENT_GAINS,
    DEFAULT_IMPEDANCE_GAINS,
    DEFAULT_POSITION_GAINS,
)
from opensourceleg.logger import Logger
from opensourceleg.thermal import ThermalModel
from opensourceleg.units import DEFAULT_UNITS, UnitsDefinition


@dataclass
class ControlModes:
    """
    Control modes for the Dephy Actpack.

    Available modes are Voltage, Current, Position, Impedance.
    """

    voltage: c_int = fxe.FX_VOLTAGE
    current: c_int = fxe.FX_CURRENT
    position: c_int = fxe.FX_POSITION
    impedance: c_int = fxe.FX_IMPEDANCE


CONTROL_MODE = ControlModes()


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

        time.sleep(0.1)

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
            counts=int(
                self._device._units.convert_to_default_units(
                    value=self._device.motor_position, attribute="position"
                )
                / constants.RAD_PER_COUNT
            )
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
    ) -> None:

        assert 0 <= kp <= 1000, "kp must be between 0 and 1000"
        assert 0 <= ki <= 1000, "ki must be between 0 and 1000"
        assert 0 <= kd <= 1000, "kd must be between 0 and 1000"

        self._device.set_gains(kp=kp, ki=ki, kd=kd, k=0, b=0, ff=0)
        self._has_gains = True

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
            counts=int(
                self._device._units.convert_to_default_units(
                    value=self._device.motor_position, attribute="position"
                )
                / constants.RAD_PER_COUNT
            )
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

        time.sleep(1 / self._device.frequency)


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
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        logger: Logger = Logger(),
        units: UnitsDefinition = DEFAULT_UNITS,
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:
        """
        Initializes the Actpack class

        Args:
            port (str): _description_
            baud_rate (int): _description_. Defaults to 230400.
            frequency (int): _description_. Defaults to 500.
            logger (Logger): _description_
            units (UnitsDefinition): _description_
            debug_level (int): _description_. Defaults to 0.
            dephy_log (bool): _description_. Defaults to False.
        """
        super().__init__(port=port, baud_rate=baud_rate)
        self._debug_level: int = debug_level
        self._dephy_log: bool = dephy_log
        self._frequency: int = frequency
        self._data: Any = None

        self._log: Logger = logger
        self._state = None
        self._units: UnitsDefinition = units if units else DEFAULT_UNITS

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=80,
            soft_border_C_windings=10,
            temp_limit_case=70,
            soft_border_C_case=10,
        )

        self._modes: dict[str, ActpackMode] = {
            "voltage": VoltageMode(device=self),
            "position": PositionMode(device=self),
            "current": CurrentMode(device=self),
            "impedance": ImpedanceMode(device=self),
        }

        self._mode: ActpackMode = self._modes["voltage"]

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
                msg=f"Need admin previleges to open the port '{self.port}'. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        time.sleep(0.1)
        self._data = self.read()
        self._mode.enter()

    def stop(self) -> None:
        self.set_mode(mode="voltage")
        self.set_voltage(value=0)

        time.sleep(0.1)
        self.close()

    def update(self) -> None:
        if self.is_streaming:
            self._data = self.read()
            self._thermal_model.T_c = self._units.convert_to_default_units(
                value=self.case_temperature, attribute="temperature"
            )
            self._thermal_model.update(
                dt=(1 / self._frequency), motor_current=self.motor_current
            )
        else:
            self._log.warning(
                msg=f"[Actpack] Please open() the device before streaming data."
            )

    def set_mode(self, mode: str) -> None:
        if mode in self._modes:
            self._mode.transition(to_state=self._modes[mode])
            self._mode = self._modes[mode]

        else:
            self._log.warning(msg=f"Mode {mode} not found")
            return

    def set_motor_zero_position(self, position: float) -> None:
        self._motor_zero_position: float = position

    def set_joint_zero_position(self, position: float) -> None:
        self._joint_zero_position: float = position

    def set_position_gains(
        self,
        kp: int = DEFAULT_POSITION_GAINS.kp,
        ki: int = DEFAULT_POSITION_GAINS.ki,
        kd: int = DEFAULT_POSITION_GAINS.kd,
    ) -> None:

        """
        Sets the position gains

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            kd (int): The derivative gain
        """
        if self._mode != self._modes["position"]:
            self._log.warning(msg=f"Cannot set position gains in mode {self._mode}")
            return

        self._mode.set_gains(kp=kp, ki=ki, kd=kd)  # type: ignore

    def set_current_gains(
        self,
        kp: int = DEFAULT_CURRENT_GAINS.kp,
        ki: int = DEFAULT_CURRENT_GAINS.ki,
        ff: int = DEFAULT_CURRENT_GAINS.ff,
    ) -> None:

        """
        Sets the current gains

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            ff (int): The feedforward gain
        """
        if self._mode != self._modes["current"]:
            self._log.warning(f"Cannot set current gains in mode {self._mode}")
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
        Sets the impedance gains

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            K (int): The spring constant
            B (int): The damping constant
            ff (int): The feedforward gain
        """
        if self._mode != self._modes["impedance"]:
            self._log.warning(msg=f"Cannot set impedance gains in mode {self._mode}")
            return

        self._mode._set_gains(kp=kp, ki=ki, K=K, B=B, ff=ff)  # type: ignore

    def set_voltage(self, value: float) -> None:
        """
        Sets the q axis voltage

        Args:
            value (float): The voltage to set
        """
        if self._mode != self._modes["voltage"]:
            self._log.warning(msg=f"Cannot set voltage in mode {self._mode}")
            return

        self._mode._set_voltage(  # type: ignore
            int(self._units.convert_to_default_units(value=value, attribute="voltage")),
        )

    def set_current(self, value: float) -> None:
        """
        Sets the q axis current

        Args:
            value (float): The current to set
        """
        if self._mode != self._modes["current"]:
            self._log.warning(msg=f"Cannot set current in mode {self._mode}")
            return

        self._mode._set_current(  # type: ignore
            int(self._units.convert_to_default_units(value=value, attribute="current")),
        )

    def set_motor_torque(self, torque: float) -> None:
        """
        Sets the motor torque

        Args:
            torque (float): The torque to set
        """
        if self._mode != self._modes["current"]:
            self._log.warning(msg=f"Cannot set motor_torque in mode {self._mode}")
            return

        self._mode._set_current(  # type: ignore
            int(
                self._units.convert_to_default_units(value=torque, attribute="torque")
                / constants.NM_PER_MILLIAMP
            ),
        )

    def set_motor_position(self, position: float) -> None:
        """
        Sets the motor position

        Args:
            position (float): The position to set
        """
        if self._mode not in [self._modes["position"], self._modes["impedance"]]:
            self._log.warning(msg=f"Cannot set motor position in mode {self._mode}")
            return

        self._mode._set_motor_position(  # type: ignore
            int(
                (
                    self._units.convert_to_default_units(
                        value=position, attribute="position"
                    )
                    / constants.RAD_PER_COUNT
                )
                + self.motor_zero_position
            ),
        )

    # Read only properties from the actpack

    @property
    def units(self) -> UnitsDefinition:
        return self._units

    @property
    def frequency(self) -> int:
        return self._frequency

    @property
    def mode(self) -> ActpackMode:
        return self._mode

    @property
    def modes(self) -> dict[str, ActpackMode]:
        return self._modes

    @property
    def motor_zero_position(self) -> float:
        return self._motor_zero_position

    @property
    def joint_zero_position(self) -> float:
        return self._joint_zero_position

    @property
    def battery_voltage(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.batt_volt,
                attribute="voltage",
            )
        else:
            return 0.0

    @property
    def batter_current(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.batt_curr,
                attribute="current",
            )
        else:
            return 0.0

    @property
    def motor_voltage(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.mot_volt,
                attribute="voltage",
            )
        else:
            return 0.0

    @property
    def motor_current(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.mot_cur,
                attribute="current",
            )
        else:
            return 0.0

    @property
    def motor_torque(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.mot_cur * constants.NM_PER_MILLIAMP,
                attribute="torque",
            )
        else:
            return 0.0

    @property
    def motor_position(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=int(self._data.mot_ang - self.motor_zero_position)
                * constants.RAD_PER_COUNT,
                attribute="position",
            )
        else:
            return 0.0

    @property
    def motor_encoder_counts(self):
        return self._data.mot_ang

    @property
    def joint_encoder_counts(self):
        return self._data.ank_ang

    @property
    def motor_velocity(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=int(self._data.mot_vel) * constants.RAD_PER_COUNT,
                attribute="velocity",
            )
        else:
            return 0.0

    @property
    def motor_acceleration(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.mot_acc,
                attribute="acceleration",
            )
        else:
            return 0.0

    @property
    def joint_position(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=int(self._data.ank_ang - self._joint_zero_position)
                * constants.RAD_PER_COUNT,
                attribute="position",
            )
        else:
            return 0.0

    @property
    def joint_velocity(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.ank_vel * constants.RAD_PER_COUNT,
                attribute="velocity",
            )
        else:
            return 0.0

    @property
    def case_temperature(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.temperature,
                attribute="temperature",
            )
        else:
            return 0.0

    @property
    def winding_temperature(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._thermal_model.T_w,
                attribute="temperature",
            )
        else:
            return 0.0

    @property
    def genvars(self):
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
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.accelx * constants.M_PER_SEC_SQUARED_ACCLSB,
                attribute="gravity",
            )
        else:
            return 0.0

    @property
    def accely(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.accely * constants.M_PER_SEC_SQUARED_ACCLSB,
                attribute="gravity",
            )
        else:
            return 0.0

    @property
    def accelz(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.accelz * constants.M_PER_SEC_SQUARED_ACCLSB,
                attribute="gravity",
            )
        else:
            return 0.0

    @property
    def gyrox(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.gyrox * constants.RAD_PER_SEC_GYROLSB,
                attribute="velocity",
            )
        else:
            return 0.0

    @property
    def gyroy(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.gyroy * constants.RAD_PER_SEC_GYROLSB,
                attribute="velocity",
            )
        else:
            return 0.0

    @property
    def gyroz(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.gyroz * constants.RAD_PER_SEC_GYROLSB,
                attribute="velocity",
            )
        else:
            return 0.0
