import time
from dataclasses import dataclass

import flexsea.fx_enums as fxe
import numpy as np
from flexsea.device import Device

from opensourceleg.constants import Constants
from opensourceleg.control import (
    DEFAULT_CURRENT_GAINS,
    DEFAULT_IMPEDANCE_GAINS,
    DEFAULT_POSITION_GAINS,
)
from opensourceleg.logger import Logger
from opensourceleg.units import DEFAULT_UNITS, UnitsDefinition


@dataclass
class ControlModes:
    voltage = fxe.FX_VOLTAGE
    current = fxe.FX_CURRENT
    position = fxe.FX_POSITION
    impedance = fxe.FX_IMPEDANCE


CONTROL_MODE = ControlModes()


class ActpackMode:
    def __init__(self, control_mode, device: "DephyActpack"):
        self._control_mode = control_mode
        self._device = device
        self._entry_callback: callable = lambda: None
        self._exit_callback: callable = lambda: None

        self._has_gains = False

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, ActpackMode):
            return self._control_mode == __o._control_mode
        return False

    def __str__(self) -> str:
        return str(self._control_mode)

    @property
    def mode(self):
        return self._control_mode

    @property
    def has_gains(self) -> bool:
        return self._has_gains

    def enter(self):
        self._entry_callback()

    def exit(self):
        self._exit_callback()

    def transition(self, to_state: "ActpackMode"):
        self.exit()
        to_state.enter()


class VoltageMode(ActpackMode):
    def __init__(self, device: "DephyActpack"):
        super().__init__(CONTROL_MODE.voltage, device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        self._device._log.debug(f"[Actpack] Entering Voltage mode.")

    def _exit(self):
        self._device._log.debug(f"[Actpack] Exiting Voltage mode.")
        self._set_voltage(0)
        time.sleep(0.1)

    def _set_voltage(self, voltage: int):
        self._device.send_motor_command(
            self.mode,
            voltage,
        )


class CurrentMode(ActpackMode):
    def __init__(self, device: "DephyActpack"):
        super().__init__(CONTROL_MODE.current, device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        self._device._log.debug(f"[Actpack] Entering Current mode.")

        if not self.has_gains:
            self._set_gains()

        self._set_current(0)

    def _exit(self):
        self._device._log.debug(f"[Actpack] Exiting Current mode.")
        self._device.send_motor_command(CONTROL_MODE.voltage, 0)
        time.sleep(1 / self._device.frequency)

    def _set_gains(
        self,
        kp: int = DEFAULT_CURRENT_GAINS.kp,
        ki: int = DEFAULT_CURRENT_GAINS.ki,
        ff: int = DEFAULT_CURRENT_GAINS.ff,
    ):

        assert 0 <= kp <= 80, "kp must be between 0 and 80"
        assert 0 <= ki <= 800, "ki must be between 0 and 800"
        assert 0 <= ff <= 128, "ff must be between 0 and 128"

        self._device.set_gains(kp=kp, ki=ki, kd=0, k=0, b=0, ff=ff)
        self._has_gains = True

        time.sleep(0.1)

    def _set_current(self, current: int):
        """Sets the Q-axis current of the motor

        Args:
            current (int): _description_
        """
        self._device.send_motor_command(
            self.mode,
            current,
        )


class PositionMode(ActpackMode):
    def __init__(self, device: "DephyActpack"):
        super().__init__(CONTROL_MODE.position, device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        self._device._log.debug(f"[Actpack] Entering Position mode.")

        if not self.has_gains:
            self._set_gains()

        self._set_motor_position(
            int(
                self._device._units.convert_to_default_units(
                    self._device.motor_position, "position"
                )
                / Constants.RAD_PER_COUNT
            )
        )

    def _exit(self):
        self._device._log.debug(f"[Actpack] Exiting Position mode.")
        self._device.send_motor_command(CONTROL_MODE.voltage, 0)
        time.sleep(0.1)

    def _set_gains(
        self,
        kp: int = DEFAULT_POSITION_GAINS.kp,
        ki: int = DEFAULT_POSITION_GAINS.ki,
        kd: int = DEFAULT_POSITION_GAINS.kd,
    ):

        assert 0 <= kp <= 1000, "kp must be between 0 and 1000"
        assert 0 <= ki <= 1000, "ki must be between 0 and 1000"
        assert 0 <= kd <= 1000, "kd must be between 0 and 1000"

        self._device.set_gains(kp=kp, ki=ki, kd=kd, k=0, b=0, ff=0)
        self._has_gains = True

        time.sleep(1 / self._device.frequency)

    def _set_motor_position(self, counts: int):
        """Sets the motor position

        Args:
            counts (int): position in counts
        """
        self._device.send_motor_command(
            self.mode,
            counts,
        )


class ImpedanceMode(ActpackMode):
    def __init__(self, device: "DephyActpack"):
        super().__init__(CONTROL_MODE.impedance, device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        self._device._log.debug(f"[Actpack] Entering Impedance mode.")
        if not self.has_gains:
            self._set_gains()

        self._set_motor_position(
            int(
                self._device._units.convert_to_default_units(
                    self._device.motor_position, "position"
                )
                / Constants.RAD_PER_COUNT
            )
        )

    def _exit(self):
        self._device._log.debug(f"[Actpack] Exiting Impedance mode.")
        self._device.send_motor_command(CONTROL_MODE.voltage, 0)
        time.sleep(1 / self._device.frequency)

    def _set_motor_position(self, counts: int):
        """Sets the motor position

        Args:
            counts (int): position in counts
        """
        self._device.send_motor_command(
            self.mode,
            counts,
        )

    def _set_gains(
        self,
        kp: int = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: int = DEFAULT_IMPEDANCE_GAINS.ki,
        K: int = DEFAULT_IMPEDANCE_GAINS.K,
        B: int = DEFAULT_IMPEDANCE_GAINS.B,
        ff: int = DEFAULT_IMPEDANCE_GAINS.ff,
    ):

        assert 0 <= kp <= 80, "kp must be between 0 and 80"
        assert 0 <= ki <= 800, "ki must be between 0 and 800"
        assert 0 <= ff <= 128, "ff must be between 0 and 128"
        assert 0 <= K, "K must be greater than 0"
        assert 0 <= B, "B must be greater than 0"

        self._device.set_gains(kp=kp, ki=ki, kd=0, k=K, b=B, ff=ff)
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
        logger: Logger = None,
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
        super().__init__(port, baud_rate)
        self._debug_level = debug_level
        self._dephy_log = dephy_log
        self._frequency = frequency
        self._data: dict = None
        self._log = logger
        self._state = None
        self._units = units if units else DEFAULT_UNITS

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0

        self._modes: dict[str, ActpackMode] = {
            "voltage": VoltageMode(self),
            "position": PositionMode(self),
            "current": CurrentMode(self),
            "impedance": ImpedanceMode(self),
        }

        self._mode: ActpackMode = self._modes["voltage"]

    def start(self):
        self.open(self._frequency, self._debug_level, log_enabled=self._dephy_log)
        time.sleep(0.1)
        self._data = self.read()
        self._mode.enter()

    def stop(self):
        self.set_mode("voltage")
        self.set_voltage(0, force=True)

        time.sleep(0.1)
        self.close()

    def update(self):
        if self.is_streaming:
            self._data = self.read()
        else:
            self._log.warning(
                f"[Actpack] Please open() the device before streaming data."
            )

    def set_mode(self, mode: str):
        if mode in self._modes:
            self._mode.transition(self._modes[mode])
            self._mode = self._modes[mode]

        else:
            self._log.warning(f"Mode {mode} not found")
            return

    def set_motor_zero_position(self, position: float):
        self._motor_zero_position = position

    def set_joint_zero_position(self, position: float):
        self._joint_zero_position = position

    def set_position_gains(
        self,
        kp: int = DEFAULT_POSITION_GAINS.kp,
        ki: int = DEFAULT_POSITION_GAINS.ki,
        kd: int = DEFAULT_POSITION_GAINS.kd,
        force: bool = True,
    ):

        """
        Sets the position gains

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            kd (int): The derivative gain
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["position"]:
            self._log.warning(f"Cannot set position gains in mode {self._mode}")
            return

        self._mode._set_gains(kp=kp, ki=ki, kd=kd)

    def set_current_gains(
        self,
        kp: int = DEFAULT_CURRENT_GAINS.kp,
        ki: int = DEFAULT_CURRENT_GAINS.ki,
        ff: int = DEFAULT_CURRENT_GAINS.ff,
        force: bool = True,
    ):

        """
        Sets the current gains

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            ff (int): The feedforward gain
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["current"]:
            self._log.warning(f"Cannot set current gains in mode {self._mode}")
            return

        self._mode._set_gains(kp=kp, ki=ki, ff=ff)

    def set_impedance_gains(
        self,
        kp: int = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: int = DEFAULT_IMPEDANCE_GAINS.ki,
        K: int = DEFAULT_IMPEDANCE_GAINS.K,
        B: int = DEFAULT_IMPEDANCE_GAINS.B,
        ff: int = DEFAULT_IMPEDANCE_GAINS.ff,
        force: bool = True,
    ):
        """
        Sets the impedance gains

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            K (int): The spring constant
            B (int): The damping constant
            ff (int): The feedforward gain
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["impedance"]:
            self._log.warning(f"Cannot set impedance gains in mode {self._mode}")
            return

        self._mode._set_gains(kp=kp, ki=ki, K=K, B=B, ff=ff)

    def set_voltage(self, value: float, force: bool = False):
        """
        Sets the q axis voltage

        Args:
            value (float): The voltage to set
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["voltage"]:
            self._log.warning(f"Cannot set voltage in mode {self._mode}")
            return

        self._mode._set_voltage(
            int(self._units.convert_to_default_units(value, "voltage")),
        )

    def set_current(self, value: float, force: bool = False):
        """
        Sets the q axis current

        Args:
            value (float): The current to set
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["current"]:
            self._log.warning(f"Cannot set current in mode {self._mode}")
            return

        self._mode._set_current(
            int(self._units.convert_to_default_units(value, "current")),
        )

    def set_motor_torque(self, torque: float, force: bool = False):
        """
        Sets the motor torque

        Args:
            torque (float): The torque to set
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["current"]:
            self._log.warning(f"Cannot set motor_torque in mode {self._mode}")
            return

        self._mode._set_current(
            int(
                self._units.convert_to_default_units(torque, "torque")
                / Constants.NM_PER_MILLIAMP
            ),
        )

    def set_motor_position(self, position: float):
        """
        Sets the motor position

        Args:
            position (float): The position to set
        """
        if self._mode not in [self._modes["position"], self._modes["impedance"]]:
            self._log.warning(f"Cannot set motor position in mode {self._mode}")
            return

        self._mode._set_motor_position(
            int(
                self._units.convert_to_default_units(position, "position")
                / Constants.RAD_PER_COUNT
            ),
        )

    # Read only properties from the actpack

    @property
    def units(self):
        return self._units

    @property
    def frequency(self):
        return self._frequency

    @property
    def mode(self):
        return self._mode

    @property
    def motor_zero_position(self):
        return self._motor_zero_position

    @property
    def joint_zero_position(self):
        return self._joint_zero_position

    @property
    def battery_voltage(self):
        return self._units.convert_from_default_units(
            self._data.batt_volt,
            "voltage",
        )

    @property
    def batter_current(self):
        return self._units.convert_from_default_units(
            self._data.batt_curr,
            "current",
        )

    @property
    def motor_voltage(self):
        return self._units.convert_from_default_units(
            self._data.mot_volt,
            "voltage",
        )

    @property
    def motor_current(self):
        return self._units.convert_from_default_units(
            self._data.mot_cur,
            "current",
        )

    @property
    def motor_torque(self):
        return self._units.convert_from_default_units(
            self._data.mot_cur * Constants.NM_PER_MILLIAMP,
            "torque",
        )

    @property
    def motor_position(self):
        return self._units.convert_from_default_units(
            int(self._data.mot_ang) * Constants.RAD_PER_COUNT,
            "position",
        )

    @property
    def motor_velocity(self):
        return self._units.convert_from_default_units(
            self._data.mot_vel * Constants.RAD_PER_DEG,
            "velocity",
        )

    @property
    def motor_acceleration(self):
        return self._units.convert_from_default_units(
            self._data.mot_acc,
            "acceleration",
        )

    @property
    def motor_torque(self):
        return self._units.convert_from_default_units(
            self.motor_current * Constants.NM_PER_AMP,
            "torque",
        )

    @property
    def joint_position(self):
        return self._units.convert_from_default_units(
            self._data.ank_ang * Constants.RAD_PER_COUNT,
            "position",
        )

    @property
    def joint_velocity(self):
        return self._units.convert_from_default_units(
            self._data.ank_vel * Constants.RAD_PER_COUNT,
            "velocity",
        )

    @property
    def genvars(self):
        return np.array(
            [
                self._data.genvar_0,
                self._data.genvar_1,
                self._data.genvar_2,
                self._data.genvar_3,
                self._data.genvar_4,
                self._data.genvar_5,
            ]
        )

    @property
    def acc_x(self):
        return self._units.convert_from_default_units(
            self._data.accelx * Constants.M_PER_SEC_SQUARED_ACCLSB,
            "gravity",
        )

    @property
    def acc_y(self):
        return self._units.convert_from_default_units(
            self._data.accely * Constants.M_PER_SEC_SQUARED_ACCLSB,
            "gravity",
        )

    @property
    def acc_z(self):
        return self._units.convert_from_default_units(
            self._data.accelz * Constants.M_PER_SEC_SQUARED_ACCLSB,
            "gravity",
        )

    @property
    def gyro_x(self):
        return self._units.convert_from_default_units(
            self._data.gyrox * Constants.RAD_PER_SEC_GYROLSB,
            "velocity",
        )

    @property
    def gyro_y(self):
        return self._units.convert_from_default_units(
            self._data.gyroy * Constants.RAD_PER_SEC_GYROLSB,
            "velocity",
        )

    @property
    def gyro_z(self):
        return self._units.convert_from_default_units(
            self._data.gyroz * Constants.RAD_PER_SEC_GYROLSB,
            "velocity",
        )
