from typing import Any, Callable, Union, overload

import ctypes
import os
import time
from ctypes import c_int
from dataclasses import dataclass
from unittest.mock import Mock

import flexsea.fx_enums as fxe
import numpy as np
from flexsea.device import Device

from opensourceleg.hardware.actuators.base import (
    ActuatorBase,
    ControlGains,
    ControlModeBase,
    ControlModesBase,
    ControlModesMapping,
    MotorConstants,
)
from opensourceleg.hardware.thermal import ThermalModel
from opensourceleg.tools.logger import LOGGER

DEFAULT_POSITION_GAINS = ControlGains(kp=50, ki=0, kd=0, K=0, B=0, ff=0)

DEFAULT_CURRENT_GAINS = ControlGains(kp=40, ki=400, kd=0, K=0, B=0, ff=128)

DEFAULT_IMPEDANCE_GAINS = ControlGains(kp=40, ki=400, kd=0, K=200, B=400, ff=128)

DEPHY_SLEEP_DURATION = 0.1


class DephyVoltageMode(ControlModeBase):
    def __init__(self, actuator: Union["DephyActpack", None] = None) -> None:
        super().__init__(
            control_mode_index=ControlModesMapping.VOLTAGE,
            control_mode_name=str(ControlModesMapping.VOLTAGE),
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
        )

    def __repr__(self) -> str:
        return f"DephyControlMode[{self.name}]"

    def _entry(self) -> None:
        # TODO: check log instance
        LOGGER.debug(msg=f"[DephyControlMode] Entering {self.name} control mode.")

    def _exit(self) -> None:
        # TODO: check log instance
        LOGGER.debug(msg=f"[DephyControlMode] Exiting {self.name} control mode.")
        self.set_command(value=0)
        time.sleep(DEPHY_SLEEP_DURATION)

    def set_gains(self, gains: ControlGains) -> None:
        LOGGER.info(
            msg=f"[{self._actuator.__repr__()}] {self.name} mode does not have gains."
        )

    def set_command(self, value: Union[float, int]) -> None:
        return super().set_command(value)


class DephyCurrentMode(ControlModeBase):
    def __init__(self, actuator: Union["DephyActpack", None] = None) -> None:
        super().__init__(
            control_mode_index=ControlModesMapping.CURRENT,
            control_mode_name=str(ControlModesMapping.CURRENT),
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
            max_gains=ControlGains(kp=80, ki=800, kd=0, K=0, B=0, ff=128),
        )

    def __repr__(self) -> str:
        return f"DephyControlMode[{self.name}]"

    def _entry(self) -> None:

        # TODO: check log instance
        LOGGER.debug(msg=f"[DephyControlMode] Entering {self.name} mode.")

        if not self.has_gains:
            self.set_gains()

        self.set_command(value=0)

    def _exit(self) -> None:
        # TODO: check log instance
        LOGGER.debug(msg=f"[DephyControlMode] Exiting {self.name} mode.")

        # Is this necessary? This was a required step for older flexsea but not sure if it is needed anymore
        self._actuator.send_motor_command(
            ctrl_mode=ControlModesMapping.VOLTAGE, value=0
        )
        time.sleep(1 / self._actuator.frequency)

    def set_gains(self, gains: ControlGains = DEFAULT_CURRENT_GAINS) -> None:
        return super().set_gains(gains)

    def set_command(self, value: Union[float, int]) -> None:
        return super().set_command(value)


class DephyPositionMode(ControlModeBase):
    def __init__(self, actuator: Union["DephyActpack", None] = None) -> None:
        super().__init__(
            control_mode_index=ControlModesMapping.POSITION,
            control_mode_name=str(ControlModesMapping.POSITION),
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
            max_gains=ControlGains(kp=1000, ki=1000, kd=1000, K=0, B=0, ff=0),
        )

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[DephyControlMode] Entering {self.name} mode.")

        if not self.has_gains:
            self.set_gains()

        self.set_command(value=0)

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[DephyControlMode] Exiting {self.name} mode.")

        # Is this necessary? This was a required step for older flexsea but not sure if it is needed anymore
        self._actuator.send_motor_command(
            ctrl_mode=ControlModesMapping.VOLTAGE, value=0
        )
        time.sleep(0.1)

    def set_gains(
        self,
        gains: ControlGains = DEFAULT_POSITION_GAINS,
    ) -> None:
        return super().set_gains(gains)

    def set_command(self, value: Union[float, int]) -> None:
        return super().set_command(value)


class DephyImpedanceMode(ControlModeBase):
    def __init__(self, actuator: Union["DephyActpack", None] = None) -> None:
        super().__init__(
            control_mode_index=ControlModesMapping.IMPEDANCE,
            control_mode_name=str(ControlModesMapping.IMPEDANCE),
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
            max_gains=ControlGains(kp=80, ki=800, kd=0, K=0, B=0, ff=128),
        )

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[DephyControlMode] Entering {self.name} mode.")
        if not self.has_gains:
            self.set_gains()

        self.set_command(self._actuator.motor_position)

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[DephyControlMode] Exiting {self.name} mode.")

        # Is this necessary? This was a required step for older flexsea but not sure if it is needed anymore
        self._actuator.send_motor_command(
            ctrl_mode=ControlModesMapping.VOLTAGE, value=0
        )
        time.sleep(1 / self._actuator.frequency)

    def set_gains(self, gains: ControlGains = DEFAULT_IMPEDANCE_GAINS):
        return super().set_gains(gains)

    def set_command(
        self,
        value: Union[float, int],
    ) -> None:
        super().set_command(value=value)


@dataclass(init=False)
class DephyActpackControlModes(ControlModesBase):
    VOLTAGE = DephyVoltageMode()
    CURRENT = DephyCurrentMode()
    POSITION = DephyPositionMode()
    IMPEDANCE = DephyImpedanceMode()

    def __init__(self, actuator: "DephyActpack") -> None:

        self.VOLTAGE.add_actuator(actuator)
        self.CURRENT.add_actuator(actuator)
        self.POSITION.add_actuator(actuator)
        self.IMPEDANCE.add_actuator(actuator)


class DephyActpack(ActuatorBase, Device):
    def __init__(
        self,
        name: str = "DephyActpack",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        debug_level: int = 0,
        dephy_log: bool = False,
        offline: bool = False,
    ) -> None:
        ActuatorBase.__init__(
            self,
            actuator_name=name,
            control_modes=DephyActpackControlModes(self),
            motor_constants=MotorConstants(
                MOTOR_COUNT_PER_REV=16384,
                NM_PER_AMP=0.1133,
                IMPEDANCE_A=0.00028444,
                IMPEDANCE_C=0.0007812,
                MAX_CASE_TEMPERATURE=80,
                M_PER_SEC_SQUARED_ACCLSB=9.80665 / 8192,
            ),
            frequency=frequency,
            offline=offline,
        )

        if not self.is_offline:
            Device.__init__(self, port=port, baud_rate=baud_rate)
        else:
            self.port = port
            self.is_streaming: bool = False
            self.is_open: bool = False

        self._debug_level: int = debug_level
        self._dephy_log: bool = dephy_log

        self._encoder_map = None
        self._motor_zero_position = 0.0
        self._motor_offset = 0.0

        self._mode: ControlModeBase = self.control_modes.VOLTAGE
        self.dephyIMU = (
            None  # TODO: Fix this, this shouldn't be used to return actpack variables
        )

    def __repr__(self) -> str:
        return f"{self.actuator_name}[DephyActpack]"

    def start(self) -> None:
        super().start()

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
        # self.dephyIMU.get_data()

        self._mode.enter()

    def stop(self) -> None:
        super().stop()

        self.set_control_mode(mode=self.control_modes.voltage)
        self.set_voltage(voltage_value=0)

        time.sleep(0.1)
        self.close()

    def update(self) -> None:
        super().update()

        if self.is_streaming:
            self._data = self.read()
            # self.dephyIMU.update()

            # Check for thermal fault, bit 2 of the execute status byte
            if self._data.status_ex & 0b00000010 == 0b00000010:
                raise RuntimeError("Actpack Thermal Limit Tripped")

        else:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Please open() the device before streaming data."
            )

    def set_control_mode(self, mode: ControlModeBase) -> None:
        super().set_control_mode(mode)

    def set_motor_voltage(self, voltage_value: float):
        if self._mode != self.control_modes.voltage:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set voltage in mode {self._mode}"
            )
            return

        self._mode.set_voltage(
            int(voltage_value),
        )

    def set_motor_current(
        self,
        current_value: float,
    ):
        if self._mode != self.control_modes.current:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set current in mode {self._mode}"
            )
            return

        self._mode.set_current(
            int(current_value),
        )

    def set_motor_torque(self, torque: float) -> None:
        """
        Sets the motor torque in Nm.

        Args:
            torque (float): The torque to set in Nm.
        """
        if self._mode != self.control_modes.current:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set motor_torque in mode {self._mode}"
            )
            return

        self._mode.set_current(
            int(torque / self._MecheConsts.NM_PER_MILLIAMP),
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
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set motor position in mode {self._mode}"
            )
            return

        self._mode.set_position(
            int(
                (position + self.motor_zero_position + self.motor_offset)
                / self._MecheConsts.RAD_PER_COUNT
            ),
        )

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
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set position gains in mode {self._mode}"
            )
            return

        self._mode.set_gains(ControlGains(kp=kp, ki=ki, kd=kd, K=0, B=0, ff=ff))  # type: ignore

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
            LOGGER.warning(
                f"[{self.__repr__()}] Cannot set current gains in mode {self._mode}"
            )
            return

        self._mode.set_gains(ControlGains(kp=kp, ki=ki, kd=0, K=0, B=0, ff=ff))  # type: ignore

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
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set impedance gains in mode {self._mode}"
            )
            return

        self._mode.set_gains(ControlGains(kp=kp, ki=ki, kd=0, K=K, B=B, ff=ff))  # type: ignore

    def set_encoder_map(self, encoder_map) -> None:
        """Sets the joint encoder map"""
        self._encoder_map = encoder_map

    def set_motor_zero_position(self, position: float) -> None:
        """Sets motor zero position in radians"""
        self._motor_zero_position = position

    def set_motor_offset(self, position: float) -> None:
        """Sets joint offset position in radians"""
        self._motor_offset = position

    def set_joint_zero_position(self, position: float) -> None:
        """Sets joint zero position in radians"""
        self.dephyIMU.joint_encoder._joint_zero_position = position

    def set_joint_offset(self, position: float) -> None:
        """Sets joint offset position in radians"""
        self.dephyIMU.joint_encoder._joint_offset = position

    def set_joint_direction(self, direction: float) -> None:
        """Sets joint direction to 1 or -1"""
        self.dephyIMU.joint_encoder._joint_direction = direction

    @property
    def frequency(self) -> int:
        return self._frequency

    @property
    # TODO: Eliminate after generalization
    def encoder_map(self):
        """Polynomial coefficients defining the joint encoder map from counts to radians."""
        return self._encoder_map

    @property
    # TODO: Eliminate after generalization
    def motor_zero_position(self) -> float:
        """Motor encoder zero position in radians."""
        return self._motor_zero_position

    @property
    # TODO: Eliminate after generalization
    def motor_offset(self) -> float:
        """Motor encoder offset in radians."""
        return self._motor_offset

    @property
    # TODO: Eliminate after generalization
    def motor_voltage(self) -> float:
        """Q-axis motor voltage in mV."""
        if self._data is not None:
            return float(self._data.mot_volt)
        else:
            return 0.0

    @property
    # TODO: Eliminate after generalization
    def motor_current(self) -> float:
        if self._data is not None:
            return float(self._data.mot_cur)
        else:
            return 0.0

    @property
    # TODO: Eliminate after generalization
    def motor_torque(self) -> float:
        if self._data is not None:
            return float(self._data.mot_cur * self._MecheConsts.NM_PER_MILLIAMP)
        else:
            return 0.0

    @property
    # TODO: Eliminate after generalization
    def motor_position(self) -> float:
        if self._data is not None:
            return (
                float(self._data.mot_ang * self._MecheConsts.RAD_PER_COUNT)
                - self._motor_zero_position
                - self.motor_offset
            )
        else:
            return 0.0

    @property
    # TODO: Eliminate after generalization
    def motor_encoder_counts(self) -> int:
        """Raw reading from motor encoder in counts."""
        return int(self._data.mot_ang)

    @property
    # TODO: Eliminate after generalization
    def motor_velocity(self) -> float:
        if self._data is not None:
            return int(self._data.mot_vel) * self._MecheConsts.RAD_PER_DEG
        else:
            return 0.0

    @property
    # TODO: Eliminate after generalization
    def motor_acceleration(self) -> float:
        if self._data is not None:
            return float(self._data.mot_acc)
        else:
            return 0.0

    @property
    # TODO: Eliminate after generalization
    def joint_zero_position(self) -> float:
        """Joint encoder zero position in radians."""
        return self.dephyIMU.joint_encoder.joint_zero_position

    @property
    # TODO: Eliminate after generalization
    def joint_offset(self) -> float:
        """Joint encoder offset in radians."""
        return self.dephyIMU.joint_encoder.joint_offset

    @property
    # TODO: Eliminate after generalization
    def joint_direction(self) -> float:
        """Joint direction: 1 or -1"""
        return self.dephyIMU.joint_encoder.joint_direction

    @property
    # TODO: Eliminate after generalization
    def joint_encoder_counts(self) -> int:
        self.update()
        """Raw reading from joint encoder in counts."""
        return self.dephyIMU.joint_encoder.joint_encoder_counts

    @property
    # TODO: Eliminate after generalization
    def joint_position(self) -> float:
        self.update()
        """Measured angle from the joint encoder in radians."""
        return self.dephyIMU.joint_encoder.joint_position

    @property
    # TODO: Eliminate after generalization
    def joint_velocity(self) -> float:
        self.update()
        return self.dephyIMU.joint_encoder.joint_velocity

    @property
    # TODO: Eliminate after generalization
    def battery_voltage(self) -> float:
        return self._data.battery_voltage

    @property
    # TODO: Eliminate after generalization
    def battery_current(self) -> float:
        """Battery current in mA."""
        self.update()
        return self.dephyIMU.battery.battery_current

    @property
    # TODO: Eliminate after generalization
    def case_temperature(self) -> float:
        self.update()
        return self.dephyIMU.thermal.case_temperature

    @property
    # TODO: Eliminate after generalization
    def winding_temperature(self) -> float:
        self.update()
        return self.dephyIMU.thermal.winding_temperature

    @property
    # TODO: Eliminate after generalization
    def thermal_scaling_factor(self) -> float:
        # self.update()
        return self.dephyIMU.thermal.thermal_scaling_factor

    @property
    # TODO: Eliminate after generalization
    def genvars(self):
        self.update()
        return self.dephyIMU.loadcell.genvars

    @property
    # TODO: Eliminate after generalization
    def accelx(self) -> float:
        self.update()
        return self.dephyIMU.loadcell.accelx

    @property
    # TODO: Eliminate after generalization
    def accely(self) -> float:
        self.update()
        return self.dephyIMU.loadcell.accely

    @property
    # TODO: Eliminate after generalization
    def accelz(self) -> float:
        self.update()
        return self.dephyIMU.loadcell.accelz

    @property
    # TODO: Eliminate after generalization
    def gyrox(self) -> float:
        self.update()
        return self.dephyIMU.loadcell.gyrox

    @property
    # TODO: Eliminate after generalization
    def gyroy(self) -> float:
        self.update()
        return self.dephyIMU.loadcell.gyroy

    @property
    # TODO: Eliminate after generalization
    def gyroz(self) -> float:
        self.update()
        return self.dephyIMU.loadcell.gyroz


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
        DephyActpack.__init__(self, offline=True)
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

        self.dephyIMU.data = MockData()
        self._data = MockData()
        self._thermal_model = self.dephyIMU.thermal._thermal_model

    def open(self, freq, log_level, log_enabled):
        if freq == 100 and log_level == 5 and log_enabled:
            raise OSError
        else:
            LOGGER.debug(msg=f"Opening Device at {self.port}")

    def send_motor_command(self, ctrl_mode, value):
        self._motor_command = f"Control Mode: {ctrl_mode}, Value: {value}"

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
        small_noise = 0

        self._data.batt_volt += 15
        self._data.batt_curr += 15
        self._data.mot_volt += 15
        self._data.mot_cur += 15
        self._data.mot_ang += 15
        self._data.ank_ang += 15
        self._data.mot_vel += 15
        self._data.mot_acc += 15
        self._data.ank_vel += 15
        self._data.temperature += 15
        self._data.genvar_0 += 15
        self._data.genvar_1 += 15
        self._data.genvar_2 += 15
        self._data.genvar_3 += 15
        self._data.genvar_4 += 15
        self._data.genvar_5 += 15
        self._data.accelx += 15
        self._data.accely += 15
        self._data.accelz += 15
        self._data.gyrox += 15
        self._data.gyroy += 15
        self._data.gyroz += 15
        self._data.status_ex = 0b00000000
        return self._data

    # Overrides the close method to do nothing
    def close(self):
        pass


if __name__ == "__main__":
    print("hola")
    dcm = DephyActpackControlModes(2)
    print(dcm)
