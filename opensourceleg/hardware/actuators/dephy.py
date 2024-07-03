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
    check_actuator_connection,
    check_actuator_control_mode,
    check_actuator_open,
    check_actuator_stream,
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
        LOGGER.debug(msg=f"[DephyControlMode] Entering {self.name} control mode.")

    def _exit(self) -> None:
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
        gear_ratio: float = 1.0,
        baud_rate: int = 230400,
        frequency: int = 500,
        debug_level: int = 0,
        dephy_log: bool = False,
        offline: bool = False,
    ) -> None:
        dephy_control_modes = DephyActpackControlModes(self)
        ActuatorBase.__init__(
            self,
            actuator_name=name,
            control_modes=dephy_control_modes,
            default_control_mode=dephy_control_modes.VOLTAGE,
            gear_ratio=gear_ratio,
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

        if self.is_offline:
            self.port = port
            self.is_streaming: bool = False
            self.is_open: bool = False
        else:
            Device.__init__(self, port=port, baud_rate=baud_rate)

        self._debug_level: int = debug_level
        self._dephy_log: bool = dephy_log

        self._encoder_map = None
        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0
        self._motor_offset = 0.0
        self._joint_offset = 0.0

    def __repr__(self) -> str:
        return f"{self.actuator_name}[DephyActpack]"

    @check_actuator_connection
    def start(self) -> None:
        self.open(
            freq=self._frequency,
            log_level=self._debug_level,
            log_enabled=self._dephy_log,
        )
        self._data = self.read()

        time.sleep(0.1)
        self.mode.enter()

    @check_actuator_stream
    @check_actuator_open
    def stop(self) -> None:
        self.set_control_mode(mode=self.CONTROL_MODES.VOLTAGE)
        self.set_motor_voltage(voltage_value=0)

        time.sleep(0.1)
        self.close()

    def update(self) -> None:
        self._data = self.read()

        # Check for thermal fault, bit 2 of the execute status byte
        if self._data.status_ex & 0b00000010 == 0b00000010:
            raise RuntimeError("Actpack Thermal Limit Tripped")

    def set_control_mode(self, mode: ControlModeBase) -> None:
        super().set_control_mode(mode)

    def set_motor_torque(self, value: float) -> None:
        """
        Sets the motor torque in Nm.

        Args:
            torque (float): The torque to set in Nm.
        """
        self.set_motor_current(
            int(value / self.MOTOR_CONSTANTS.NM_PER_MILLIAMP),
        )

    @check_actuator_control_mode(ControlModesMapping.CURRENT)
    def set_motor_current(
        self,
        value: float,
    ):
        self.mode.set_command(
            int(value),
        )

    @check_actuator_control_mode(ControlModesMapping.VOLTAGE)
    def set_motor_voltage(self, value: float) -> None:
        """
        Sets the motor voltage in mV.

        Args:
            voltage_value (float): The voltage to set in mV.
        """
        self.mode.set_command(
            int(value),
        )

    @check_actuator_control_mode(ControlModesMapping.POSITION)
    def set_motor_position(self, value: float) -> None:
        """
        Sets the motor position in radians.
        If in impedance mode, this sets the equilibrium angle in radians.

        Args:
            position (float): The position to set
        """
        self.mode.set_command(
            int(
                (value + self.motor_zero_position + self.motor_offset)
                / self.MOTOR_CONSTANTS.RAD_PER_COUNT
            ),
        )

    @check_actuator_control_mode(ControlModesMapping.POSITION)
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
        self.mode.set_gains(ControlGains(kp=kp, ki=ki, kd=kd, K=0, B=0, ff=ff))  # type: ignore

    @check_actuator_control_mode(ControlModesMapping.CURRENT)
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
        self.mode.set_gains(ControlGains(kp=kp, ki=ki, kd=0, K=0, B=0, ff=ff))  # type: ignore

    @check_actuator_control_mode(ControlModesMapping.IMPEDANCE)
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
        self.mode.set_gains(ControlGains(kp=kp, ki=ki, kd=0, K=K, B=B, ff=ff))  # type: ignore

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
        self._joint_zero_position = position

    def set_joint_offset(self, position: float) -> None:
        """Sets joint offset position in radians"""
        self._joint_offset = position

    def set_joint_direction(self, direction: float) -> None:
        """Sets joint direction to 1 or -1"""
        self._joint_direction = direction

    @property
    def encoder_map(self):
        """Polynomial coefficients defining the joint encoder map from counts to radians."""
        return self._encoder_map

    @property
    def motor_zero_position(self) -> float:
        """Motor encoder zero position in radians."""
        return self._motor_zero_position

    @property
    def motor_offset(self) -> float:
        """Motor encoder offset in radians."""
        return self._motor_offset

    @property
    def motor_voltage(self) -> float:
        """Q-axis motor voltage in mV."""
        if self._data is not None:
            return float(self._data.mot_volt)
        else:
            return 0.0

    @property
    def motor_current(self) -> float:
        if self._data is not None:
            return float(self._data.mot_cur)
        else:
            return 0.0

    @property
    def motor_torque(self) -> float:
        if self._data is not None:
            return float(self._data.mot_cur * self.MOTOR_CONSTANTS.NM_PER_MILLIAMP)
        else:
            return 0.0

    @property
    def motor_position(self) -> float:
        if self._data is not None:
            return (
                float(self._data.mot_ang * self.MOTOR_CONSTANTS.RAD_PER_COUNT)
                - self._motor_zero_position
                - self.motor_offset
            )
        else:
            return 0.0

    @property
    def motor_encoder_counts(self) -> int:
        """Raw reading from motor encoder in counts."""
        return int(self._data.mot_ang)

    @property
    def motor_velocity(self) -> float:
        if self._data is not None:
            return int(self._data.mot_vel) * self.MOTOR_CONSTANTS.RAD_PER_DEG
        else:
            return 0.0

    @property
    def motor_acceleration(self) -> float:
        if self._data is not None:
            return float(self._data.mot_acc)
        else:
            return 0.0

    @property
    def joint_zero_position(self) -> float:
        """Joint encoder zero position in radians."""
        return self._joint_zero_position

    @property
    def joint_offset(self) -> float:
        """Joint encoder offset in radians."""
        return self._joint_offset

    @property
    def joint_direction(self) -> float:
        """Joint direction: 1 or -1"""
        return self._joint_direction


if __name__ == "__main__":
    pass
