"""
Moteus Controller for Open-Source Leg Project
07/2024"""

from typing import Any

import os
import time

import moteus
import moteus.regression
import moteus_pi3hat as pihat
import numpy as np

import opensourceleg.hardware.actuators.base as base
from opensourceleg.logging.logger import LOGGER

# from flexsea.device import Device

DEFAULT_POSITION_GAINS = base.ControlGains(kp=0, ki=0, kd=0, K=0, B=0, ff=0)

DEFAULT_CURRENT_GAINS = base.ControlGains(kp=0, ki=0, kd=0, K=0, B=0, ff=0)

DEFAULT_IMPEDANCE_GAINS = base.ControlGains(kp=0, ki=0, kd=0, K=0, B=0, ff=0)


class ControlModes(base.ControlModesMapping):
    """TODO: Check Mapping to the Moteus Modes"""

    VOLTAGE = moteus.Register.VOLTAGE, "voltage"
    CURRENT = moteus.Register.D_CURRENT, "current"
    POSITION = moteus.Register.POSITION, "position"


class VoltageMode(base.ControlModeBase):
    def __init__(self, actuator: "Moteus") -> None:
        super().__init__(
            control_mode_index=ControlModes.VOLTAGE,
            control_mode_name=str(ControlModes.VOLTAGE),
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
        )

    def _entry(self) -> None:
        # TODO: check log instance
        LOGGER.debug(msg=f"[Moteus] Entering {self.name} mode.")

    def _exit(self) -> None:
        # TODO: check log instance
        LOGGER.debug(msg=f"[Moteus] Exiting {self.name} mode.")
        # self.set_voltage(voltage_value=0)
        time.sleep(0.1)

    def set_gains(self, gains: base.ControlGains) -> None:
        LOGGER.info(
            msg=f"[{self._actuator.__repr__()}] {self.name} mode does not have gains."
        )

    def set_command(self, value: float | int) -> None:
        super().set_command(
            value,
        )
        self._actuator._commands.append(
            self._actuator._servo.make_vfoc(
                voltage=value,
            )
        )
        # TODO: add instance here
        pass


class CurrentMode(base.ControlModeBase):
    def __init__(self, actuator: "Moteus") -> None:
        super().__init__(
            control_mode_index=ControlModes.VOLTAGE,
            control_mode_name=str(ControlModes.VOLTAGE),
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
        )

    def _entry(self) -> None:

        # TODO: check log instance
        LOGGER.debug(msg=f"[Moteus] Entering {self.name} mode.")

        if not self.has_gains:
            self.set_gains(DEFAULT_CURRENT_GAINS)

        self.set_current(current_value=0)

    def _exit(self) -> None:
        # TODO: check log instance
        LOGGER.debug(msg=f"[Moteus] Exiting Current mode.")
        # self._device.send_motor_command(ctrl_mode=fxe.FX_VOLTAGE, value=0)
        time.sleep(1 / self._actuator.frequency)

    def set_command(
        self,
        value: float | int,
    ) -> None:
        # TODO: add instance here
        """Sets the Q-axis current of the motor

        Args:
            current_value (int): _description_
        """
        super().set_command(
            value,
        )
        self._actuator._commands.append(
            self._actuator._servo.make_current(
                q_A=value,
            )
        )

    def set_gains(
        self,
        gains: base.ControlGains = DEFAULT_CURRENT_GAINS,
    ) -> None:
        # TODO: Check Gain Instances
        super().set_gains(
            gains,
        )
        # self._device.set_gains(kp=gains.kp, ki=gains.ki, kd=0, k=0, b=0, ff=gains.ff)


class PositionMode(base.ControlModeBase):
    def __init__(self, actuator: "Moteus") -> None:
        super().__init__(
            control_mode_index=ControlModes.VOLTAGE,
            control_mode_name=str(ControlModes.VOLTAGE),
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
        )

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[Moteus] Entering {self.name} mode.")

        if not self.has_gains:
            self.set_gains(self._gains)

        self.set_position(encoder_count=0)

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[Moteus] Exiting {self.name} mode.")
        time.sleep(0.1)

    def set_position(
        self,
        encoder_count: int,
    ) -> None:
        super().set_position(encoder_count=encoder_count)
        self._actuator._commands.append(
            self._actuator._servo.make_vfoc(
                position=encoder_count,
            )
        )

    def set_gains(
        self,
        gains: base.ControlGains = DEFAULT_POSITION_GAINS,
    ) -> None:
        # TODO: Check Gain Instances
        super().set_gains(
            gains,
        )


class Moteus(base.ActuatorBase):
    def __init__(
        self,
        name: str = "MoteusObject",
        dev_id: int = 11,
        addr_map={
            1: [11],
        },
        frequency: int = 500,
        # dephy_log: bool = False,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(
            self,
            actuator_name=name,
            control_modes=[
                VoltageMode(actuator=self),
                CurrentMode(actuator=self),
                PositionMode(actuator=self),
            ],
            frequency=frequency,
        )
        self._addr_map = addr_map

        self._hat_bus_item = []
        self._controller_id_item = []

        for index, (hat_bus, controller_id) in enumerate(self._addr_map.items()):
            self._hat_bus_item.append(hat_bus)
            self._controller_id_item.append(controller_id)

        self._transport = pihat.Pi3HatRouter(
            servo_bus_map=addr_map,
        )
        # self._dephy_log: bool = dephy_log
        self._encoder_map = None

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0

        self._joint_offset = 0.0
        self._motor_offset = 0.0

        self._joint_direction = 1.0

        self._servo = {
            item: moteus.Controller(
                id=item,
                transport=self._transport,
                # query_resolution=qr,
            )
            for row in self._controller_id_item
            for item in row
        }
        self._commands: list[moteus.Command] = []

    def __repr__(self) -> str:
        return f"Moteus[{self._actuator_name}]"

    def start(self) -> None:
        super().start()
        try:
            self._transport.cycle([x.make_stop() for x in self._servo.values()])
        except OSError as e:
            print("\n")
            LOGGER.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port '{self.port}'. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        time.sleep(0.1)
        self._data = self.read()
        self._mode.enter()

    def stop(self) -> None:
        super().stop()
        self._transport.cycle([x.make_stop() for x in self._servo.values()])
        self.set_voltage(voltage_value=0)
        self._commands = []
        # time.sleep(0.1)
        # self.close()

    def update(self) -> None:
        super().update()
        self._transport.cycle(
            self._commands,
        )
        self._commands = []

    def set_mode(self, mode: base.ControlModeBase) -> None:
        if type(mode) in self._control_modes:
            self._mode.transition(to_state=mode)
            self._mode = mode
        else:
            LOGGER.warning(msg=f"[{self.__repr__()}] Mode {mode} not found")
            pass

    def set_voltage(self, voltage_value: float):
        if self._mode != self.control_modes.voltage:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set voltage in mode {self._mode}"
            )
            return

        self._mode.set_command(
            voltage_value,
        )

    def set_current(
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

        self._mode.set_command(
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
            # self.control_modes.impedance,
        ]:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set motor position in mode {self._mode}"
            )
            return

        self._mode.set_command(
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

        self._mode.set_gains(base.ControlGains(kp=kp, ki=ki, kd=kd, K=0, B=0, ff=ff))  # type: ignore

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

        self._mode.set_gains(base.ControlGains(kp=kp, ki=ki, kd=0, K=0, B=0, ff=ff))  # type: ignore


if __name__ == "__main__":
    pass
