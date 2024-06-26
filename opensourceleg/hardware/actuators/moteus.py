from typing import Any, Callable, Union, overload

import ctypes
import enum
import os
import time
from ctypes import c_int
from dataclasses import dataclass

import moteus
import moteus_pi3hat as pihat
import numpy as np

import opensourceleg.hardware.actuators.base as base
from opensourceleg.hardware.thermal import ThermalModel
from opensourceleg.tools.logger import Logger

# from flexsea.device import Device


DEFAULT_POSITION_GAINS = base.ControlGains(kp=0, ki=0, kd=0, K=0, B=0, ff=0)

DEFAULT_CURRENT_GAINS = base.ControlGains(kp=0, ki=0, kd=0, K=0, B=0, ff=0)

DEFAULT_IMPEDANCE_GAINS = base.ControlGains(kp=0, ki=0, kd=0, K=0, B=0, ff=0)


class VoltageMode(base.VoltageMode):
    def __init__(self, device: "MoteusObject") -> None:
        super().__init__(mode_pass=moteus.Mode.VOLTAGE, device=device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit
        self._device: MoteusObject = device
        # self._control_mode = moteus.Mode.VOLTAGE

    def _entry(self) -> None:
        # TODO: check log instance
        self._device._log.debug(msg=f"[Actpack] Entering Voltage mode.")

    def _exit(self) -> None:
        # TODO: check log instance
        self._device._log.debug(msg=f"[Actpack] Exiting Voltage mode.")
        # self.set_voltage(voltage_value=0)
        time.sleep(0.1)

    def set_voltage(self, voltage_value: int) -> None:
        # TODO: Check instances here
        # TODO: Check servo mappings
        self._device._commands.append(
            self._device._servos[1].make_vfoc(
                theta=0, voltage=voltage_value, query=True
            )
        )


class CurrentMode(base.CurrentMode):
    def __init__(self, device: "MoteusObject") -> None:
        super().__init__(mode_pass=moteus.Mode.CURRENT, device=device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit
        self._device: MoteusObject = device
        self._gains = DEFAULT_CURRENT_GAINS

        # self._control_mode = moteus.Mode.CURRENT

    def _entry(self) -> None:

        # TODO: check log instance
        self._device._log.debug(msg=f"[Actpack] Entering Current mode.")

        if not self.has_gains:
            self.set_gains(DEFAULT_CURRENT_GAINS)

        self.set_current(current_value=0)

    def _exit(self) -> None:
        # TODO: check log instance
        self._device._log.debug(msg=f"[Actpack] Exiting Current mode.")
        # self._device.send_motor_command(ctrl_mode=fxe.FX_VOLTAGE, value=0)

    def set_current(
        self,
        current_value: int,
    ) -> None:
        """Sets the Q-axis current of the motor

        Args:
            current_value (int): _description_
        """
        super().set_current(current_value=current_value)

        # assert 0 <= Gains.kp <= 80, "kp must be between 0 and 80"
        # assert 0 <= Gains.ki <= 800, "ki must be between 0 and 800"
        # assert 0 <= Gains.ff <= 128, "ff must be between 0 and 128"

        # self._device.set_gains(kp=Gains.kp, ki=Gains.ki, kd=0, k=0, b=0, ff=Gains.ff)
        # TODO: Check instances here
        self._device._commands.append(
            self._device._servos[1].make_current(
                # TODO: Check servo mappings
                q_A=current_value,
                query=True,
            )
        )

    def set_gains(
        self,
        gains: base.ControlGains = DEFAULT_CURRENT_GAINS,
    ) -> None:

        assert 0 <= gains.kp <= 80, "kp must be between 0 and 80"
        assert 0 <= gains.ki <= 800, "ki must be between 0 and 800"
        assert 0 <= gains.ff <= 128, "ff must be between 0 and 128"
        self._has_gains = True
        self._gains = gains
        # TODO: Check Gain Instances
        # self._device.set_gains(kp=gains.kp, ki=gains.ki, kd=0, k=0, b=0, ff=gains.ff)


class PositionMode(base.PositionMode):
    def __init__(self, device: "MoteusObject") -> None:
        super().__init__(mode_pass=moteus.Mode.POSITION, device=device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit
        self._device: MoteusObject = device
        self._gains = DEFAULT_POSITION_GAINS
        # self._control_mode = moteus.Mode.POSITION
        pass

    def _entry(self) -> None:
        self._device._log.debug(msg=f"[Actpack] Entering Position mode.")

        if not self.has_gains:
            self.set_gains(self._gains)

        self.set_position(encoder_count=0)

    def _exit(self) -> None:
        self._device._log.debug(msg=f"[Actpack] Exiting Position mode.")
        # self._device.send_motor_command(ctrl_mode=fxe.FX_VOLTAGE, value=0)
        time.sleep(0.1)

    def set_position(
        self,
        encoder_count: int,
    ) -> None:
        super().set_position(encoder_count=encoder_count)
        self._device._commands.append(
            self._device._servos[1].make_position(
                # TODO: Check servo mappings
                position=encoder_count,
                velocity=0.1,
                query=True,
            )
        )

    def set_gains(
        self,
        gains: base.ControlGains = DEFAULT_POSITION_GAINS,
    ) -> None:

        assert 0 <= gains.kp <= 1000, "kp must be between 0 and 1000"
        assert 0 <= gains.ki <= 1000, "ki must be between 0 and 1000"
        assert 0 <= gains.kd <= 1000, "kd must be between 0 and 1000"
        self._has_gains = True
        self._gains = gains
        # TODO: Check Gain Instances
        # self._device.set_gains(
        #     kp=gains.kp, ki=gains.ki, kd=gains.kd, k=0, b=0, ff=gains.ff
        # )


@dataclass(init=False)
class MoteusControlModes:
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
    # impedance: ImpedanceMode

    def __init__(self, device: "MoteusObject") -> None:
        self.voltage = VoltageMode(device=device)
        self.current = CurrentMode(device=device)
        self.position = PositionMode(device=device)
        # self.impedance = ImpedanceMode(device=device)

    def __repr__(self) -> str:
        return f"ActpackControlModes"


class MoteusObject(base.Actuator):
    def __init__(
        self,
        name: str = "MoteusObject",
        addr_map = {
            1: [11],
            2: [12],
        },
        bitrate: int = 125000,
        logger: Logger = Logger(),
        # dephy_log: bool = False,
        *args,
        **kwargs,
    ) -> None:
        # TODO: check for better ways!!!
        os.system("sudo bash")
        base.Actuator.__init__(
            self,
            Gains=base.ControlGains(),
            MecheSpecs=base.MechanicalConstants(),
        )
        self._transport = pihat.Pi3HatRouter(servo_bus_map=addr_map)
        self._commands: list[moteus.Command]

        self._servos = {
            servo_id: moteus.Controller(id=servo_id, transport=self._transport)
            for servo_id in addr_map
        }
        self._name: str = name

        # self._frequency: int = frequency
        # self._debug_level: int = debug_level
        # # self._dephy_log: bool = dephy_log
        # self._name: str = name
        self._log: Logger = logger
        self._encoder_map = None

        self._CANbus = pihat.CanConfiguration()
        self._CANbus.bitrate = bitrate
        self._CANbus.fdcan_frame = False
        self._CANbus.bitrate_switch = False

        self._motor_zero_position = 0.0

        self._motor_offset = 0.0

        self.control_modes: MoteusControlModes = MoteusControlModes(device=self)

        self._mode: base.ActuatorMode
        self._data: Any = None

        self._result: Any = None

    def __repr__(self) -> str:
        return f"MoteusObject[{self._name}]"

    def start(self) -> None:
        super().start()
        try:
            self._result = self._transport.cycle(
                [x.make_stop() for x in self._servos.values()]
            )
        except OSError as e:
            print("\n")
            self._log.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        time.sleep(0.1)
        self._mode.enter()

    def stop(self) -> None:
        super().stop()
        self._transport.cycle([x.make_stop()] for x in self._servos.values())

        time.sleep(0.1)
        # self.close()

    def update(self) -> None:
        super().update()
        if hasattr(self._result, "id"):
            # TODO: check instance here
            self._result = self._transport.cycle(self._commands)
        else:
            self._log.warning(
                msg=f"[{self.__repr__()}] Please open() the device before streaming data."
            )

    def set_mode(self, mode: base.ActuatorMode) -> None:
        if type(mode) in [VoltageMode, CurrentMode, PositionMode]:
            self._mode.transition(to_state=mode)
            self._mode = mode
        else:
            self._log.warning(msg=f"[{self.__repr__()}] Mode {mode} not found")

    def set_voltage(self, voltage_value: float):
        if self._mode != self.control_modes.voltage:
            self._log.warning(
                msg=f"[{self.__repr__()}] Cannot set voltage in mode {self._mode}"
            )
            return

        self._mode.set_voltage(
            int(voltage_value),
        )

    def set_current(
        self,
        current_value: float,
    ):
        if self._mode != self.control_modes.current:
            self._log.warning(
                msg=f"[{self.__repr__()}] Cannot set current in mode {self._mode}"
            )
            return

        self._mode.set_current(
            int(current_value),
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
            self._log.warning(
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
            self._log.warning(
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
            self._log.warning(
                f"[{self.__repr__()}] Cannot set current gains in mode {self._mode}"
            )
            return

        self._mode.set_gains(base.ControlGains(kp=kp, ki=ki, kd=0, K=0, B=0, ff=ff))  # type: ignore

    def set_motor_zero_position(self, position: float) -> None:
        """Sets motor zero position in radians"""
        self._motor_zero_position = position

    def set_motor_offset(self, position: float) -> None:
        """Sets joint offset position in radians"""
        self._motor_offset = position

    def set_encoder_map(self, encoder_map) -> None:
        """Sets the joint encoder map"""
        self._encoder_map = encoder_map


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


if __name__ == "__main__":
    pass
