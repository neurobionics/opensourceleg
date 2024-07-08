"""
Moteus Controller for Open-Source Leg Project
07/2024"""

from typing import Any, Union

import os
import time
from dataclasses import dataclass
import asyncio
import time

import moteus
import moteus_pi3hat as pihat
import numpy as np

from opensourceleg.hardware.actuators.base import (
    ActuatorBase,
    ControlGains,
    ControlModeBase,
    ControlModesBase,
    ControlModesMapping,
    MotorConstants,
    ControlModeException, 
    check_actuator_connection,
    check_actuator_control_mode,
    check_actuator_open,
    check_actuator_stream,
)

from opensourceleg.tools.logger import LOGGER


DEFAULT_POSITION_GAINS = ControlGains(kp=0, ki=0, kd=0, k=0, b=0, ff=0)

DEFAULT_CURRENT_GAINS = ControlGains(kp=0, ki=0, kd=0, k=0, b=0, ff=0)

DEFAULT_IMPEDANCE_GAINS = ControlGains(kp=0, ki=0, kd=0, k=0, b=0, ff=0)


class MoteusStop(ControlModeBase):
    
    # TODO: check instance
    def __init__(self, actuator: Union["MoteusController", None] = None) -> None: 
        super().__init__(
            control_mode_map=ControlModesMapping.VOLTAGE,
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
        )
    def __repr__(self) -> str:
        return f"MoteusControlMode[{self.name}]"

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Entering {self.name} control mode.")

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Exiting {self.name} control mode.")
        self._actuator._command = None
        time.sleep(0.1)

    def set_gains(self, gains: ControlGains) -> None:
        LOGGER.info(
            msg=f"[{self.actuator.__repr__()}] {self.name} mode does not have gains."
        )

    def set_stop(self, value: Union[float, int]) -> None:
        if self.max_command is not None:
            assert 0 <= value <= self._max_command

        if self.actuator is not None:
            self._actuator._command = self._actuator._servo.make_stop()


class MoteusVoltageMode(ControlModeBase):
    def __init__(self, actuator: Union["MoteusController", None] = None) -> None: 
        super().__init__(
            control_mode_map=ControlModesMapping.VOLTAGE,
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
        )
    def __repr__(self) -> str:
        return f"MoteusControlMode[{self.name}]"

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Entering {self.name} control mode.")

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Exiting {self.name} control mode.")
        self._actuator._command = None
        time.sleep(0.1)

    def set_gains(self, gains: ControlGains) -> None:
        LOGGER.info(
            msg=f"[{self.actuator.__repr__()}] {self.name} mode does not have gains."
        )

    def set_voltage(self, value: Union[float, int]):
        self._actuator._command = self.actuator._servo.make_vfoc(theta = 0, voltage = value, query = True)

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

class MoteusCurrentMode(ControlModeBase):
    def __init__(self, actuator: Union["MoteusController", None] = None) -> None:
        super().__init__(
            control_mode_map=ControlModesMapping.CURRENT,
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
            max_gains=ControlGains(kp=80, ki=800, kd=0, k=0, b=0, ff=128),
        )

    def __repr__(self) -> str:
        return f"MoteusControlMode[{self.name}]"

    def _entry(self) -> None:

        LOGGER.debug(msg=f"[MoteusControlMode] Entering {self.name} mode.")

        if not self.has_gains:
            self.set_gains()

        self.set_command(value=0)

    def _exit(self) -> None:

        LOGGER.debug(msg=f"[MoteusControlMode] Exiting {self.name} mode.")

        # Is this necessary? This was a required step for older flexsea but not sure if it is needed anymore
        self._actuator._command = None
        time.sleep(1 / self.actuator.frequency)

    def set_gains(self, gains: ControlGains = DEFAULT_CURRENT_GAINS) -> None:
        super().set_gains(gains)

    def set_voltage(self, value: Union[float, int]):
        
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.CURRENT),
            mode=self.name,
        )
    def set_current(self, value: Union[float, int]):
        self._actuator._command = self.actuator._servo.make_current(d_A = value, q_A = 0, query = True)

    def set_position(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.POSITION),
            mode=self.name,
        )

class MoteusPositionMode(ControlModeBase):
    def __init__(self, actuator: Union["MoteusController", None] = None) -> None:
        super().__init__(
            control_mode_map=ControlModesMapping.POSITION,
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
            max_gains=ControlGains(kp=1000, ki=1000, kd=1000, k=0, b=0, ff=0),
        )

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Entering {self.name} mode.")

        if not self.has_gains:
            self.set_gains()

        self.set_command(value=0)

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Exiting {self.name} mode.")

        # Is this necessary? This was a required step for older flexsea but not sure if it is needed anymore
        self._actuator._command = None
        time.sleep(0.1)

    def set_gains(
        self,
        gains: ControlGains = DEFAULT_POSITION_GAINS,
    ) -> None:
        super().set_gains(gains)

    def set_voltage(self, value: Union[float, int]):
        
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.CURRENT),
            mode=self.name,
        )
    def set_current(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.CURRENT),
            mode=self.name,
        )

    def set_position(self, value: Union[float, int]):
        self._actuator._command = self.actuator._servo.make_vfoc(position = value, query = True)

@dataclass(init=False)
class MoteusControlModes(ControlModesBase):
    VOLTAGE = MoteusVoltageMode()
    CURRENT = MoteusCurrentMode()
    POSITION = MoteusPositionMode()

    def __init__(self, actuator: "MoteusController") -> None:

        self.VOLTAGE.add_actuator(actuator)
        self.CURRENT.add_actuator(actuator)
        self.POSITION.add_actuator(actuator)

class MoteusInterface:
    """
    Singleton Class as Communication Portal between Moteus Controller and Moteus PiHat
    """
    
    _instance = None
    
    def __new__(cls, *args, **kwargs): 
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        self.transport = None
        self.bus_map: dict[int: list[int]] = {}
        self._servos_id: list[int] = []
        self._commands: list[moteus.Command] = []
        # self._servos: dict[int: moteus.Controller] = {}
        
    def __repr__(self):
        return f"MoteusInterface"
        
    def add2map(self, servo_id, bus_id) -> None:
        #TODO: Implement bus map here
        if bus_id in self.bus_map.keys():
            self.bus_map[bus_id].append(servo_id)
        else:
            self.bus_map[bus_id] = [servo_id]
            
        # self._servos_id.append(servo_id)
    
    async def start(self):
        """
        Initialization of Pi3HatRouter
        """
        if self.transport is None:
            self.transport = pihat.Pi3HatRouter(
                servo_bus_map = self.bus_map
            )
        
    async def update(self):
        """
        update method for 
        """
        # self._data = await self.transport.cycle(
        #     self._commands
        # )
        self._commands = []
    
    async def stop(self):
        pass

class MoteusController(ActuatorBase):
    def __init__(
        self,
        name: str = "MoteusController",
        # TODO: create instance for joint names
        servo_id: int = 1,
        bus_id: int = 11,
        # transport: pihat.Pi3HatRouter = None,
        gear_ratio: float = 1.0,
        frequency: int = 500,
        offline: bool = False,
    ) -> None:
        self._servo_id = servo_id
        self._bus_id = bus_id
        moteus_control_modes = MoteusControlModes(actuator=self)
        super().__init__(
            actuator_name=name,
            control_modes=moteus_control_modes,
            default_control_mode=moteus_control_modes.VOLTAGE,
            gear_ratio=gear_ratio,
            motor_constants=MotorConstants(),
            frequency=frequency,
            offline=offline,
        )
        
        self._interface = MoteusInterface()
        self._interface.add2map(servo_id=servo_id, bus_id=bus_id)
        
        if self.is_offline: 
            self.is_streaming: bool = False
            self.is_open: bool = False
        else:
            
            #TODO: add communication to MoteusInterface
            pass
        
        self._encoder_map = None

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0

        self._joint_offset = 0.0
        self._motor_offset = 0.0
        self._joint_direction = 1.0
        
        self._servo: moteus.Controller = None
        self._command: moteus.Command = None

    def __repr__(self) -> str:
        return f"Moteus[{self._actuator_name}]"

    @check_actuator_connection
    def start(self) -> None:
        super().start()
        try:
            self._interface.start()
            self._servo = moteus.Controller(
                id = self._servo_id, 
                transport = self._interface.transport, 
                query_resolution=moteus.QueryResolution()
            )
        except OSError as e:
            print("\n")
            LOGGER.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        time.sleep(0.1)
        self._command = None
        self._mode.enter()

    @check_actuator_stream
    @check_actuator_open
    def stop(self) -> None:
        super().stop()
        self.set_motor_voltage(value=0)
        self._command = None
        time.sleep(0.1)

    def update(self) -> None:
        # TODO: update command
        super().update()
        # TODO: find a better way to send multiple commands at one time
        self._interface.transport.cycle(
            self._command
        )
        self._command = None
    
    def set_control_mode(self, mode: ControlModeBase) -> None:
        super().set_control_mode(mode)


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
        self.mode.set_gains(ControlGains(kp=kp, ki=ki, kd=kd, k=0, b=0, ff=ff))  # type: ignore

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
        self.mode.set_gains(ControlGains(kp=kp, ki=ki, kd=0, k=0, b=0, ff=ff))  # type: ignore

    def set_impedance_gains(
        self,
        kp: int,
        ki: int,
        kd: int,
        k: int,
        b: int,
        ff: int,
    ):
        """set_impedance_gains method for the actuator

        Args:
            kp (int): Proportional gain
            ki (int): Integral gain
            kd (int): Derivative gain
            k (int): Stiffness of the impedance controller
            b (int): Damping of the impedance controller
            ff (int): Feedforward gain
        """
        pass

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
    def CONTROL_MODES(self) -> ControlModesBase:
        """Control Modes (Read Only)

        Returns:
            List[ControlModeBase]: List of control modes
        """
        return self._CONTROL_MODES

    @property
    def MOTOR_CONSTANTS(self) -> MotorConstants:
        """Motor Constants (Read Only)

        Returns:
            MotorConstants: Motor Constants
        """
        return self._MOTOR_CONSTANTS

    @property
    def mode(self) -> ControlModeBase:
        """Current Control Mode (Read Only)

        Returns:
            ControlModeBase: Current control mode
        """
        return self._mode

    @property
    def actuator_name(self) -> str:
        """Actuator Name (Read Only)

        Returns:
            str: Actuator name
        """
        return self._actuator_name

    @property
    def frequency(self) -> int:
        """Frequency (Read Only)

        Returns:
            int: Frequency
        """
        return self._frequency

    @property
    def is_offline(self) -> bool:
        """Offline (Read Only)

        Returns:
            bool: Offline
        """
        return self._is_offline

    @property
    def gear_ratio(self) -> float:
        """Gear Ratio (Read Only)

        Returns:
            float: Gear Ratio
        """
        return self._gear_ratio


if __name__ == "__main__":
    pass
