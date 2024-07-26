"""
Moteus Controller for Open-Source Leg Project
07/2024
"""

from typing import Any, Union

import os
import time
import math
from dataclasses import dataclass
import time
import asyncio
import numpy as np
from moteus import (
    Controller, 
    Command, 
    Register as MoteusRegister,
    multiplex as mp, 
) 
import moteus_pi3hat as pihat

from opensourceleg.actuators.base import (
    ActuatorBase,
    ControlGains,
    ControlModeBase,
    ControlModesBase,
    ControlModesMapping,
    MotorConstants,
)
from opensourceleg.actuators.decorators import (
    check_actuator_connection,
    check_actuator_open,
    check_actuator_stream,
)
from opensourceleg.actuators.exceptions import (
    ActuatorIsNoneException,
    ControlModeException,
)
from opensourceleg.logging.decorators import (
    deprecated,
    deprecated_with_routing,
    deprecated_with_suggestion,
)

from opensourceleg.math import ThermalModel

from opensourceleg.safety import ThermalLimitException

from opensourceleg.logging.logger import LOGGER


# Default gains to be tuned
DEFAULT_POSITION_GAINS = ControlGains(kp=1, ki=1, kd=1, k=0, b=0, ff=0)

DEFAULT_VELOCITY_GAINS = ControlGains(kp=1, ki=0.5, kd=1, k=0, b=0, ff=0)

DEFAULT_CURRENT_GAINS = ControlGains(kp=0, ki=0, kd=0, k=0, b=0, ff=0)

DEFAULT_IMPEDANCE_GAINS = ControlGains(kp=0, ki=0, kd=0, k=0, b=0, ff=0)

class MoteusQueryResolution:

    mode = mp.INT8
    position = mp.F32
    velocity = mp.F32
    torque = mp.F32
    
    voltage = mp.INT8
    temperature = mp.INT32
    fault = mp.INT8


    # Kept for Controller init reference
    q_current = mp.F32
    d_current = mp.IGNORE
    abs_position = mp.IGNORE
    power = mp.IGNORE
    motor_temperature = mp.IGNORE
    trajectory_complete = mp.IGNORE
    rezero_state = mp.IGNORE
    home_state = mp.IGNORE

    aux1_gpio = mp.IGNORE
    aux2_gpio = mp.IGNORE

    _extra = {
        MoteusRegister.COMMAND_KP_SCALE: mp.F32,
        MoteusRegister.COMMAND_ILIMIT_SCALE: mp.F32,
        MoteusRegister.COMMAND_KD_SCALE: mp.F32,
        MoteusRegister.COMMAND_POSITION: mp.F32,
        MoteusRegister.COMMAND_VELOCITY: mp.F32,
    }

class MoteusStopMode(ControlModeBase):
    def __init__(self, actuator: Union["MoteusController", None] = None) -> None:
        super().__init__(
            control_mode_map=ControlModesMapping.STOP,
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
        )

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Entering {self.name} control mode.")

        if self.actuator is None:
            raise ActuatorIsNoneException(mode=self.name)

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Exiting {self.name} control mode.")
        time.sleep(0.1)

    def set_gains(self, gains: ControlGains) -> None:
        LOGGER.info(
            msg=f"[{self._actuator.__repr__()}] {self.name} mode does not have gains."
        )

    def set_velocity(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.VELOCITY),
            mode=self.name,
        )
    
    def set_voltage(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.VOLTAGE),
            mode=self.name,
        )

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
    def set_stop(self):
        self.actuator._command = self.actuator.make_stop(query = True)
    
    def set_torque(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.TORQUE),
            mode=self.name,
        )

class MoteusVelocityMode(ControlModeBase):
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

        if self.actuator is None:
            raise ActuatorIsNoneException(mode=self.name)

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Exiting {self.name} control mode.")
        self.set_velocity(0)
        time.sleep(0.1)

    def set_gains(self, gains: ControlGains) -> None:
        super().set_gains(gains)

    def set_velocity(self, value: Union[float, int]):
        self.actuator._command = self.actuator.make_position(
            position = math.nan, 
            kp_scale = self._gains.kp, 
            kd_scale = self._gains.kd,
            ilimit_scale = self._gains.ki,
            velocity = value / (np.pi * 2),
            query = True, 
            # feedforward_torque = 0.1,
            watchdog_timeout = math.nan,
        )
    
    def set_voltage(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.VOLTAGE),
            mode=self.name,
        )

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
    def set_stop(self):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.STOP),
            mode=self.name,
        )
    def set_torque(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.TORQUE),
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

        if self.actuator is None:
            raise ActuatorIsNoneException(mode=self.name)

        if not self.has_gains:
            self.set_gains()

        self.set_position(value=0)

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Exiting {self.name} mode.")

        # Is this necessary? This was a required step for older flexsea but not sure if it is needed anymore
        
        time.sleep(0.1)

    def set_gains(
        self,
        gains: ControlGains = DEFAULT_POSITION_GAINS,
    ) -> None:
        super().set_gains(gains)

    def set_position(self, value: Union[float, int]):
        print(value)
        self.actuator._command = self.actuator.make_position(
            kp_scale = self._gains.kp, 
            kd_scale = self._gains.kd,
            ilimit_scale = self._gains.ki,
            position = float((value) / (2 * np.pi)), # in revolutions
            query = True,
            watchdog_timeout = math.nan,
        )
    def set_current(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.CURRENT),
            mode=self.name,
        )

    def set_velocity(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.VELOCITY),
            mode=self.name,
        )
    
    def set_voltage(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.VOLTAGE),
            mode=self.name,
        )
    
    def set_stop(self):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.STOP),
            mode=self.name,
        )
    def set_torque(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.TORQUE),
            mode=self.name,
        )


class MoteusTorqueMode(ControlModeBase):
    def __init__(self, actuator: Union["MoteusController", None] = None) -> None:
        super().__init__(
            control_mode_map=ControlModesMapping.TORQUE,
            actuator=actuator,
            entry_callbacks=[self._entry],
            exit_callbacks=[self._exit],
            max_gains=ControlGains(kp=1000, ki=1000, kd=1000, k=0, b=0, ff=0),
        )

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Entering {self.name} mode.")

        if self.actuator is None:
            raise ActuatorIsNoneException(mode=self.name)

        if not self.has_gains:
            self.set_gains()


    def _exit(self) -> None:
        LOGGER.debug(msg=f"[MoteusControlMode] Exiting {self.name} mode.")

        # Is this necessary? This was a required step for older flexsea but not sure if it is needed anymore
        
        time.sleep(0.1)

    def set_gains(
        self,
        gains: ControlGains = DEFAULT_POSITION_GAINS,
    ) -> None:
        LOGGER.info(
            msg=f"[{self._actuator.__repr__()}] {self.name} mode does not have gains."
        )

    def set_position(self, value: Union[float, int]):
        print(value)
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.POSITION),
            mode=self.name,
        )
    def set_current(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.CURRENT),
            mode=self.name,
        )

    def set_velocity(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.VELOCITY),
            mode=self.name,
        )
    
    def set_voltage(self, value: Union[float, int]):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.VOLTAGE),
            mode=self.name,
        )
    
    def set_stop(self):
        raise ControlModeException(
            tag=self.actuator.tag,
            attribute=str(ControlModesMapping.STOP),
            mode=self.name,
        )
    
    def set_torque(self, value: Union[float, int]):
        self.actuator._command = self.actuator.make_position(
                position = math.nan, 
                velocity = 0,
                feedforward_torque = value, 
                kp_scale=0, 
                kd_scale=0,
                ilimit_scale=0, 
                watchdog_timeout = math.nan,
                query = True, 
        )


@dataclass(init=False)
class MoteusControlModes(ControlModesBase):

    def __init__(self, actuator: "MoteusController") -> None:

        self.VELOCITY = MoteusVelocityMode(actuator=actuator)
        self.POSITION = MoteusPositionMode(actuator=actuator)
        self.STOP = MoteusStopMode(actuator=actuator)
        self.TORQUE = MoteusTorqueMode(actuator=actuator)

class MoteusInterface:
    """
    Singleton Class as Communication Portal between Moteus Controller and Moteus PiHat
    """
    
    _instance = None
    
    def __new__(cls, *args, **kwargs): 
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls.bus_map: dict[int: list[int]] = {}
            cls._commands: list[Command] = []
            cls.transport = None
        return cls._instance
    
    def __init__(self):
        pass
    
    def __repr__(self):
        return f"MoteusInterface"
        
    def _add2map(self, servo_id, bus_id) -> None:
        
        if bus_id in self.bus_map.keys():
            self.bus_map[bus_id].append(servo_id)
        else:
            self.bus_map[bus_id] = [servo_id]
            
    def start(self):
        """
        Initialization of Pi3HatRouter
        """
        if self.transport is None:
            self.transport = pihat.Pi3HatRouter(
                servo_bus_map = self.bus_map
            )
        
    async def update(self):
        # TODO: multiple servo syncing process should go here
        self._commands = []
    
    async def stop(self):
        # TODO: multiple servo syncing process should go here
        self._commands = []

class MoteusController(ActuatorBase, Controller):
    def __init__(
        self,
        tag: str = "Moteus",
        servo_id: int = 0,
        bus_id: int = 0,
        gear_ratio: float = 1.0,
        frequency: int = 500,
        offline: bool = False,
        query: MoteusQueryResolution = MoteusQueryResolution(),
    ) -> None:
        self._servo_id = servo_id
        self._bus_id = bus_id
        moteus_control_modes = MoteusControlModes(self)
        super().__init__(
            tag=tag,
            control_modes=moteus_control_modes,
            default_control_mode=moteus_control_modes.STOP,
            gear_ratio=gear_ratio,
            motor_constants=MotorConstants(),
            frequency=frequency,
            offline=offline,
        )
        
        self._interface = MoteusInterface()
        self._interface._add2map(servo_id=servo_id, bus_id=bus_id)
        
        if self.is_offline: 
            self.is_streaming: bool = False
            self.is_open: bool = False
        else:
            # for streaming check
            self.is_streaming: bool = True
            self.is_open: bool = True
        
        self._command: Command = None
        self._data = None
        self._query = query
        
        self._encoder_map = None

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0
        self._motor_position_offset = 0.0

        self._is_homed: bool = False

        self._joint_offset = 0.0
        self._motor_position_offset = 0.0
        self._joint_direction = 1.0
        
        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=self.max_winding_temperature,
            soft_border_C_windings=10,
            temp_limit_case=self.max_case_temperature,
            soft_border_C_case=10,
        )
        self._thermal_scale: float = 1.0
        
        

    def __repr__(self) -> str:
        return f"Moteus[{self._tag}]"

    @check_actuator_connection
    def start(self) -> None:
        super().start()
        try:
            self._interface.start()
            Controller.__init__(
                self, 
                id = self._servo_id, 
                transport=self._interface.transport, 
                query_resolution=self._query,
            )
        except OSError as e:
            print("\n")
            LOGGER.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        self.mode.enter()
        self.mode.set_stop()
        self._command = self.make_query()

    @check_actuator_stream
    @check_actuator_open
    async def stop(self) -> None:
        super().stop()
        self.set_control_mode(mode = self.CONTROL_MODES.STOP)
        self.set_motor_stop()
        await self._interface.transport.cycle(
            [self._command]
        )
        self._command = self.make_query()
        

    async def update(self):
        super().update()
        self._data = await self._interface.transport.cycle(
            [self._command]
        )
        
        self._thermal_model.T_c = self.case_temperature
        self._thermal_scale = self._thermal_model.update_and_get_scale(
            dt = 1/self.frequency,
            motor_current = self.motor_current,
        )
        if self.case_temperature >= self.max_case_temperature:
                self._log.error(
                    msg=f"[{str.upper(self._name)}] Case thermal limit {self.max_case_temperature} reached. Stopping motor."
                )
                raise ThermalLimitException()

        if self.winding_temperature >= self.max_winding_temperature:
            self._log.error(
                msg=f"[{str.upper(self._name)}] Winding thermal limit {self.max_winding_temperature} reached. Stopping motor."
            )
            raise ThermalLimitException()
        self._command = self.make_query()

    def set_motor_stop(self): 
        self.mode.set_stop()

    def home(self): 
        # To be continued ... 
        pass
    
    
    def set_control_mode(self, mode: ControlModeBase) -> None:
        super().set_control_mode(mode)

    def set_motor_torque(self, value: float) -> None:
        """
        Sets the motor torque in Nm.

        Args:
            value (float): The torque to set in Nm.
        """
        self.mode.set_torque(
            value, 
        )

    def set_joint_torque(self, value: float) -> None:
        """
        Set the joint torque of the joint.
        This is the torque that is applied to the joint, not the motor.

        Args:
            value (float): torque in N_m
        """
        self.set_motor_torque(value=value / self.gear_ratio)


    def set_motor_current(
        self,
        value: float,
    ):
        LOGGER.info(f"Voltage Mode Not Implemented")

    def set_motor_velocity(self, value: float) -> None:
        self.mode.set_velocity(
            value = value,
        )
    
    def set_motor_voltage(self, value: float) -> None:
        """
        Sets the motor voltage in mV.

        Args:
            voltage_value (float): The voltage to set in mV.
        """
        # self._command = self.make_vfoc(
        #     theta = 0, 
        #     voltage = value, 
        #     query = True, 
        # )
        LOGGER.info(f"Voltage Mode Not Implemented")

    def set_motor_position(self, value: float) -> None:
        """
        Sets the motor position in radians.
        If in impedance mode, this sets the equilibrium angle in radians.

        Args:
            position (float): The position to set
        """
        self.mode.set_position(
            value = value, 
        )
        # self._command = self.make_position(
        #     position = value, 
        #     query = True, 
        # )

    def set_position_gains(
        self,
        kp: int = DEFAULT_POSITION_GAINS.kp,
        ki: int = DEFAULT_POSITION_GAINS.ki,
        kd: int = DEFAULT_POSITION_GAINS.kd,
        ff: int = DEFAULT_POSITION_GAINS.ff,
    ) -> None:
        """
        Sets the position gains in arbitrary Moteus units.

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            kd (int): The derivative gain
            ff (int): The feedforward gain
        """
        self.mode.set_gains(ControlGains(kp=kp, ki=ki, kd=kd, k=0, b=0, ff=ff))  # type: ignore

    def set_velocity_gains(
        self,
        kp: int = DEFAULT_VELOCITY_GAINS.kp,
        ki: int = DEFAULT_VELOCITY_GAINS.ki,
        kd: int = DEFAULT_VELOCITY_GAINS.kd,
        ff: int = DEFAULT_VELOCITY_GAINS.ff,
    ) -> None:
        """
        Sets the position gains in arbitrary Moteus units.

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            kd (int): The derivative gain
            ff (int): The feedforward gain
        """
        self.mode.set_gains(ControlGains(kp=kp, ki=ki, kd=kd, k=0, b=0, ff=ff))  # type: ignore

    def set_current_gains(
        self,
        kp: int = DEFAULT_CURRENT_GAINS.kp,
        ki: int = DEFAULT_CURRENT_GAINS.ki,
        ff: int = DEFAULT_CURRENT_GAINS.ff,
    ) -> None:
        """
        Sets the current gains in arbitrary Moteus units.

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            ff (int): The feedforward gain
        """
        self.mode.set_gains(ControlGains(kp=kp, ki=ki, kd=0, k=0, b=0, ff=ff))  # type: ignore

    def set_impedance_gains(
        self, 
        kp: int = DEFAULT_IMPEDANCE_GAINS.kp,
        ki: int = DEFAULT_IMPEDANCE_GAINS.ki,
        kd: int = DEFAULT_IMPEDANCE_GAINS.kd,
        ff: int = DEFAULT_IMPEDANCE_GAINS.ff,
    ) -> None: 
        LOGGER.info(msg=f"[MoteusControlMode] Impedance mode not applicable.")
    
    def set_encoder_map(self, encoder_map) -> None:
        """Sets the joint encoder map"""
        self._encoder_map = encoder_map

    def set_motor_zero_position(self, position: float) -> None:
        """Sets motor zero position in radians"""
        self._motor_zero_position = position

    def set_motor_position_offset(self, position: float) -> None:
        """Sets joint offset position in radians"""
        self._motor_position_offset = position

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
    def joint_zero_position(self) -> float:
        """Joint encoder zero position in radians."""
        return self._joint_zero_position

    @property
    def joint_position_offset(self) -> float:
        """Joint encoder offset in radians."""
        return self._joint_position_offset

    @property
    def joint_direction(self) -> float:
        """Joint direction: 1 or -1"""
        return self._joint_direction

    @property
    def encoder_map(self):
        """Polynomial coefficients defining the joint encoder map from counts to radians."""
        return self._encoder_map

    @property
    def motor_zero_position(self) -> float:
        """Motor encoder zero position in radians."""
        return self._motor_zero_position

    @property
    def motor_position_offset(self) -> float:
        """Motor encoder offset in radians."""
        return self._motor_position_offset

    @property
    def motor_voltage(self) -> float:
        """Q-axis motor voltage in mV."""
        if self._data is not None:
            return float(self._data[0].values[MoteusRegister.VOLTAGE])
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_current(self) -> float:
        if self._data is not None:
            return float(self._data[0].values[MoteusRegister.Q_CURRENT])
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_torque(self) -> float:
        if self._data is not None:
            return float(self.motor_current * self.MOTOR_CONSTANTS.NM_PER_MILLIAMP)
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_position(self) -> float:
        if self._data is not None:
            return (
                float(self._data[0].values[MoteusRegister.POSITION] * 2 * np.pi)
                - self.motor_zero_position
                - self.motor_position_offset
            )
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_velocity(self) -> float:
        if self._data is not None:
            return self._data[0].values[MoteusRegister.VELOCITY] * 2 * np.pi
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def battery_voltage(self) -> float:
        """Battery voltage in mV."""
        if self._data is not None:
            return float(self._data[0].values[MoteusRegister.VOLTAGE])
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def battery_current(self) -> float:
        """Battery current in mA."""
        if self._data is not None:
            return float(self._data[0].values[MoteusRegister.Q_CURRENT])
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def output_position(self) -> float:
        """
        Position of the output in radians.
        This is calculated by scaling the motor angle with the gear ratio.
        Note that this method does not consider compliance from an SEA.
        """
        return self.motor_position / self.gear_ratio

    @property
    def output_velocity(self) -> float:
        """
        Velocity of the output in radians.
        This is calculated by scaling the motor angle with the gear ratio.
        Note that this method does not consider compliance from an SEA.
        """
        return self.motor_velocity / self.gear_ratio

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
            return self._data[0].values[MoteusRegister.TEMPERATURE]
        else:
            LOGGER.warning(
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
    def thermal_scaling_factor(self) -> float:
        """
        Scale factor to use in torque control, in [0,1].
        If you scale the torque command by this factor, the motor temperature will never exceed max allowable temperature.
        For a proof, see paper referenced in thermal model.
        """
        return self._thermal_scale

if __name__ == "__main__":
    pass