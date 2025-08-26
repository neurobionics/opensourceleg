import math
import os
from typing import ClassVar, Optional

import numpy as np
from moteus import Command, Controller, Stream
from moteus import Register as MoteusRegister
from moteus import multiplex as mp

from opensourceleg.actuators.base import (
    CONTROL_MODE_CONFIGS,
    CONTROL_MODES,
    MOTOR_CONSTANTS,
    ActuatorBase,
    ControlGains,
    ControlModeConfig,
)
from opensourceleg.actuators.decorators import (
    check_actuator_connection,
    check_actuator_open,
    check_actuator_stream,
)
from opensourceleg.logging.logger import LOGGER
from opensourceleg.math import ThermalModel

try:
    import moteus_pi3hat as pihat

    _PIHAT_AVAILABLE = True
except ImportError:
    pihat = None
    _PIHAT_AVAILABLE = False

DEFAULT_POSITION_GAINS = ControlGains(kp=0.07, ki=0.08, kd=0.012, k=0, b=0, ff=0)

DEFAULT_VELOCITY_GAINS = ControlGains(kp=5.0, ki=0.2, kd=0.1, k=0, b=0, ff=0)

DEFAULT_TORQUE_GAINS = ControlGains(kp=0.025876, ki=76.910477, kd=0, k=0, b=0, ff=0)

DEFAULT_CURRENT_GAINS = ControlGains(kp=0, ki=0, kd=0, k=0, b=0, ff=0)

DEFAULT_IMPEDANCE_GAINS = ControlGains(kp=0, ki=0, kd=0, k=0, b=0, ff=0)

RAD_PER_DEG = np.pi / 180

MOTEUS_ACTUATOR_CONSTANTS = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=16384,
    NM_PER_AMP=0.1133,
    NM_PER_RAD_TO_K=1e-6,  # Small positive value - TODO: Change this value when impedance control is implemented
    NM_S_PER_RAD_TO_B=1e-6,  # Small positive value - TODO: Change this value when impedance control is implemented
    MAX_CASE_TEMPERATURE=80,
    MAX_WINDING_TEMPERATURE=110,
)


class MoteusQueryResolution:
    mode = mp.INT8
    position = mp.F32
    velocity = mp.F32
    torque = mp.F32

    voltage = mp.INT8
    temperature = mp.INT32
    fault = mp.INT8

    # Kept for Controller init reference
    q_current = mp.INT32
    d_current = mp.IGNORE
    abs_position = mp.IGNORE
    power = mp.IGNORE
    motor_temperature = mp.IGNORE
    trajectory_complete = mp.IGNORE
    rezero_state = mp.IGNORE
    home_state = mp.IGNORE

    aux1_gpio = mp.IGNORE
    aux2_gpio = mp.IGNORE

    _extra: ClassVar = {
        MoteusRegister.COMMAND_POSITION: mp.F32,
        MoteusRegister.COMMAND_VELOCITY: mp.F32,
        MoteusRegister.COMMAND_FEEDFORWARD_TORQUE: mp.F32,
        MoteusRegister.COMMAND_Q_CURRENT: mp.F32,
    }


def _moteus_velocity_mode_exit(moteus_actuator: "MoteusActuator") -> None:
    moteus_actuator.set_motor_velocity(0)


MOTEUS_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    POSITION=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=lambda _: None,
        has_gains=False,
        max_gains=ControlGains(kp=1000, ki=1000, kd=1000, k=0, b=0, ff=0),
    ),
    TORQUE=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=lambda _: None,
        has_gains=False,
        max_gains=ControlGains(kp=80, ki=800, kd=0, k=0, b=0, ff=128),
    ),
    VELOCITY=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=_moteus_velocity_mode_exit,
        has_gains=False,
        max_gains=ControlGains(kp=80, ki=800, kd=0, k=1000, b=1000, ff=128),
    ),
    IDLE=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=lambda _: None,
        has_gains=False,
        max_gains=None,
    ),
)


class MoteusInterface:
    """
    Singleton Class as Communication Portal between Moteus Controller and Moteus PiHat
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls.bus_map: dict[int, list[int]] = {}
            cls.actuator_map: dict[int, MoteusActuator] = {}
            cls._command_map: dict[int, Command] = {}
            cls._stop_servos: list[MoteusActuator] = []
            cls.transport = None

            batch = kwargs.pop("batch", False)  # default to false
            cls.batch = batch

        return cls._instance

    def __init__(self):
        pass

    def __repr__(self):
        return "MoteusInterface"

    def _add2map(self, servo_id, bus_id) -> None:
        if bus_id in self.bus_map:
            self.bus_map[bus_id].append(servo_id)
        else:
            self.bus_map[bus_id] = [servo_id]

    def start(self):
        """
        Initialization of Pi3HatRouter
        """
        if self.transport is None:
            if not _PIHAT_AVAILABLE:
                LOGGER.error(msg="Moteus PiHat not found. Please install the moteus_pi3hat package.")
                raise ImportError("moteus_pi3hat package is required for MoteusRouter")
            self.transport = pihat.Pi3HatRouter(servo_bus_map=self.bus_map)

    async def update(self):
        if not self.batch:
            LOGGER.warning("Batching is disabled. Use the update method of MoteusActuator")

        commands = []
        for servo_id in self.actuator_map:
            if servo_id in self._command_map:
                # Command has been set for this cycle
                commands.append(self._command_map[servo_id])
            else:
                # Default to query
                actuator = self.actuator_map.get(servo_id)
                commands.append(actuator.make_query())

        results = await self.transport.cycle(commands)

        for result in results:
            actuator = self.actuator_map.get(result.id)
            if actuator:
                actuator._data = result
                actuator.check_thermals()

        self._command_map.clear()

    async def stop(self):
        if not self.batch:
            LOGGER.warning("Batching is disabled. Use the stop method of MoteusActuator")

        if not self._stop_servos:
            return

        stopCommands = []

        for actuator in self._stop_servos:
            actuator.set_control_mode(mode=CONTROL_MODES.IDLE)
            stopCommands.append(actuator.make_stop(query=True))

        await self.transport.cycle(stopCommands)
        self._stop_servos.clear()


class MoteusActuator(ActuatorBase, Controller):
    def __init__(
        self,
        tag: str = "Moteus",
        servo_id: int = 0,
        bus_id: int = 0,
        gear_ratio: float = 1.0,
        frequency: int = 500,
        offline: bool = False,
        query: Optional[MoteusQueryResolution] = None,
    ) -> None:
        if query is None:
            query = MoteusQueryResolution()

        self._servo_id = servo_id
        self._bus_id = bus_id
        super().__init__(
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=MOTEUS_ACTUATOR_CONSTANTS,
            frequency=frequency,
            offline=offline,
        )

        self._interface = MoteusInterface()
        self._interface._add2map(servo_id=servo_id, bus_id=bus_id)
        self._interface.actuator_map[servo_id] = self

        self._is_streaming: bool = False
        self._is_open: bool = False

        self._command: Command = None
        self._data = None
        self._query = query

        self._thermal_model: ThermalModel = ThermalModel(
            motor_constants=self.MOTOR_CONSTANTS,
            actuator_tag=self.tag,
        )
        self._thermal_scale: float = 1.0

        self._mode = CONTROL_MODES.IDLE

    def __repr__(self) -> str:
        return f"Moteus[{self._tag}]"

    @property
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        return MOTEUS_CONTROL_MODE_CONFIGS

    @check_actuator_connection
    async def start(self) -> None:
        try:
            self._interface.start()
            Controller.__init__(
                self,
                id=self._servo_id,
                transport=self._interface.transport,
                query_resolution=self._query,
            )
            self._stream = Stream(controller=self)

            self._is_open = True
            self._is_streaming = True

        except OSError:
            print("\n")
            LOGGER.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port. \n\n \
                    Please run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        default_mode_config = self._get_control_mode_config(self._mode)
        if default_mode_config:
            default_mode_config.entry_callback(self)

        if (await self._interface.transport.cycle([self.make_stop(query=True)])) == []:
            LOGGER.error(msg=f"[{self.__repr__()}] Could not start the actuator. Please check the connection.")
            self._is_streaming = False
            self._is_open = False
        # Keep the default command as query -- reading sensor data
        self._command = self.make_query()

    @check_actuator_stream
    @check_actuator_open
    async def stop(self) -> None:
        if self._interface.batch:
            LOGGER.warning("Batching is enabled. Use the stop method of MoteusInterface")

        self.set_control_mode(mode=CONTROL_MODES.IDLE)

        await self._interface.transport.cycle([self.make_stop(query=True)])
        self._command = self.make_query()

    async def update(self):
        if self._interface.batch:
            LOGGER.warning("Batching is enabled. Use the update method of MoteusInterface")

        self._data = await self._interface.transport.cycle([self._command])
        self.check_thermals()
        self._command = self.make_query()

    def check_thermals(self):
        self._thermal_scale = self._thermal_model.update(
            dt=1 / self.frequency,
            motor_current=self.motor_current,
            case_temperature=self.case_temperature,
        )

    def enqueue_command(self) -> None:
        # Set the command for this cycle
        self._interface._command_map[self._servo_id] = self._command

    def enqueue_stop_command(self) -> None:
        self._interface._stop_servos.append(self)

    def home(
        self,
        homing_voltage: int = 2000,
        homing_frequency: Optional[int] = None,
        homing_direction: int = -1,
        output_position_offset: float = 0.0,
        current_threshold: int = 5000,
        velocity_threshold: float = 0.001,
    ) -> None:
        # TODO: implement homing
        LOGGER.info(msg=f"[{self.__repr__()}] Homing not implemented.")

    def set_motor_torque(self, value: float) -> None:
        """
        Sets the motor torque in Nm.

        Args:
            value (float): The torque to set in Nm.
        """
        self._command = self.make_position(
            position=math.nan,
            velocity=math.nan,
            feedforward_torque=value,
            kp_scale=0,
            kd_scale=0,
            ilimit_scale=0,
            watchdog_timeout=math.nan,
            query=True,
        )

        if self._interface.batch:
            self.enqueue_command()

    def set_output_torque(self, value: float) -> None:
        """
        Set the output torque of the actuator.
        This is the torque that is applied to the joint, not the motor.

        Args:
            value (float): torque in N_m
        """
        self.set_motor_torque(value=value / self.gear_ratio)

    def set_motor_current(
        self,
        value: float,
    ) -> None:
        LOGGER.info("Current Mode Not Implemented")

    def set_motor_velocity(self, value: float) -> None:
        self._command = self.make_position(
            position=math.nan,
            velocity=value / (np.pi * 2),  # TODO: Verify this conversion, are we converting from rad/s to rev/s?
            query=True,
            watchdog_timeout=math.nan,
        )

        if self._interface.batch:
            self.enqueue_command()

    def set_motor_voltage(self, value: float) -> None:
        """
        Sets the motor voltage in mV.

        Args:
            value (float): The voltage to set in mV.
        """
        LOGGER.info("Voltage Mode Not Implemented")

    def set_motor_position(self, value: float) -> None:
        """
        Sets the motor position in radians.
        If in impedance mode, this sets the equilibrium angle in radians.

        Args:
            value (float): The position to set
        """
        self._command = self.make_position(
            position=float((value) / (2 * np.pi)),  # TODO: Verify this conversion, are we converting from rad to rev?
            query=True,
            watchdog_timeout=math.nan,
        )

        if self._interface.batch:
            self.enqueue_command()

    async def set_torque_gains(
        self,
        kp: float = DEFAULT_TORQUE_GAINS.kp,
        ki: float = DEFAULT_TORQUE_GAINS.ki,
    ) -> None:
        """
        Sets the position gains in arbitrary Moteus units.

        Args:
            kp (float): The proportional gain
            ki (float): The integral gain
        """
        await self._stream.command(f"conf set servo.pid_dq.kp {kp}".encode())
        await self._stream.command(f"conf set servo.pid_dq.ki {ki}".encode())

    async def set_position_gains(
        self,
        kp: float = DEFAULT_POSITION_GAINS.kp,
        ki: float = DEFAULT_POSITION_GAINS.ki,
        kd: float = DEFAULT_POSITION_GAINS.kd,
        ff: float = DEFAULT_POSITION_GAINS.ff,
    ) -> None:
        """
        Sets the position gains in arbitrary Moteus units.

        Args:
            kp (float): The proportional gain
            ki (float): The integral gain
            kd (float): The derivative gain
            ff (float): The feedforward gain
        """
        await self._stream.command(f"conf set servo.pid_position.kp {kp}".encode())
        await self._stream.command(f"conf set servo.pid_position.ki {ki}".encode())
        await self._stream.command(f"conf set servo.pid_position.kd {kd}".encode())

    async def set_velocity_gains(
        self,
        kp: float = DEFAULT_VELOCITY_GAINS.kp,
        ki: float = DEFAULT_VELOCITY_GAINS.ki,
        kd: float = DEFAULT_VELOCITY_GAINS.kd,
        ff: float = DEFAULT_VELOCITY_GAINS.ff,
    ) -> None:
        """
        Sets the position gains in arbitrary Moteus units.

        Args:
            kp (float): The proportional gain
            ki (float): The integral gain
            kd (float): The derivative gain
            ff (float): The feedforward gain
        """
        await self._stream.command(f"conf set servo.pid_position.kp {kp}".encode())
        await self._stream.command(f"conf set servo.pid_position.ki {ki}".encode())
        await self._stream.command(f"conf set servo.pid_position.kd {kd}".encode())

    def set_current_gains(
        self,
        kp: float = DEFAULT_CURRENT_GAINS.kp,
        ki: float = DEFAULT_CURRENT_GAINS.ki,
        kd: float = DEFAULT_CURRENT_GAINS.kd,
        ff: float = DEFAULT_CURRENT_GAINS.ff,
    ) -> None:
        """
        Sets the current gains in arbitrary Moteus units.

        Args:
            kp (float): The proportional gain
            ki (float): The integral gain
            kd (float): The derivative gain
            ff (float): The feedforward gain
        """
        LOGGER.info(msg=f"[{self.__repr__()}] Current mode not implemented.")

    def _set_impedance_gains(
        self,
        k: float = DEFAULT_IMPEDANCE_GAINS.k,
        b: float = DEFAULT_IMPEDANCE_GAINS.b,
    ) -> None:
        LOGGER.info(msg=f"[{self.__repr__()}] Impedance mode not implemented.")

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
            return float(self.motor_current * self.MOTOR_CONSTANTS.NM_PER_MILLIAMP) / self.gear_ratio
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_position(self) -> float:
        if self._data is not None:
            return float(self._data[0].values[MoteusRegister.POSITION] * 2 * np.pi) - self.motor_zero_position
        else:
            LOGGER.warning(
                msg="Actuator data is none, please ensure that the actuator is connected and streaming. Returning 0.0."
            )
            return 0.0

    @property
    def motor_velocity(self) -> float:
        if self._data is not None:
            return float(self._data[0].values[MoteusRegister.VELOCITY] * 2 * np.pi)
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
    def joint_torque(self) -> float:
        """
        Torque at the joint output in Nm.
        This is calculated using motor current, k_t, and the gear ratio.
        """
        return self.motor_torque * self.gear_ratio

    @property
    def case_temperature(self) -> float:
        if self._data is not None:
            return float(self._data[0].values[MoteusRegister.TEMPERATURE])
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
            return float(self._thermal_model.winding_temperature)
        else:
            return 0.0

    @property
    def thermal_scaling_factor(self) -> float:
        """
        Scale factor to use in torque control, in [0,1].
        If you scale the torque command by this factor, the motor temperature will
        never exceed max allowable temperature.
        For a proof, see paper referenced in thermal model.
        """
        return self._thermal_scale


if __name__ == "__main__":
    pass
