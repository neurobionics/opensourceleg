from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from functools import partial
from typing import (
    Any,
    Callable,
    NamedTuple,
    Optional,
    Protocol,
    TypeVar,
    Union,
    cast,
    runtime_checkable,
)

import numpy as np

from opensourceleg.logging.logger import LOGGER

# TODO: Add validators for every custom data type


@dataclass
class MOTOR_CONSTANTS:
    MOTOR_COUNT_PER_REV: float
    NM_PER_AMP: float
    NM_PER_RAD_TO_K: float
    NM_S_PER_RAD_TO_B: float
    MAX_CASE_TEMPERATURE: float
    MAX_WINDING_TEMPERATURE: float

    def __post_init__(self):
        if any(x <= 0 for x in self.__dict__.values()):
            raise ValueError("All values in MOTOR_CONSTANTS must be non-zero and positive.")

    @property
    def RAD_PER_COUNT(self) -> float:
        return 2 * np.pi / self.MOTOR_COUNT_PER_REV

    @property
    def NM_PER_MILLIAMP(self) -> float:
        return self.NM_PER_AMP / 1000


class CONTROL_MODES(Enum):
    IDLE = -1
    POSITION = 0
    VOLTAGE = 1
    CURRENT = 2
    IMPEDANCE = 3
    VELOCITY = 4
    TORQUE = 5


# TODO: This can be ordered and requires validation
@dataclass
class ControlGains:
    kp: float = 0
    ki: float = 0
    kd: float = 0
    k: float = 0
    b: float = 0
    ff: float = 0


@dataclass
class ControlModeConfig:
    entry_callback: Callable[[Any], None]
    exit_callback: Callable[[Any], None]
    has_gains: bool = False
    max_gains: Union[ControlGains, None] = None


class CONTROL_MODE_CONFIGS(NamedTuple):
    IDLE: Optional[ControlModeConfig] = None
    POSITION: Optional[ControlModeConfig] = None
    CURRENT: Optional[ControlModeConfig] = None
    VOLTAGE: Optional[ControlModeConfig] = None
    IMPEDANCE: Optional[ControlModeConfig] = None
    VELOCITY: Optional[ControlModeConfig] = None
    TORQUE: Optional[ControlModeConfig] = None


CONTROL_MODE_METHODS: list[str] = [
    "set_motor_torque",
    "set_joint_torque",
    "set_output_torque",
    "set_motor_current",
    "set_current",
    "set_motor_voltage",
    "set_voltage",
    "set_motor_position",
    "set_position_gains",
    "set_current_gains",
    "set_motor_impedance",
    "set_output_impedance",
    "set_impedance_gains",
]


T = TypeVar("T", bound=Callable[..., Any])


@runtime_checkable
class MethodWithRequiredModes(Protocol):
    _required_modes: set[CONTROL_MODES]


def requires(*modes: CONTROL_MODES):
    def decorator(func: T) -> T:
        if not all(isinstance(mode, CONTROL_MODES) for mode in modes):
            raise TypeError("All arguments to 'requires' must be of type CONTROL_MODES")

        if not hasattr(func, "_required_modes"):
            func._required_modes = set(modes)
        else:
            func._required_modes.update(modes)

        return func

    return decorator


class ActuatorBase(ABC):
    def __init__(
        self,
        tag: str,
        gear_ratio: float,
        motor_constants: MOTOR_CONSTANTS,
        frequency: int = 1000,
        offline: bool = False,
        *args,
        **kwargs,
    ) -> None:
        self._MOTOR_CONSTANTS: MOTOR_CONSTANTS = motor_constants
        self._gear_ratio: float = gear_ratio
        self._tag: str = tag
        self._frequency: int = frequency
        self._data: Any = None
        self._is_offline: bool = offline
        self._is_homed: bool = False

        self._mode: CONTROL_MODES = CONTROL_MODES.IDLE

        self._motor_zero_position: float = 0.0
        self._motor_position_offset: float = 0.0

        self._joint_zero_position: float = 0.0
        self._joint_position_offset: float = 0.0
        self._joint_direction: int = 1

        self._is_open: bool = False
        self._is_streaming: bool = False

        self._original_methods: dict[str, MethodWithRequiredModes] = {}

        self._set_original_methods()
        self._set_mutated_methods()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.stop()

    def _restricted_method(self, method_name: str, *args, **kwargs):
        LOGGER.error(f"{method_name}() is not available in {self._mode.name} mode.")
        return None

    def _set_original_methods(self):
        for method_name in CONTROL_MODE_METHODS:
            try:
                method = getattr(self, method_name)
                if callable(method) and hasattr(method, "_required_modes"):
                    self._original_methods[method_name] = method
            except AttributeError:
                LOGGER.debug(msg=f"[{self.tag}] {method_name}() is not implemented in {self.tag}.")

    def _set_mutated_methods(self):
        for method_name, method in self._original_methods.items():
            if self._mode in method._required_modes:
                setattr(self, method_name, method)
            else:
                setattr(self, method_name, partial(self._restricted_method, method_name))

    @property
    @abstractmethod
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        pass

    @abstractmethod
    def start(self) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

    def _get_control_mode_config(self, mode: CONTROL_MODES) -> Optional[ControlModeConfig]:
        return cast(
            Optional[ControlModeConfig],
            getattr(self._CONTROL_MODE_CONFIGS, mode.name),
        )

    def set_control_mode(self, mode: CONTROL_MODES) -> None:
        if self.mode == mode:
            LOGGER.debug(msg=f"[{self.tag}] Already in {self.mode.name} control mode.")
            return

        current_config = self._get_control_mode_config(self.mode)
        if current_config:
            current_config.exit_callback(self)

        self._mode = mode

        new_config = self._get_control_mode_config(self.mode)
        if new_config:
            new_config.entry_callback(self)

        self._set_mutated_methods()

    @abstractmethod
    @requires(CONTROL_MODES.VOLTAGE)
    def set_motor_voltage(self, value: float) -> None:
        pass

    @abstractmethod
    @requires(CONTROL_MODES.CURRENT)
    def set_motor_current(self, value: float) -> None:
        pass

    @abstractmethod
    @requires(CONTROL_MODES.POSITION)
    def set_motor_position(self, value: float) -> None:
        pass

    @requires(
        CONTROL_MODES.POSITION
    )  # This needs to be tested as set_motor_position is already decorated with requires
    def set_output_position(self, value: float) -> None:
        self.set_motor_position(value=value * self.gear_ratio)

    @abstractmethod
    @requires(CONTROL_MODES.TORQUE)
    def set_motor_torque(self, value: float) -> None:
        pass

    @abstractmethod
    @requires(CONTROL_MODES.TORQUE)
    def set_joint_torque(self, value: float) -> None:
        pass

    @abstractmethod
    @requires(CONTROL_MODES.CURRENT)
    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        pass

    @abstractmethod
    @requires(CONTROL_MODES.POSITION)
    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        pass

    @abstractmethod
    @requires(CONTROL_MODES.IMPEDANCE)
    def set_impedance_gains(self, kp: float, ki: float, kd: float, k: float, b: float, ff: float) -> None:
        pass

    @abstractmethod
    def home(self) -> None:
        pass

    def set_motor_zero_position(self, value: float) -> None:
        """Sets motor zero position in radians"""
        self._motor_zero_position = value

    def set_motor_position_offset(self, value: float) -> None:
        """Sets joint offset position in radians"""
        self._motor_position_offset = value

    def set_joint_zero_position(self, value: float) -> None:
        """Sets joint zero position in radians"""
        self._joint_zero_position = value

    def set_joint_position_offset(self, value: float) -> None:
        """Sets joint offset position in radians"""
        self._joint_position_offset = value

    def set_joint_direction(self, value: int) -> None:
        """Sets joint direction to 1 or -1"""
        self._joint_direction = value

    @property
    @abstractmethod
    def motor_position(self) -> float:
        pass

    @property
    def output_position(self) -> float:
        """
        Position of the output in radians.
        This is calculated by scaling the motor angle with the gear ratio.
        Note that this method does not consider compliance from an SEA.
        """
        return self.motor_position / self.gear_ratio

    @property
    @abstractmethod
    def motor_velocity(self) -> float:
        pass

    @property
    def output_velocity(self) -> float:
        """
        Velocity of the output in radians.
        This is calculated by scaling the motor angle with the gear ratio.
        Note that this method does not consider compliance from an SEA.
        """
        return self.motor_velocity / self.gear_ratio

    @property
    @abstractmethod
    def motor_voltage(self) -> float:
        pass

    @property
    @abstractmethod
    def motor_current(self) -> float:
        pass

    @property
    @abstractmethod
    def motor_torque(self) -> float:
        pass

    @property
    def MOTOR_CONSTANTS(self) -> MOTOR_CONSTANTS:
        return self._MOTOR_CONSTANTS

    @property
    def mode(self) -> CONTROL_MODES:
        return self._mode

    @property
    def tag(self) -> str:
        return self._tag

    @property
    def is_homed(self) -> bool:
        return self._is_homed

    @property
    def frequency(self) -> int:
        return self._frequency

    @property
    def is_offline(self) -> bool:
        return self._is_offline

    @property
    def gear_ratio(self) -> float:
        return self._gear_ratio

    @property
    def max_case_temperature(self) -> float:
        return self._MOTOR_CONSTANTS.MAX_CASE_TEMPERATURE

    @property
    @abstractmethod
    def case_temperature(self) -> float:
        pass

    @property
    @abstractmethod
    def winding_temperature(self) -> float:
        pass

    @property
    def max_winding_temperature(self) -> float:
        return self._MOTOR_CONSTANTS.MAX_WINDING_TEMPERATURE

    @property
    def motor_zero_position(self) -> float:
        return self._motor_zero_position

    @property
    def motor_position_offset(self) -> float:
        return self._motor_position_offset

    @property
    def joint_zero_position(self) -> float:
        return self._joint_zero_position

    @property
    def joint_position_offset(self) -> float:
        return self._joint_position_offset

    @property
    def joint_direction(self) -> int:
        return self._joint_direction

    @property
    def is_open(self) -> bool:
        return self._is_open

    @property
    def is_streaming(self) -> bool:
        return self._is_streaming
