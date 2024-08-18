from typing import (
    Any,
    Callable,
    Dict,
    NamedTuple,
    Optional,
    Protocol,
    Set,
    TypeVar,
    cast,
)

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from functools import partial

import numpy as np

from opensourceleg.logging.logger import LOGGER


@dataclass(frozen=True)
class MOTOR_CONSTANTS:
    MOTOR_COUNT_PER_REV: float = 16384
    NM_PER_AMP: float = 0.1133
    IMPEDANCE_A: float = 0.00028444
    IMPEDANCE_C: float = 0.0007812
    MAX_CASE_TEMPERATURE: float = 80
    MAX_WINDING_TEMPERATURE: float = 110
    M_PER_SEC_SQUARED_ACCLSB: float = 9.80665 / 8192

    @property
    def NM_PER_MILLIAMP(self) -> float:
        return self.NM_PER_AMP / 1000

    @property
    def RAD_PER_COUNT(self) -> float:
        return 2 * np.pi / self.MOTOR_COUNT_PER_REV

    @property
    def RAD_PER_DEG(self) -> float:
        return np.pi / 180

    @property
    def RAD_PER_SEC_GYROLSB(self) -> float:
        return np.pi / 180 / 32.8

    @property
    def NM_PER_RAD_TO_K(self) -> float:
        return self.RAD_PER_COUNT / self.IMPEDANCE_C * 1e3 / self.NM_PER_AMP

    @property
    def NM_S_PER_RAD_TO_B(self) -> float:
        return self.RAD_PER_DEG / self.IMPEDANCE_A * 1e3 / self.NM_PER_AMP

    def __repr__(self) -> str:
        return "MOTOR_CONSTANTS"


class CONTROL_MODES(Enum):
    POSITION = 0
    CURRENT = 1
    VOLTAGE = 2
    IMPEDANCE = 3
    VELOCITY = 4
    TORQUE = 5
    IDLE = 6


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
    entry_callback: Callable[["ActuatorBase"], None]
    exit_callback: Callable[["ActuatorBase"], None]
    has_gains: bool = False
    max_gains: ControlGains = ControlGains()


class CONTROL_MODE_CONFIGS(NamedTuple):
    POSITION: Optional[ControlModeConfig] = None
    CURRENT: Optional[ControlModeConfig] = None
    VOLTAGE: Optional[ControlModeConfig] = None
    IMPEDANCE: Optional[ControlModeConfig] = None
    VELOCITY: Optional[ControlModeConfig] = None
    TORQUE: Optional[ControlModeConfig] = None
    IDLE: Optional[ControlModeConfig] = None


T = TypeVar("T", bound=Callable)


class MethodWithRequiredModes(Protocol):
    _required_modes: set[CONTROL_MODES]


def requires(*modes: CONTROL_MODES):
    def decorator(func: T) -> T:
        setattr(func, "_required_modes", set(modes))
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
        for method_name in dir(self):
            if not method_name.startswith("_"):
                method = getattr(self, method_name)
                if callable(method) and hasattr(method, "_required_modes"):
                    self._original_methods[method_name] = method

    def _set_mutated_methods(self):
        for method_name, method in self._original_methods.items():
            if self._mode in method._required_modes:
                setattr(self, method_name, method)
            else:
                setattr(
                    self, method_name, partial(self._restricted_method, method_name)
                )

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

    def set_control_mode(self, mode: CONTROL_MODES) -> None:
        if self._mode == mode:
            return

        current_config = cast(
            Optional[ControlModeConfig],
            getattr(self._CONTROL_MODE_CONFIGS, self._mode.name),
        )
        if current_config:
            current_config.exit_callback(self)

        new_config = cast(
            Optional[ControlModeConfig],
            getattr(self._CONTROL_MODE_CONFIGS, mode.name),
        )
        if new_config:
            new_config.entry_callback(self)

        self._mode = mode
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
    def set_impedance_gains(
        self, kp: float, ki: float, kd: float, k: float, b: float, ff: float
    ) -> None:
        pass

    @abstractmethod
    def home(self) -> None:
        pass

    @property
    @abstractmethod
    def motor_position(self) -> float:
        pass

    @property
    @abstractmethod
    def motor_velocity(self) -> float:
        pass

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
    def output_position(self) -> float:
        return self.motor_position / self.gear_ratio

    @property
    def output_velocity(self) -> float:
        return self.motor_velocity / self.gear_ratio


if __name__ == "__main__":
    pass
