from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from functools import partial
from typing import (
    Any,
    Callable,
    ClassVar,
    NamedTuple,
    Optional,
    Protocol,
    TypeVar,
    Union,
    cast,
    runtime_checkable,
)

import numpy as np

from opensourceleg.logging.exceptions import ControlModeException
from opensourceleg.logging.logger import LOGGER

# TODO: Add validators for every custom data type


@dataclass
class MOTOR_CONSTANTS:
    """
    Class to define the motor constants.

    Examples:
        >>> constants = MOTOR_CONSTANTS(
        ...     MOTOR_COUNT_PER_REV=2048,
        ...     NM_PER_AMP=0.02,
        ...     NM_PER_RAD_TO_K=0.001,
        ...     NM_S_PER_RAD_TO_B=0.0001,
        ...     MAX_CASE_TEMPERATURE=80.0,
        ...     MAX_WINDING_TEMPERATURE=120.0
        ... )
        >>> print(constants.MOTOR_COUNT_PER_REV)
        2048
    """

    MOTOR_COUNT_PER_REV: float
    NM_PER_AMP: float
    NM_PER_RAD_TO_K: float
    NM_S_PER_RAD_TO_B: float
    MAX_CASE_TEMPERATURE: float
    MAX_WINDING_TEMPERATURE: float

    def __post_init__(self) -> None:
        """
        Function to validate the motor constants.

        Examples:
            >>> # This will raise a ValueError because a negative value is invalid.
            >>> MOTOR_CONSTANTS(
            ...     MOTOR_COUNT_PER_REV=-2048,
            ...     NM_PER_AMP=0.02,
            ...     NM_PER_RAD_TO_K=0.001,
            ...     NM_S_PER_RAD_TO_B=0.0001,
            ...     MAX_CASE_TEMPERATURE=80.0,
            ...     MAX_WINDING_TEMPERATURE=120.0
            ... )
        """
        if any(x <= 0 for x in self.__dict__.values()):
            raise ValueError("All values in MOTOR_CONSTANTS must be non-zero and positive.")

    @property
    def RAD_PER_COUNT(self) -> float:
        """
        Calculate the radians per count.

        Returns:
            Radians per count.

        Examples:
            >>> constants = MOTOR_CONSTANTS(2048, 0.02, 0.001, 0.0001, 80.0, 120.0)
            >>> constants.RAD_PER_COUNT
            0.0030679615757712823
        """
        return 2 * np.pi / self.MOTOR_COUNT_PER_REV

    @property
    def NM_PER_MILLIAMP(self) -> float:
        """
        Convert NM per amp to NM per milliamp.

        Returns:
            NM per milliamp.

        Examples:
            >>> constants = MOTOR_CONSTANTS(2048, 0.02, 0.001, 0.0001, 80.0, 120.0)
            >>> constants.NM_PER_MILLIAMP
            2e-05
        """
        return self.NM_PER_AMP / 1000


class CONTROL_MODES(Enum):
    """
    Enum to define various control modes.

    Examples:
        >>> CONTROL_MODES.POSITION
        <CONTROL_MODES.POSITION: 0>
    """

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
    """
    Class to define the control gains.

    Examples:
        >>> gains = ControlGains(kp=1.0, ki=0.1, kd=0.01, k=0.5, b=0.05, ff=0.0)
        >>> gains.kp
        1.0
    """

    kp: float = 0
    ki: float = 0
    kd: float = 0
    k: float = 0
    b: float = 0
    ff: float = 0


@dataclass
class ControlModeConfig:
    """
    Configuration for a control mode.

    Attributes:
        entry_callback: Callback to execute when entering this mode.
        exit_callback: Callback to execute when exiting this mode.
        has_gains: Indicates if the control mode utilizes control gains.
        max_gains: The maximum allowable control gains (if applicable).

    Examples:
        >>> def enter(actuator):
        ...     print("Entering mode")
        >>> def exit(actuator):
        ...     print("Exiting mode")
        >>> config = ControlModeConfig(
        ...     entry_callback=enter,
        ...     exit_callback=exit,
        ...     has_gains=True,
        ...     max_gains=ControlGains(1.0, 0.1, 0.01, 0.5, 0.05, 0.0)
        ... )
        >>> config.has_gains
        True
    """

    entry_callback: Callable[[Any], None]
    exit_callback: Callable[[Any], None]
    has_gains: bool = False
    max_gains: Union[ControlGains, None] = None


class CONTROL_MODE_CONFIGS(NamedTuple):
    """
    Named tuple containing control mode configurations.

    Attributes:
        IDLE: Configuration for IDLE mode.
        POSITION: Configuration for POSITION mode.
        CURRENT: Configuration for CURRENT mode.
        VOLTAGE: Configuration for VOLTAGE mode.
        IMPEDANCE: Configuration for IMPEDANCE mode.
        VELOCITY: Configuration for VELOCITY mode.
        TORQUE: Configuration for TORQUE mode.

    Examples:
        >>> idle_config = ControlModeConfig(
        ...     entry_callback=lambda a: print("Idle entered"),
        ...     exit_callback=lambda a: print("Idle exited")
        ...
        >>> mode_configs = CONTROL_MODE_CONFIGS(IDLE=idle_config)
        >>> mode_configs.IDLE.entry_callback(None)
        Idle entered
    """

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
    """
    Protocol for methods that define required control modes.

    Attributes:
        _required_modes: A set of control modes in which the method is permitted.

    Examples:
        >>> class Dummy:
        ...     _required_modes = {CONTROL_MODES.IDLE}
        >>> isinstance(Dummy(), MethodWithRequiredModes)
        True
    """

    _required_modes: set[CONTROL_MODES]


def requires(*modes: CONTROL_MODES) -> Callable[[T], T]:
    """
    Decorator to specify required control modes for a method.

    Args:
        *modes: One or more control modes required for the method.

    Returns:
        The decorated method.

    Raises:
        ControlModeException: If the method is called in an invalid control mode.

    Examples:
        >>> class MyActuator(ActuatorBase):
        ...     @requires(CONTROL_MODES.POSITION)
        ...     def some_method(self, value):
        ...         print(f"Value: {value}")
    """

    def decorator(func: T) -> T:
        """
        Attach required control modes to the decorated function.

        Args:
            func (T): The function to be decorated.

        Returns:
            T: The same function with an updated `_required_modes` attribute.
        """
        if not all(isinstance(mode, CONTROL_MODES) for mode in modes):
            raise TypeError("All arguments to 'requires' must be of type CONTROL_MODES")

        if not hasattr(func, "_required_modes"):
            func._required_modes = set(modes)  # type: ignore[attr-defined]
        else:
            func._required_modes.update(modes)

        return func

    return decorator


class ActuatorBase(ABC):
    """
    Base class defining the structure and interface for an actuator.

    This abstract class provides common functionality for controlling an actuator,
    including managing control mode transitions, restricting method calls based on mode,
    and exposing actuator properties.

    Examples:
        >>> class DummyActuator(ActuatorBase):
        ...     @property
        ...     def _CONTROL_MODE_CONFIGS(self):
        ...         return CONTROL_MODE_CONFIGS()
        ...     def start(self):
        ...         print("Started")
        ...     def stop(self):
        ...         print("Stopped")
        ...     def update(self):
        ...         print("Updated")
        ...     def set_motor_voltage(self, value: float) -> None:
        ...         print(f"Motor voltage set to {value}")
        ...     def set_motor_current(self, value: float) -> None:
        ...         print(f"Motor current set to {value}")
        ...     def set_motor_position(self, value: float) -> None:
        ...         print(f"Motor position set to {value}")
        ...     def set_motor_torque(self, value: float) -> None:
        ...         print(f"Motor torque set to {value}")
        ...     def set_joint_torque(self, value: float) -> None:
        ...         print(f"Joint torque set to {value}")
        ...     def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        ...         print("Current gains set")
        ...     def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        ...         print("Position gains set")
        ...     def set_impedance_gains(self, kp: float, ki: float, kd: float, k: float, b: float, ff: float) -> None:
        ...         print("Impedance gains set")
        ...     def home(self) -> None:
        ...         print("Homed")
        ...     @property
        ...     def motor_position(self) -> float:
        ...         return 100.0
        ...     @property
        ...     def motor_velocity(self) -> float:
        ...         return 10.0
        ...     @property
        ...     def motor_voltage(self) -> float:
        ...         return 24.0
        ...     @property
        ...     def motor_current(self) -> float:
        ...         return 0.5
        ...     @property
        ...     def motor_torque(self) -> float:
        ...         return 2.0
        ...     @property
        ...     def case_temperature(self) -> float:
        ...         return 70.0
        ...     @property
        ...     def winding_temperature(self) -> float:
        ...         return 90.0
        >>> actuator = DummyActuator(
        ...     tag="act1",
        ...     gear_ratio=100,
        ...     motor_constants=MOTOR_CONSTANTS(2048, 0.02, 0.001, 0.0001, 80.0, 120.0)
        ... )
        >>> actuator.start()
        Started
    """

    # Class-level mapping of methods to their required control modes
    _METHOD_REQUIRED_MODES: ClassVar[dict[str, set[CONTROL_MODES]]] = {
        "set_motor_voltage": {CONTROL_MODES.VOLTAGE},
        "set_motor_current": {CONTROL_MODES.CURRENT},
        "set_motor_position": {CONTROL_MODES.POSITION},
        "set_motor_torque": {CONTROL_MODES.TORQUE},
        "set_joint_torque": {CONTROL_MODES.TORQUE},
        "set_current_gains": {CONTROL_MODES.CURRENT},
        "set_position_gains": {CONTROL_MODES.POSITION},
        "set_impedance_gains": {CONTROL_MODES.IMPEDANCE},
    }

    def __init__(
        self,
        tag: str,
        gear_ratio: float,
        motor_constants: MOTOR_CONSTANTS,
        frequency: int = 1000,
        offline: bool = False,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        """
        Initialize the actuator.

        Args:
            tag: A unique identifier for the actuator.
            gear_ratio: The gear ratio of the actuator.
            motor_constants: The motor constants for the actuator.
            frequency: The control frequency in Hz. Defaults to 1000.
            offline: Whether the actuator is in offline mode. Defaults to False.
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.
        """
        self._tag = tag
        self._gear_ratio = gear_ratio
        self._motor_constants = motor_constants
        self._frequency = frequency
        self._offline = offline
        self._mode = CONTROL_MODES.IDLE
        self._is_homed = False
        self._is_open = False
        self._is_streaming = False
        self._motor_zero_position = 0.0
        self._motor_position_offset = 0.0
        self._joint_zero_position = 0.0
        self._joint_position_offset = 0.0
        self._joint_direction = 1
        self._original_methods: dict[str, Callable] = {}
        self._mutated_methods: dict[str, Callable] = {}

        self._set_original_methods()
        self._set_mutated_methods()

    def __enter__(self) -> "ActuatorBase":
        """
        Context manager entry.

        Returns:
            The actuator instance.
        """
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc_value: Any, exc_traceback: Any) -> None:
        """
        Context manager exit.

        Args:
            exc_type: The type of the exception, if any.
            exc_value: The value of the exception, if any.
            exc_traceback: The traceback of the exception, if any.
        """
        self.stop()

    def _restricted_method(self, method_name: str, *args: Any, **kwargs: Any) -> None:
        """
        Check if a method can be called in the current control mode.

        Args:
            method_name: The name of the method to check.
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.

        Raises:
            ControlModeException: If the method cannot be called in the current mode.
        """
        if method_name in self._METHOD_REQUIRED_MODES:
            required_modes = self._METHOD_REQUIRED_MODES[method_name]
            if self._mode not in required_modes:
                raise ControlModeException(
                    f"Method {method_name} requires control mode(s) {required_modes}, but current mode is {self._mode}"
                )
        self._mutated_methods[method_name](*args, **kwargs)

    def _set_original_methods(self) -> None:
        """Store original method implementations."""
        for name, method in self.__class__.__dict__.items():
            if callable(method):
                self._original_methods[name] = method

    def _set_mutated_methods(self) -> None:
        """Create mutated method implementations with mode restrictions."""
        for name, method in self._original_methods.items():
            if name in self._METHOD_REQUIRED_MODES:
                self._mutated_methods[name] = partial(self._restricted_method, name)
            else:
                self._mutated_methods[name] = method

    @property
    @abstractmethod
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        """
        Get the control mode configurations.

        Returns:
            The control mode configurations.
        """
        pass

    @abstractmethod
    def start(self) -> None:
        """
        Start the actuator.
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """
        Stop the actuator.
        """
        pass

    @abstractmethod
    def update(self) -> None:
        """
        Update the actuator state.
        """
        pass

    def _get_control_mode_config(self, mode: CONTROL_MODES) -> Optional[ControlModeConfig]:
        """
        Get the configuration for a control mode.

        Args:
            mode: The control mode to get the configuration for.

        Returns:
            The control mode configuration, if it exists.
        """
        configs = self._CONTROL_MODE_CONFIGS
        return getattr(configs, mode.name, None)

    def set_control_mode(self, mode: CONTROL_MODES) -> None:
        """
        Set the control mode of the actuator.

        Args:
            mode: The control mode to set.

        Raises:
            ControlModeException: If the control mode is not supported.
        """
        if mode == self._mode:
            return

        # Exit current mode
        current_config = self._get_control_mode_config(self._mode)
        if current_config and current_config.exit_callback:
            current_config.exit_callback(self)

        # Enter new mode
        new_config = self._get_control_mode_config(mode)
        if new_config and new_config.entry_callback:
            new_config.entry_callback(self)

        self._mode = mode

    @abstractmethod
    def set_motor_voltage(self, value: float) -> None:
        """
        Set the motor voltage.

        Args:
            value: The voltage to set.
        """
        pass

    @abstractmethod
    def set_motor_current(self, value: float) -> None:
        """
        Set the motor current.

        Args:
            value: The current to set.
        """
        pass

    @abstractmethod
    def set_motor_position(self, value: float) -> None:
        """
        Set the motor position.

        Args:
            value: The position to set.
        """
        pass

    def set_output_position(self, value: float) -> None:
        """
        Set the output position.

        Args:
            value: The position to set.
        """
        motor_value = value * self._gear_ratio
        self.set_motor_position(motor_value)

    @abstractmethod
    def set_motor_torque(self, value: float) -> None:
        """
        Set the motor torque.

        Args:
            value: The torque to set.
        """
        pass

    @abstractmethod
    def set_joint_torque(self, value: float) -> None:
        """
        Set the joint torque.

        Args:
            value: The torque to set.
        """
        pass

    @abstractmethod
    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """
        Set the current control gains.

        Args:
            kp: The proportional gain.
            ki: The integral gain.
            kd: The derivative gain.
            ff: The feedforward gain.
        """
        pass

    @abstractmethod
    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """
        Set the position control gains.

        Args:
            kp: The proportional gain.
            ki: The integral gain.
            kd: The derivative gain.
            ff: The feedforward gain.
        """
        pass

    @abstractmethod
    def set_impedance_gains(self, kp: float, ki: float, kd: float, k: float, b: float, ff: float) -> None:
        """
        Set the impedance control gains.

        Args:
            kp: The proportional gain.
            ki: The integral gain.
            kd: The derivative gain.
            k: The stiffness gain.
            b: The damping gain.
            ff: The feedforward gain.
        """
        pass

    @abstractmethod
    def home(self) -> None:
        """
        Home the actuator.
        """
        pass

    def set_motor_zero_position(self, value: float) -> None:
        """
        Set the motor zero position.

        Args:
            value: The zero position to set.
        """
        self._motor_zero_position = value

    def set_motor_position_offset(self, value: float) -> None:
        """
        Set the motor position offset.

        Args:
            value: The position offset to set.
        """
        self._motor_position_offset = value

    def set_joint_zero_position(self, value: float) -> None:
        """
        Set the joint zero position.

        Args:
            value: The zero position to set.
        """
        self._joint_zero_position = value

    def set_joint_position_offset(self, value: float) -> None:
        """
        Set the joint position offset.

        Args:
            value: The position offset to set.
        """
        self._joint_position_offset = value

    def set_joint_direction(self, value: int) -> None:
        """
        Set the joint direction.

        Args:
            value: The direction to set.
        """
        if value not in [-1, 1]:
            raise ValueError("Joint direction must be either 1 or -1")
        self._joint_direction = value

    @property
    @abstractmethod
    def motor_position(self) -> float:
        """
        Get the motor position.

        Returns:
            The motor position.
        """
        pass

    @property
    def output_position(self) -> float:
        """
        Get the output position.

        Returns:
            The output position.
        """
        return self.motor_position / self._gear_ratio

    @property
    @abstractmethod
    def motor_velocity(self) -> float:
        """
        Get the motor velocity.

        Returns:
            The motor velocity.
        """
        pass

    @property
    def output_velocity(self) -> float:
        """
        Get the output velocity.

        Returns:
            The output velocity.
        """
        return self.motor_velocity / self._gear_ratio

    @property
    @abstractmethod
    def motor_voltage(self) -> float:
        """
        Get the motor voltage.

        Returns:
            The motor voltage.
        """
        pass

    @property
    @abstractmethod
    def motor_current(self) -> float:
        """
        Get the motor current.

        Returns:
            The motor current.
        """
        pass

    @property
    @abstractmethod
    def motor_torque(self) -> float:
        """
        Get the motor torque.

        Returns:
            The motor torque.
        """
        pass

    @property
    def MOTOR_CONSTANTS(self) -> MOTOR_CONSTANTS:
        """
        Get the motor constants.

        Returns:
            The motor constants.
        """
        return self._motor_constants

    @property
    def mode(self) -> CONTROL_MODES:
        """
        Get the current control mode.

        Returns:
            The current control mode.
        """
        return self._mode

    @property
    def tag(self) -> str:
        """
        Get the actuator tag.

        Returns:
            The actuator tag.
        """
        return self._tag

    @property
    def is_homed(self) -> bool:
        """
        Check if the actuator is homed.

        Returns:
            True if the actuator is homed, False otherwise.
        """
        return self._is_homed

    @property
    def frequency(self) -> int:
        """
        Get the control frequency.

        Returns:
            The control frequency in Hz.
        """
        return self._frequency

    @property
    def is_offline(self) -> bool:
        """
        Check if the actuator is offline.

        Returns:
            True if the actuator is offline, False otherwise.
        """
        return self._offline

    @property
    def gear_ratio(self) -> float:
        """
        Get the gear ratio.

        Returns:
            The gear ratio.
        """
        return self._gear_ratio

    @property
    def max_case_temperature(self) -> float:
        """
        Get the maximum case temperature.

        Returns:
            The maximum case temperature.
        """
        return self._motor_constants.MAX_CASE_TEMPERATURE

    @property
    @abstractmethod
    def case_temperature(self) -> float:
        """
        Get the case temperature.

        Returns:
            The case temperature.
        """
        pass

    @property
    @abstractmethod
    def winding_temperature(self) -> float:
        """
        Get the winding temperature.

        Returns:
            The winding temperature.
        """
        pass

    @property
    def max_winding_temperature(self) -> float:
        """
        Get the maximum winding temperature.

        Returns:
            The maximum winding temperature.
        """
        return self._motor_constants.MAX_WINDING_TEMPERATURE

    @property
    def motor_zero_position(self) -> float:
        """
        Get the motor zero position.

        Returns:
            The motor zero position.
        """
        return self._motor_zero_position

    @property
    def motor_position_offset(self) -> float:
        """
        Get the motor position offset.

        Returns:
            The motor position offset.
        """
        return self._motor_position_offset

    @property
    def joint_zero_position(self) -> float:
        """
        Get the joint zero position.

        Returns:
            The joint zero position.
        """
        return self._joint_zero_position

    @property
    def joint_position_offset(self) -> float:
        """
        Get the joint position offset.

        Returns:
            The joint position offset.
        """
        return self._joint_position_offset

    @property
    def joint_direction(self) -> int:
        """
        Get the joint direction.

        Returns:
            The joint direction.
        """
        return self._joint_direction

    @property
    def is_open(self) -> bool:
        """
        Check if the actuator is open.

        Returns:
            True if the actuator is open, False otherwise.
        """
        return self._is_open

    @property
    def is_streaming(self) -> bool:
        """
        Check if the actuator is streaming.

        Returns:
            True if the actuator is streaming, False otherwise.
        """
        return self._is_streaming
