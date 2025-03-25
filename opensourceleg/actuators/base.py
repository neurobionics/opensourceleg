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
            float: Radians per count.

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
            float: NM per milliamp.

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
        entry_callback (Callable[[Any], None]): Callback to execute when entering this mode.
        exit_callback (Callable[[Any], None]): Callback to execute when exiting this mode.
        has_gains (bool): Indicates if the control mode utilizes control gains.
        max_gains (Union[ControlGains, None]): The maximum allowable control gains (if applicable).

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
        IDLE (Optional[ControlModeConfig]): Configuration for IDLE mode.
        POSITION (Optional[ControlModeConfig]): Configuration for POSITION mode.
        CURRENT (Optional[ControlModeConfig]): Configuration for CURRENT mode.
        VOLTAGE (Optional[ControlModeConfig]): Configuration for VOLTAGE mode.
        IMPEDANCE (Optional[ControlModeConfig]): Configuration for IMPEDANCE mode.
        VELOCITY (Optional[ControlModeConfig]): Configuration for VELOCITY mode.
        TORQUE (Optional[ControlModeConfig]): Configuration for TORQUE mode.

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
        _required_modes (set[CONTROL_MODES]): A set of control modes in which the method is permitted.

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

    The decorator attaches a set of required modes to the function,
    ensuring that the function is only active in the specified control modes.

    Args:
        *modes (CONTROL_MODES): One or more control modes required for the method.

    Raises:
        TypeError: If any argument is not an instance of CONTROL_MODES.

    Returns:
        Callable[[T], T]: The decorated function with an attached `_required_modes` attribute.

    Examples:
        >>> @requires(CONTROL_MODES.POSITION, CONTROL_MODES.TORQUE)
        ... def some_method(self, value):
        ...     return value
        >>> some_method._required_modes  # May output: {<CONTROL_MODES.POSITION: 0>, <CONTROL_MODES.TORQUE: 5>}
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
        ...     def set_output_torque(self, value: float) -> None:
        ...         print(f"Output torque set to {value}")
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
        "set_motor_position": {CONTROL_MODES.POSITION, CONTROL_MODES.IMPEDANCE},
        "set_output_position": {CONTROL_MODES.POSITION, CONTROL_MODES.IMPEDANCE},
        "set_motor_impedance": {CONTROL_MODES.IMPEDANCE},
        "set_output_impedance": {CONTROL_MODES.IMPEDANCE},
        "set_motor_torque": {CONTROL_MODES.CURRENT, CONTROL_MODES.TORQUE},
        "set_output_torque": {CONTROL_MODES.CURRENT, CONTROL_MODES.TORQUE},
        "set_current_gains": {CONTROL_MODES.CURRENT, CONTROL_MODES.TORQUE},
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
        **kwargs: Any,
    ) -> None:
        """
        Initialize an actuator.

        Args:
            tag (str): A unique identifier for the actuator.
            gear_ratio (float): The gear ratio of the actuator.
            motor_constants (MOTOR_CONSTANTS): Motor constant configuration parameters.
            frequency (int, optional): Control frequency in Hz. Defaults to 1000.
            offline (bool, optional): Flag indicating if the actuator operates in offline mode. Defaults to False.
            **kwargs (Any): Additional keyword arguments.

        Examples:
            >>> actuator = DummyActuator(
            ...     tag="act1",
            ...     gear_ratio=100,
            ...     motor_constants=MOTOR_CONSTANTS(2048, 0.02, 0.001, 0.0001, 80.0, 120.0)
            ... )
        """
        self._MOTOR_CONSTANTS: MOTOR_CONSTANTS = motor_constants
        self._gear_ratio: float = gear_ratio
        self._tag: str = tag
        self._frequency: int = frequency
        self._data: Any = None
        self._is_offline: bool = offline
        self._is_homed: bool = False

        self._mode: CONTROL_MODES = CONTROL_MODES.IDLE

        self._motor_zero_position: float = 0.0

        self._is_open: bool = False
        self._is_streaming: bool = False

        self._original_methods: dict[str, MethodWithRequiredModes] = {}

        self._set_original_methods()
        self._set_mutated_methods()

    def __enter__(self) -> "ActuatorBase":
        """
        Enter the runtime context related to this actuator.

        Starts the actuator and returns the instance.

        Returns:
            ActuatorBase: The actuator instance.

        Examples:
            >>> with actuator as a:
            ...     print("Inside context")
            Started
            Inside context
            Stopped
        """
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc_value: Any, exc_traceback: Any) -> None:
        """
        Exit the runtime context and stop the actuator.

        Args:
            exc_type (Any): Exception type, if any.
            exc_value (Any): Exception value, if any.
            exc_traceback (Any): Exception traceback, if any.

        Examples:
            >>> try:
            ...     with actuator:
            ...         raise ValueError("Test error")
            ... except ValueError:
            ...     pass  # actuator.stop() was automatically called
        """
        self.stop()

    def _restricted_method(self, method_name: str, *args: Any, **kwargs: Any) -> None:
        """
        Fallback method for restricted operations.

        Logs an error indicating that the requested method is not available
        in the current control mode.

        Args:
            method_name (str): Name of the restricted method.
            *args (Any): Positional arguments passed to the method.
            **kwargs (Any): Keyword arguments passed to the method.

        Examples:
            >>> actuator._restricted_method("set_motor_voltage")
            # (Logs an error message and returns None)
        """
        raise ControlModeException(tag=self._tag, attribute=method_name, mode=self._mode.name)

    def _set_original_methods(self) -> None:
        """
        Store the original methods that require specific control modes.

        Uses a class-level mapping of methods to their required control modes
        to ensure proper inheritance of restrictions in derived classes.

        Examples:
            >>> print(actuator._original_methods)  # Dictionary of method names to methods
        """
        # Get the method-to-required-modes mapping for this class
        method_modes_map = getattr(self.__class__, "_METHOD_REQUIRED_MODES", {})

        for method_name, _required_modes in method_modes_map.items():
            try:
                method = getattr(self, method_name)
                if callable(method):
                    self._original_methods[method_name] = method
                    # LOGGER.debug(
                    #     msg=f"[{self.tag}] {method_name}() is available in modes: "
                    #     f"{[mode.name for mode in required_modes]}"
                    # )
            except AttributeError:
                LOGGER.debug(msg=f"[{self.tag}] {method_name}() is not implemented in {self.__class__.__name__}.")

    def _set_mutated_methods(self) -> None:
        """
        Update actuator methods based on the current control mode.

        For each method stored in `_original_methods`, if the current control mode
        is permitted, the original method is used; otherwise, a restricted version
        that logs an error is assigned.

        Examples:
            >>> actuator._set_mutated_methods()
        """
        for method_name, method in self._original_methods.items():
            if self._mode in self._METHOD_REQUIRED_MODES[method_name]:
                setattr(self, method_name, method)
            else:
                setattr(self, method_name, partial(self._restricted_method, method_name))

    @property
    @abstractmethod
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        """
        Abstract property to obtain control mode configurations.

        Returns:
            CONTROL_MODE_CONFIGS: The configuration settings for each control mode.

        Examples:
            >>> config = actuator._CONTROL_MODE_CONFIGS  # Implemented in subclass
        """
        pass

    @abstractmethod
    def start(self) -> None:
        """
        Start the actuator.

        Must be implemented by subclasses to initialize and activate the actuator.

        Examples:
            >>> actuator.start()
            Started
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """
        Stop the actuator.

        Must be implemented by subclasses to safely deactivate the actuator.

        Examples:
            >>> actuator.stop()
            Stopped
        """
        pass

    @abstractmethod
    def update(self) -> None:
        """
        Update the actuator's state.

        Must be implemented by subclasses to refresh or recalculate state values.

        Examples:
            >>> actuator.update()
            Updated
        """
        pass

    def _get_control_mode_config(self, mode: CONTROL_MODES) -> Optional[ControlModeConfig]:
        """
        Retrieve the control mode configuration for a specified mode.

        Args:
            mode (CONTROL_MODES): The control mode for which to retrieve the configuration.

        Returns:
            Optional[ControlModeConfig]: The configuration if it exists; otherwise, None.

        Examples:
            >>> config = actuator._get_control_mode_config(CONTROL_MODES.IDLE)
            >>> print(config)
            None
        """
        return cast(
            Optional[ControlModeConfig],
            getattr(self._CONTROL_MODE_CONFIGS, mode.name),
        )

    def set_control_mode(self, mode: CONTROL_MODES) -> None:
        """
        Set the actuator's control mode.

        If the mode is changing, the exit callback for the current mode and
        the entry callback for the new mode are executed, and methods are updated
        to reflect any restrictions imposed by the new mode.

        Args:
            mode (CONTROL_MODES): The new control mode to be set.

        Examples:
            >>> actuator.set_control_mode(CONTROL_MODES.POSITION)
        """
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
    def set_motor_voltage(self, value: float) -> None:
        """
        Set the motor voltage.

        Args:
            value (float): The voltage value to be applied to the motor.

        Must be implemented by subclasses.

        Examples:
            >>> actuator.set_motor_voltage(12.0)
        """
        pass

    @abstractmethod
    def set_motor_current(self, value: float) -> None:
        """
        Set the motor current.

        Args:
            value (float): The current value to be applied to the motor.

        Must be implemented by subclasses.

        Examples:
            >>> actuator.set_motor_current(1.5)
        """
        pass

    @abstractmethod
    def set_motor_position(self, value: float) -> None:
        """
        Set the motor position.

        Args:
            value (float): The target motor position in radians.

        Must be implemented by subclasses.

        Examples:
            >>> actuator.set_motor_position(0.5)
        """
        pass

    def set_output_position(self, value: float) -> None:
        """
        Set the output position of the actuator.

        Converts the desired output position (in radians) to a motor position by
        applying the gear ratio, then delegates to `set_motor_position`.

        Args:
            value (float): The desired output position in radians.

        Examples:
            >>> # Assuming gear_ratio is 100, this will set the motor position to 100 * value.
            >>> actuator.set_motor_position = lambda value: print(f"Motor position set to {value}")
            >>> actuator.set_output_position(1.0)
            Motor position set to 100.0
        """
        self.set_motor_position(value=value * self.gear_ratio)

    @abstractmethod
    def set_motor_torque(self, value: float) -> None:
        """
        Set the motor torque.

        Args:
            value (float): The torque value to be applied to the motor.

        Must be implemented by subclasses.

        Examples:
            >>> actuator.set_motor_torque(5.0)
        """
        pass

    @abstractmethod
    def set_output_torque(self, value: float) -> None:
        """
        Set the output torque.

        Args:
            value (float): The torque value to be applied to the joint.

        Must be implemented by subclasses.

        Examples:
            >>> actuator.set_output_torque(5.0)
        """
        pass

    @abstractmethod
    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """
        Set the current control gains.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            ff (float): Feed-forward gain.

        Must be implemented by subclasses.

        Examples:
            >>> actuator.set_current_gains(1.0, 0.1, 0.01, 0.0)
        """
        pass

    @abstractmethod
    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """
        Set the position control gains.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            ff (float): Feed-forward gain.

        Must be implemented by subclasses.

        Examples:
            >>> actuator.set_position_gains(1.0, 0.1, 0.01, 0.0)
        """
        pass

    @abstractmethod
    def set_impedance_gains(self, kp: float, ki: float, kd: float, k: float, b: float, ff: float) -> None:
        """
        Set the impedance control gains.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            k (float): Stiffness coefficient.
            b (float): Damping coefficient.
            ff (float): Feed-forward gain.

        Must be implemented by subclasses.

        Examples:
            >>> actuator.set_impedance_gains(1.0, 0.1, 0.01, 0.5, 0.05, 0.0)
        """
        pass

    @abstractmethod
    def home(
        self,
        homing_voltage: int = 2000,
        homing_frequency: Optional[int] = None,
        homing_direction: int = -1,
        output_position_offset: float = 0.0,
        current_threshold: int = 5000,
        velocity_threshold: float = 0.001,
    ) -> None:
        """
        Home the actuator.

        Aligns the actuator to a known reference position.
        Must be implemented by subclasses.

        Args:
            homing_voltage (int): Voltage to use for homing.
            homing_frequency (Optional[int]): Frequency to use for homing.
            homing_direction (int): Direction to move the actuator during homing.
            output_position_offset (float): Offset to add to the output position.
            current_threshold (int): Current threshold to stop homing.
            velocity_threshold (float): Velocity threshold to stop homing.

        Examples:
            >>> actuator.home()
            Homed
        """
        pass

    def set_motor_zero_position(self, value: float) -> None:
        """
        Set the motor zero position.

        Args:
            value (float): The motor zero position in radians.

        Examples:
            >>> actuator.set_motor_zero_position(0.0)
            >>> actuator.motor_zero_position
            0.0
        """
        self._motor_zero_position = value

    @property
    @abstractmethod
    def motor_position(self) -> float:
        """
        Get the motor position.

        Returns:
            float: The current motor position in radians.

        Must be implemented by subclasses.

        Examples:
            >>> pos = actuator.motor_position
            >>> print(pos)
            100.0
        """
        pass

    @property
    def output_position(self) -> float:
        """
        Get the output position.

        Returns:
            float: The output position in radians, calculated by dividing the motor
                   position by the gear ratio. Note that this does not account for SEA compliance.

        Examples:
            >>> # If motor_position is 100.0 and gear_ratio is 100, output_position will be 1.0
            >>> actuator.output_position
            1.0
        """
        return self.motor_position / self.gear_ratio

    @property
    @abstractmethod
    def motor_velocity(self) -> float:
        """
        Get the motor velocity.

        Returns:
            float: The current motor velocity in radians per second.

        Must be implemented by subclasses.

        Examples:
            >>> velocity = actuator.motor_velocity
            >>> print(velocity)
            10.0
        """
        pass

    @property
    def output_velocity(self) -> float:
        """
        Get the output velocity.

        Returns:
            float: The output velocity in radians per second, calculated by dividing the motor
                   velocity by the gear ratio. Note that this does not account for SEA compliance.

        Examples:
            >>> # If motor_velocity is 10.0 and gear_ratio is 100, output_velocity will be 0.1
            >>> actuator.output_velocity
            0.1
        """
        return self.motor_velocity / self.gear_ratio

    @property
    @abstractmethod
    def motor_voltage(self) -> float:
        """
        Get the motor voltage.

        Returns:
            float: The current motor voltage.

        Must be implemented by subclasses.

        Examples:
            >>> voltage = actuator.motor_voltage
            >>> print(voltage)
            24.0
        """
        pass

    @property
    @abstractmethod
    def motor_current(self) -> float:
        """
        Get the motor current.

        Returns:
            float: The current motor current.

        Must be implemented by subclasses.

        Examples:
            >>> current = actuator.motor_current
            >>> print(current)
            0.5
        """
        pass

    @property
    @abstractmethod
    def motor_torque(self) -> float:
        """
        Get the motor torque.

        Returns:
            float: The current motor torque.

        Must be implemented by subclasses.

        Examples:
            >>> torque = actuator.motor_torque
            >>> print(torque)
            2.0
        """
        pass

    @property
    def MOTOR_CONSTANTS(self) -> MOTOR_CONSTANTS:
        """
        Get the motor constants configuration.

        Returns:
            MOTOR_CONSTANTS: The motor constants.

        Examples:
            >>> constants = actuator.MOTOR_CONSTANTS
            >>> constants.MAX_CASE_TEMPERATURE
            80.0
        """
        return self._MOTOR_CONSTANTS

    @property
    def mode(self) -> CONTROL_MODES:
        """
        Get the current control mode.

        Returns:
            CONTROL_MODES: The actuator's current control mode.

        Examples:
            >>> actuator.mode
            <CONTROL_MODES.IDLE: -1>
        """
        return self._mode

    @property
    def tag(self) -> str:
        """
        Get the actuator tag.

        Returns:
            str: The unique identifier for the actuator.

        Examples:
            >>> actuator.tag
            "act1"
        """
        return self._tag

    @property
    def is_homed(self) -> bool:
        """
        Check if the actuator has been homed.

        Returns:
            bool: True if the actuator is homed; otherwise, False.

        Examples:
            >>> actuator.is_homed
            False
        """
        return self._is_homed

    @property
    def frequency(self) -> int:
        """
        Get the actuator's control frequency.

        Returns:
            int: The control frequency in Hz.

        Examples:
            >>> actuator.frequency
            1000
        """
        return self._frequency

    @property
    def is_offline(self) -> bool:
        """
        Check if the actuator is in offline mode.

        Returns:
            bool: True if offline; otherwise, False.

        Examples:
            >>> actuator.is_offline
            False
        """
        return self._is_offline

    @property
    def gear_ratio(self) -> float:
        """
        Get the gear ratio.

        Returns:
            float: The gear ratio of the actuator.

        Examples:
            >>> actuator.gear_ratio
            100
        """
        return self._gear_ratio

    @property
    def max_case_temperature(self) -> float:
        """
        Get the maximum allowed case temperature.

        Returns:
            float: The maximum case temperature defined in motor constants.

        Examples:
            >>> actuator.max_case_temperature
            80.0
        """
        return self._MOTOR_CONSTANTS.MAX_CASE_TEMPERATURE

    @property
    @abstractmethod
    def case_temperature(self) -> float:
        """
        Get the current case temperature.

        Returns:
            float: The current case temperature.

        Must be implemented by subclasses.

        Examples:
            >>> temp = actuator.case_temperature
            >>> print(temp)
            70.0
        """
        pass

    @property
    @abstractmethod
    def winding_temperature(self) -> float:
        """
        Get the current winding temperature.

        Returns:
            float: The current winding temperature.

        Must be implemented by subclasses.

        Examples:
            >>> temp = actuator.winding_temperature
            >>> print(temp)
            90.0
        """
        pass

    @property
    def max_winding_temperature(self) -> float:
        """
        Get the maximum allowed winding temperature.

        Returns:
            float: The maximum winding temperature defined in motor constants.

        Examples:
            >>> actuator.max_winding_temperature
            120.0
        """
        return self._MOTOR_CONSTANTS.MAX_WINDING_TEMPERATURE

    @property
    def motor_zero_position(self) -> float:
        """
        Get the motor zero position.

        Returns:
            float: The motor zero position in radians.

        Examples:
            >>> actuator.set_motor_zero_position(0.0)
            >>> actuator.motor_zero_position
            0.0
        """
        return self._motor_zero_position

    @property
    def is_open(self) -> bool:
        """
        Check if the actuator is open.

        Returns:
            bool: True if open; otherwise, False.

        Examples:
            >>> actuator._is_open = True
            >>> actuator.is_open
            True
        """
        return self._is_open

    @property
    def is_streaming(self) -> bool:
        """
        Check if the actuator is streaming data.

        Returns:
            bool: True if streaming; otherwise, False.

        Examples:
            >>> actuator._is_streaming = True
            >>> actuator.is_streaming
            True
        """
        return self._is_streaming
