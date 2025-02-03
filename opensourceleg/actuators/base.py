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
    '''
    Class to define the motor constants.
    '''
    MOTOR_COUNT_PER_REV: float
    NM_PER_AMP: float
    NM_PER_RAD_TO_K: float
    NM_S_PER_RAD_TO_B: float
    MAX_CASE_TEMPERATURE: float
    MAX_WINDING_TEMPERATURE: float

    def __post_init__(self) -> None:
        '''
        Function to validate the motor constants.
        '''
        if any(x <= 0 for x in self.__dict__.values()):
            raise ValueError("All values in MOTOR_CONSTANTS must be non-zero and positive.")

    @property
    def RAD_PER_COUNT(self) -> float:
        '''
        Function to calculate the radians per count.

        Returns: 
            float: Radians per count.
        '''
        return 2 * np.pi / self.MOTOR_COUNT_PER_REV

    @property
    def NM_PER_MILLIAMP(self) -> float:
        """
        Function to convert NM per amp to NM per milliamp.

        Returns:
            float: NM per milliamp.
        """
        return self.NM_PER_AMP / 1000


class CONTROL_MODES(Enum):
    """
    Enum to define various control modes.
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
        _required_modes (set[CONTROL_MODES]): A set of control modes in which the method is permitted.
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
    """
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
        Initialize an actuator.

        Args:
            tag (str): A unique identifier for the actuator.
            gear_ratio (float): The gear ratio of the actuator.
            motor_constants (MOTOR_CONSTANTS): Motor constant configuration parameters.
            frequency (int, optional): Control frequency in Hz. Defaults to 1000.
            offline (bool, optional): Flag indicating if the actuator operates in offline mode. Defaults to False.
            *args (Any): Additional positional arguments.
            **kwargs (Any): Additional keyword arguments.
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
        self._motor_position_offset: float = 0.0

        self._joint_zero_position: float = 0.0
        self._joint_position_offset: float = 0.0
        self._joint_direction: int = 1

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
        """
        LOGGER.error(f"{method_name}() is not available in {self._mode.name} mode.")
        return None

    def _set_original_methods(self) -> None:
        """
        Store the original methods that require specific control modes.

        Iterates through known control mode methods and saves those that have
        a `_required_modes` attribute to allow mode-based restrictions.
        """
        for method_name in CONTROL_MODE_METHODS:
            try:
                method = getattr(self, method_name)
                if callable(method) and hasattr(method, "_required_modes"):
                    self._original_methods[method_name] = method
            except AttributeError:
                LOGGER.debug(msg=f"[{self.tag}] {method_name}() is not implemented in {self.tag}.")

    def _set_mutated_methods(self) -> None:
        """
        Update actuator methods based on the current control mode.

        For each method stored in `_original_methods`, if the current control mode
        is permitted, the original method is used; otherwise, a restricted version
        that logs an error is assigned.
        """
        for method_name, method in self._original_methods.items():
            if self._mode in method._required_modes:
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
        """
        pass

    @abstractmethod
    def start(self) -> None:
        """
        Start the actuator.

        Must be implemented by subclasses to initialize and activate the actuator.
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """
        Stop the actuator.

        Must be implemented by subclasses to safely deactivate the actuator.
        """
        pass

    @abstractmethod
    def update(self) -> None:
        """
        Update the actuator's state.

        Must be implemented by subclasses to refresh or recalculate state values.
        """
        pass

    def _get_control_mode_config(self, mode: CONTROL_MODES) -> Optional[ControlModeConfig]:
        """
        Retrieve the control mode configuration for a specified mode.

        Args:
            mode (CONTROL_MODES): The control mode for which to retrieve the configuration.

        Returns:
            Optional[ControlModeConfig]: The configuration if it exists; otherwise, None.
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
    @requires(CONTROL_MODES.VOLTAGE)
    def set_motor_voltage(self, value: float) -> None:
        """
        Set the motor voltage.

        Args:
            value (float): The voltage value to be applied to the motor.

        Must be implemented by subclasses.
        """
        pass

    @abstractmethod
    @requires(CONTROL_MODES.CURRENT)
    def set_motor_current(self, value: float) -> None:
        """
        Set the motor current.

        Args:
            value (float): The current value to be applied to the motor.

        Must be implemented by subclasses.
        """
        pass

    @abstractmethod
    @requires(CONTROL_MODES.POSITION)
    def set_motor_position(self, value: float) -> None:
        """
        Set the motor position.

        Args:
            value (float): The target motor position in radians.

        Must be implemented by subclasses.
        """
        pass

    @requires(
        CONTROL_MODES.POSITION
    )  # This needs to be tested as set_motor_position is already decorated with requires
    def set_output_position(self, value: float) -> None:
        """
        Set the output position of the actuator.

        Converts the desired output position (in radians) to a motor position by
        applying the gear ratio, then delegates to `set_motor_position`.

        Args:
            value (float): The desired output position in radians.
        """
        self.set_motor_position(value=value * self.gear_ratio)

    @abstractmethod
    @requires(CONTROL_MODES.TORQUE)
    def set_motor_torque(self, value: float) -> None:
        """
        Set the motor torque.

        Args:
            value (float): The torque value to be applied to the motor.

        Must be implemented by subclasses.
        """
        pass

    @abstractmethod
    @requires(CONTROL_MODES.TORQUE)
    def set_joint_torque(self, value: float) -> None:
        """
        Set the joint torque.

        Args:
            value (float): The torque value to be applied to the joint.

        Must be implemented by subclasses.
        """
        pass

    @abstractmethod
    @requires(CONTROL_MODES.CURRENT)
    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """
        Set the current control gains.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            ff (float): Feed-forward gain.

        Must be implemented by subclasses.
        """
        pass

    @abstractmethod
    @requires(CONTROL_MODES.POSITION)
    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """
        Set the position control gains.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            ff (float): Feed-forward gain.

        Must be implemented by subclasses.
        """
        pass

    @abstractmethod
    @requires(CONTROL_MODES.IMPEDANCE)
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
        """
        pass

    @abstractmethod
    def home(self) -> None:
        """
        Home the actuator.

        Aligns the actuator to a known reference position.
        Must be implemented by subclasses.
        """
        pass

    def set_motor_zero_position(self, value: float) -> None:
        """
        Set the motor zero position.

        Args:
            value (float): The motor zero position in radians.
        """
        self._motor_zero_position = value

    def set_motor_position_offset(self, value: float) -> None:
        """
        Set the motor position offset.

        Args:
            value (float): The motor position offset in radians.
        """
        self._motor_position_offset = value

    def set_joint_zero_position(self, value: float) -> None:
        """
        Set the joint zero position.

        Args:
            value (float): The joint zero position in radians.
        """
        self._joint_zero_position = value

    def set_joint_position_offset(self, value: float) -> None:
        """
        Set the joint position offset.

        Args:
            value (float): The joint position offset in radians.
        """
        self._joint_position_offset = value

    def set_joint_direction(self, value: int) -> None:
        """
        Set the joint direction.

        Args:
            value (int): The joint direction, expected to be either 1 or -1.
        """
        self._joint_direction = value

    @property
    @abstractmethod
    def motor_position(self) -> float:
        """
        Get the motor position.

        Returns:
            float: The current motor position in radians.

        Must be implemented by subclasses.
        """
        pass


    @property
    def output_position(self) -> float:
        """
        Get the output position.

        Returns:
            float: The output position in radians, calculated by dividing the motor
                    position by the gear ratio. Note that this does not account for SEA compliance.
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
        """
        pass

    @property
    def output_velocity(self) -> float:
        """
        Get the output velocity.

        Returns:
            float: The output velocity in radians per second, calculated by dividing the motor
                   velocity by the gear ratio. Note that this does not account for SEA compliance.
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
        """
        pass

    @property
    def MOTOR_CONSTANTS(self) -> MOTOR_CONSTANTS:
        """
        Get the motor constants configuration.

        Returns:
            MOTOR_CONSTANTS: The motor constants.
        """
        return self._MOTOR_CONSTANTS

    @property
    def mode(self) -> CONTROL_MODES:
        """
        Get the current control mode.

        Returns:
            CONTROL_MODES: The actuator's current control mode.
        """
        return self._mode

    @property
    def tag(self) -> str:
        """
        Get the actuator tag.

        Returns:
            str: The unique identifier for the actuator.
        """
        return self._tag

    @property
    def is_homed(self) -> bool:
        """
        Check if the actuator has been homed.

        Returns:
            bool: True if the actuator is homed; otherwise, False.
        """
        return self._is_homed

    @property
    def frequency(self) -> int:
        """
        Get the actuator's control frequency.

        Returns:
            int: The control frequency in Hz.
        """
        return self._frequency

    @property
    def is_offline(self) -> bool:
        """
        Check if the actuator is in offline mode.

        Returns:
            bool: True if offline; otherwise, False.
        """
        return self._is_offline

    @property
    def gear_ratio(self) -> float:
        """
        Get the gear ratio.

        Returns:
            float: The gear ratio of the actuator.
        """
        return self._gear_ratio

    @property
    def max_case_temperature(self) -> float:
        """
        Get the maximum allowed case temperature.

        Returns:
            float: The maximum case temperature defined in motor constants.
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
        """
        pass

    @property
    def max_winding_temperature(self) -> float:
        """
        Get the maximum allowed winding temperature.

        Returns:
            float: The maximum winding temperature defined in motor constants.
        """
        return self._MOTOR_CONSTANTS.MAX_WINDING_TEMPERATURE

    @property
    def motor_zero_position(self) -> float:
        """
        Get the motor zero position.

        Returns:
            float: The motor zero position in radians.
        """
        return self._motor_zero_position

    @property
    def motor_position_offset(self) -> float:
        """
        Get the motor position offset.

        Returns:
            float: The motor position offset in radians.
        """
        return self._motor_position_offset

    @property
    def joint_zero_position(self) -> float:
        """
        Get the joint zero position.

        Returns:
            float: The joint zero position in radians.
        """
        return self._joint_zero_position

    @property
    def joint_position_offset(self) -> float:
        """
        Get the joint position offset.

        Returns:
            float: The joint position offset in radians.
        """
        return self._joint_position_offset

    @property
    def joint_direction(self) -> int:
        """
        Get the joint direction.

        Returns:
            int: The joint direction (1 or -1).
        """
        return self._joint_direction

    @property
    def is_open(self) -> bool:
        """
        Check if the actuator is open.

        Returns:
            bool: True if open; otherwise, False.
        """
        return self._is_open

    @property
    def is_streaming(self) -> bool:
        """
        Check if the actuator is streaming data.

        Returns:
            bool: True if streaming; otherwise, False.
        """
        return self._is_streaming