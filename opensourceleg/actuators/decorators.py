from functools import wraps
from typing import Any, Callable

from opensourceleg.actuators.base import ActuatorBase
from opensourceleg.logging.exceptions import (
    ActuatorConnectionException,
    ActuatorStreamException,
)


def check_actuator_connection(func: Callable) -> Callable:
    """
    Decorator that verifies the actuator is connected before executing the method.

    This decorator checks if the actuator is operating online. If the actuator is
    offline, it allows the method to run as a no-op (returns None) instead of raising
    an ActuatorConnectionException. This enables offline mode for testing scripts
    without hardware.

    Args:
        func (Callable): The method to wrap. It is expected to be an instance method of ActuatorBase.

    Returns:
        Callable: The wrapped method that executes only if the actuator is online, or is a no-op if offline.

    Raises:
        ActuatorConnectionException: If the actuator is offline (only in legacy behavior; now a no-op).

    Examples:
        >>> class MyActuator(ActuatorBase):
        ...     @check_actuator_connection
        ...     def my_method(self):
        ...         return "Hello, world!"
        ...
        >>> actuator = MyActuator()
        >>> actuator.my_method()
    """

    @wraps(func)
    def wrapper(self: ActuatorBase, *args: Any, **kwargs: Any) -> Any:
        if getattr(self, "is_offline", False):
            # In offline mode, do nothing (no-op)
            return None
        if not hasattr(self, "is_offline") or not hasattr(self, "tag"):
            raise AttributeError("Object missing required attributes for actuator decorators.")
        return func(self, *args, **kwargs)
    return wrapper


def check_actuator_open(func: Callable) -> Callable:
    """
    Decorator that ensures the actuator is open before executing the method.

    This decorator checks if the actuator is in an open state. If it is not open,
    it raises an ActuatorConnectionException with the actuator's tag. In offline mode,
    this decorator allows the method to run as a no-op (returns None) for testing.

    Args:
        func (Callable): The method to wrap. It is expected to be an instance method of ActuatorBase.

    Returns:
        Callable: The wrapped method that executes only if the actuator is open, or is a no-op if offline.

    Raises:
        ActuatorConnectionException: If the actuator is not open (only in online mode).

    Example:
        >>> class MyActuator(ActuatorBase):
        ...     @check_actuator_open
        ...     def my_method(self):
        ...         return "Hello, world!"
        ...
        >>> actuator = MyActuator()
        >>> actuator.my_method()
    """

    @wraps(func)
    def wrapper(self: ActuatorBase, *args: Any, **kwargs: Any) -> Any:
        if getattr(self, "is_offline", False):
            # In offline mode, do nothing (no-op)
            return None
        if not self.is_open:
            raise ActuatorConnectionException(tag=self.tag)
        return func(self, *args, **kwargs)
    return wrapper


def check_actuator_stream(func: Callable) -> Callable:
    """
    Decorator that verifies the actuator is streaming data before executing the method.

    This decorator checks if the actuator is actively streaming data. If the actuator
    is not streaming, it raises an ActuatorStreamException using the actuator's tag. In offline mode,
    this decorator allows the method to run as a no-op (returns None) for testing.

    Args:
        func (Callable): The method to wrap. It is expected to be an instance method of ActuatorBase.

    Returns:
        Callable: The wrapped method that executes only if the actuator is streaming, or is a no-op if offline.

    Raises:
        ActuatorStreamException: If the actuator is not streaming (only in online mode).

    Example:
        >>> class MyActuator(ActuatorBase):
        ...     @check_actuator_stream
        ...     def my_method(self):
        ...         return "Hello, world!"
        ...
        >>> actuator = MyActuator()
        >>> actuator.my_method()
    """

    @wraps(func)
    def wrapper(self: ActuatorBase, *args: Any, **kwargs: Any) -> Any:
        if getattr(self, "is_offline", False):
            # In offline mode, do nothing (no-op)
            return None
        if not self.is_streaming:
            raise ActuatorStreamException(tag=self.tag)
        return func(self, *args, **kwargs)
    return wrapper
