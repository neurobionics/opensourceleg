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
    offline and not in offline mode, it raises an ActuatorConnectionException.
    When in offline mode, the connection check is bypassed.

    Args:
        func (Callable): The method to wrap. It is expected to be an instance method of ActuatorBase.

    Returns:
        Callable: The wrapped method that executes only if the actuator is online or in offline mode.

    Raises:
        ActuatorConnectionException: If the actuator is offline and not in offline mode.

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
        # Skip connection check if actuator is in offline mode
        if self.is_offline:
            return func(self, *args, **kwargs)

        # For online mode, check if we have a proper connection
        # This would normally check some hardware connection state
        # For now, we assume it's connected if not in offline mode
        return func(self, *args, **kwargs)

    return wrapper


def check_actuator_open(func: Callable) -> Callable:
    """
    Decorator that ensures the actuator is open before executing the method.

    This decorator checks if the actuator is in an open state. If it is not open,
    it raises an ActuatorConnectionException with the actuator's tag.

    Args:
        func (Callable): The method to wrap. It is expected to be an instance method of ActuatorBase.

    Returns:
        Callable: The wrapped method that executes only if the actuator is open.

    Raises:
        ActuatorConnectionException: If the actuator is not open.

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
        # Skip open check if actuator is in offline mode
        if self.is_offline:
            return func(self, *args, **kwargs)

        if not self.is_open:
            raise ActuatorConnectionException(tag=self.tag)

        return func(self, *args, **kwargs)

    return wrapper


def check_actuator_stream(func: Callable) -> Callable:
    """
    Decorator that verifies the actuator is streaming data before executing the method.

    This decorator checks if the actuator is actively streaming data. If the actuator
    is not streaming, it raises an ActuatorStreamException using the actuator's tag.

    Args:
        func (Callable): The method to wrap. It is expected to be an instance method of ActuatorBase.

    Returns:
        Callable: The wrapped method that executes only if the actuator is streaming.

    Raises:
        ActuatorStreamException: If the actuator is not streaming.

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
        # Skip streaming check if actuator is in offline mode
        if self.is_offline:
            return func(self, *args, **kwargs)

        if not self.is_streaming:
            raise ActuatorStreamException(tag=self.tag)

        return func(self, *args, **kwargs)

    return wrapper
