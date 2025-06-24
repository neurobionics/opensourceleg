from functools import wraps
from typing import Any, Callable, TypeVar, cast

from opensourceleg.sensors.base import SensorNotStreamingException

F = TypeVar("F", bound=Callable[..., object])


def no_op_offline(return_value: Any = None) -> Callable[[F], F]:
    """
    Parameterized decorator that makes the method a no-op if the actuator is in offline mode.
    Returns the specified return_value in offline mode.
    """

    def decorator(func: F) -> F:
        @wraps(func)
        def wrapper(self: Any, *args: Any, **kwargs: Any) -> Any:
            if getattr(self, "is_offline", False):
                return return_value
            return func(self, *args, **kwargs)

        return cast(F, wrapper)

    return decorator


def check_sensor_stream(func: Callable) -> Callable:
    """
    Decorator to ensure that a sensor is streaming before executing the decorated method.

    If the sensor is not streaming, a SensorNotStreamingException is raised.

    Args:
        func (Callable): The sensor method to be wrapped.

    Returns:
        Callable: The wrapped method that checks streaming status before execution.
    """

    @wraps(func)
    def wrapper(self: Any, *args: Any, **kwargs: Any) -> Any:
        # TODO: This could be a generic type that points to actuator, sensor, etc.
        if not self.is_streaming:
            raise SensorNotStreamingException(sensor_name=self.__repr__())
        return func(self, *args, **kwargs)

    return wrapper
