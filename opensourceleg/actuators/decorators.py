from functools import wraps
from typing import Any, Callable

from opensourceleg.actuators.base import ActuatorBase
from opensourceleg.logging.exceptions import (
    ActuatorConnectionException,
    ActuatorStreamException,
)


def check_actuator_connection(func: Callable) -> Callable:
    @wraps(func)
    def wrapper(self: ActuatorBase, *args: Any, **kwargs: Any) -> Any:
        if self.is_offline:
            raise ActuatorConnectionException(tag=self.tag)

        return func(self, *args, **kwargs)

    return wrapper


def check_actuator_open(func: Callable) -> Callable:
    @wraps(func)
    def wrapper(self: ActuatorBase, *args: Any, **kwargs: Any) -> Any:
        if not self.is_open:
            raise ActuatorConnectionException(tag=self.tag)

        return func(self, *args, **kwargs)

    return wrapper


def check_actuator_stream(func: Callable) -> Callable:
    @wraps(func)
    def wrapper(self: ActuatorBase, *args: Any, **kwargs: Any) -> Any:
        if not self.is_streaming:
            raise ActuatorStreamException(tag=self.tag)

        return func(self, *args, **kwargs)

    return wrapper
