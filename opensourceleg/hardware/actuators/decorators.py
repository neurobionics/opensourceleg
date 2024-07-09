from functools import wraps

from opensourceleg.actuators.exceptions import (
    ActuatorConnectionException,
    ActuatorStreamException,
)


def check_actuator_connection(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if self.is_offline:
            raise ActuatorConnectionException(tag=self.tag)

        return func(self, *args, **kwargs)

    return wrapper


def check_actuator_open(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.is_open:
            raise ActuatorConnectionException(tag=self.tag)

        return func(self, *args, **kwargs)

    return wrapper


def check_actuator_stream(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.is_streaming:
            raise ActuatorStreamException(tag=self.tag)

        return func(self, *args, **kwargs)

    return wrapper
