from typing import Callable, List

from collections import deque
from dataclasses import dataclass

import numpy as np


class SafetyManager:
    def __init__(self):
        self._safe_objects: dict[object, dict[str, list[Callable]]] = {}

    def add_safety(self, instance: object, attribute: str, decorator: Callable):
        """Applies a decorator to the getter of a property for a specific instance by creating or updating a subclass."""

        if instance in self._safe_objects.keys():
            if attribute in self._safe_objects[instance].keys():
                self._safe_objects[instance][attribute].append(decorator)
            else:
                self._safe_objects[instance][attribute] = [decorator]
        else:
            self._safe_objects[instance] = {attribute: [decorator]}

    def start(self):
        for container, safe_attributes in self.safe_objects.items():
            container_subclass = type(
                f"{container.__class__.__name__}:SAFE", (container.__class__,), {}
            )
            for attribute_name, attribute_decorators in safe_attributes.items():

                original_property = getattr(container.__class__, attribute_name)
                if not isinstance(original_property, property):
                    raise TypeError(
                        f"The attribute {attribute_name} is not a property."
                    )

                decorated_getter = original_property.fget
                for attribute_decorator in reversed(attribute_decorators):
                    decorated_getter = attribute_decorator(decorated_getter)

                setattr(
                    container_subclass,
                    attribute_name,
                    property(
                        decorated_getter, original_property.fset, original_property.fdel
                    ),
                )

            container.__class__ = container_subclass

    def update(self):
        for container, safe_attributes in self.safe_objects.items():
            for attribute_name, _ in safe_attributes.items():
                print(getattr(container, attribute_name))

    @property
    def safe_objects(self):
        return self._safe_objects


def is_changing(attribute_name, max_points, threshold, proxy_attribute_name=None):
    """Decorator to monitor the standard deviation of an attribute's values."""
    history_key = f"_{attribute_name}_history"
    proxy_key = f"_{attribute_name}_proxy"

    def decorator(func):
        def wrapper(instance, *args, **kwargs):
            instance.__dict__.setdefault(history_key, deque(maxlen=max_points))

            try:
                if instance.__dict__[proxy_key] is True:
                    return getattr(instance, proxy_attribute_name)
            except KeyError:
                pass

            value = func(instance, *args, **kwargs)
            history = getattr(instance, history_key)
            history.append(value)
            if len(history) == max_points:
                current_std = np.std(list(history))
                if current_std < threshold:
                    if proxy_attribute_name is not None:
                        print(
                            f"{attribute_name} isn't stable, returning {proxy_attribute_name}"
                        )
                        if not hasattr(instance, proxy_key):
                            setattr(instance, proxy_key, True)
                        return getattr(instance, proxy_attribute_name)
                    else:
                        raise ValueError(f"{attribute_name} is unstable")
            return value

        return wrapper

    return decorator


def is_negative(clamp=False):
    """Decorator to check if a property's value is negative."""

    def decorator(func):
        def wrapper(instance, *args, **kwargs):
            value = func(instance, *args, **kwargs)
            if value > 0:
                if clamp:
                    return 0
                raise ValueError("Value must be negative")
            return value

        return wrapper

    return decorator


def is_positive(clamp=False):
    """Decorator to check if a property's value is positive."""

    def decorator(func):
        def wrapper(instance, *args, **kwargs):
            value = func(instance, *args, **kwargs)
            if value < 0:
                if clamp:
                    return 0
                raise ValueError("Value must be positive")
            return value

        return wrapper

    return decorator


def is_within_range(min_value, max_value, clamp=False):
    """Decorator to check if a property's value is within a given range."""

    def decorator(func):
        def wrapper(instance, *args, **kwargs):
            value = func(instance, *args, **kwargs)
            if value < min_value or value > max_value:
                if clamp:
                    return min(max_value, max(min_value, value))
                raise ValueError(f"Value must be within {min_value} and {max_value}")
            return value

        return wrapper

    return decorator


def is_greater_than(min_value, clamp=False):
    """Decorator to check if a property's value is greater than a given value."""

    def decorator(func):
        def wrapper(instance, *args, **kwargs):
            value = func(instance, *args, **kwargs)
            if value < min_value:
                if clamp:
                    return min_value
                raise ValueError(f"Value must be greater than {min_value}")
            return value

        return wrapper

    return decorator


def is_less_than(max_value, clamp=False):
    """Decorator to check if a property's value is less than a given value."""

    def decorator(func):
        def wrapper(instance, *args, **kwargs):
            value = func(instance, *args, **kwargs)
            if value > max_value:
                if clamp:
                    return max_value
                raise ValueError(f"Value must be less than {max_value}")
            return value

        return wrapper

    return decorator


def custom_criteria(criteria):
    """Decorator to check if a property's value meets a custom criteria."""

    def decorator(func):
        def wrapper(instance, *args, **kwargs):
            value = func(instance, *args, **kwargs)
            if not criteria(value):
                raise ValueError(f"Value does not meet custom criteria")
            return value

        return wrapper

    return decorator


if __name__ == "__main__":

    class Sensor:
        def __init__(self, value):
            self._value = value
            self.a = 10

        @property
        def value(self):
            return self._value

        @value.setter
        def value(self, value):
            self._value = value

    sensor = Sensor(100)
    safety_manager = SafetyManager()

    safety_manager.add_safety(sensor, "value", is_positive())
    # safety_manager.add_safety(sensor, "a", is_negative())
    safety_manager.start()
    safety_manager.update()
    sensor.value = -10
    safety_manager.update()
