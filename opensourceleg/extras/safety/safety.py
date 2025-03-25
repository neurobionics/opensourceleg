from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Optional

import numpy as np

__all__ = [
    "I2tLimitException",
    "SafetyDecorators",
    "SafetyManager",
    "ThermalLimitException",
    "custom_criteria",
    "is_changing",
    "is_greater_than",
    "is_less_than",
    "is_negative",
    "is_positive",
    "is_within_range",
    "is_zero",
]


class ThermalLimitException(Exception):
    def __init__(self, message: str = "Software thermal limit exceeded. Exiting.") -> None:
        self.message = message
        super().__init__(self.message)


class I2tLimitException(Exception):
    def __init__(self, message: str = "Dephy Actpack I2t limit exceeded. Exiting.") -> None:
        self.message = message
        super().__init__(self.message)


def is_changing(
    attribute_name: str,
    max_points: int = 10,
    threshold: float = 1e-6,
    proxy_attribute_name: Optional[str] = None,
) -> Callable:
    """
    Creates a decorator to check if a property's value is changing.
    If the standard deviation of the last 'max_points' values is less than 'threshold',
    the decorator will raise an error or return a proxy attribute.

    Args:
        attribute_name: Name of the attribute.
        max_points: Number of points to consider. Defaults to 10.
        threshold: Threshold for the standard deviation. Defaults to 1e-6.
        proxy_attribute_name: Name of the proxy attribute to return if the property is not changing. Defaults to None.

    Returns:
        Callable: Decorator function.

    Raises:
        ValueError: If the property is not changing and no proxy attribute is provided.

    """

    history_key = f"_{attribute_name}_history"
    proxy_key = f"_{attribute_name}_proxy"

    def decorator(func: Callable) -> Callable:
        def wrapper(instance: object, *args: Any, **kwargs: Any) -> Any:
            instance.__dict__.setdefault(history_key, deque(maxlen=max_points))

            if proxy_attribute_name is not None and getattr(instance, proxy_key):
                return getattr(instance, proxy_attribute_name)

            value = func(instance, *args, **kwargs)
            history = getattr(instance, history_key)
            history.append(value)
            if len(history) == max_points:
                current_std = np.std(list(history))
                if current_std < threshold:
                    if proxy_attribute_name is not None:
                        print(f"{attribute_name} isn't changing, returning {proxy_attribute_name}")
                        if not hasattr(instance, proxy_key):
                            setattr(instance, proxy_key, True)
                        return getattr(instance, proxy_attribute_name)
                    else:
                        raise ValueError(f"{attribute_name} is not changing")
            return value

        return wrapper

    return decorator


def is_negative(clamp: bool = False) -> Callable:
    """
    Creates a decorator to check if a property's value is negative.

    Args:
        clamp (bool): If True, the decorator will return 0 instead of raising an error. Defaults to False.

    Returns:
        Callable: Decorator function.

    Raises:
        ValueError: If the property's value is not negative.
    """

    def decorator(func: Callable) -> Callable:
        def wrapper(instance: object, *args: Any, **kwargs: Any) -> Any:
            value = func(instance, *args, **kwargs)
            if value >= 0:
                if clamp:
                    return 0
                raise ValueError("Value must be negative")
            return value

        return wrapper

    return decorator


def is_positive(clamp: bool = False) -> Callable:
    """
    Creates a decorator to check if a property's value is positive.

    Args:
        clamp (bool): If True, the decorator will return 0 instead of raising an error. Defaults to False.

    Returns:
        Callable: Decorator function.

    Raises:
        ValueError: If the property's value is not positive.
    """

    def decorator(func: Callable) -> Callable:
        def wrapper(instance: object, *args: Any, **kwargs: Any) -> Any:
            value = func(instance, *args, **kwargs)
            if value <= 0:
                if clamp:
                    return 0
                raise ValueError("Value must be positive")
            return value

        return wrapper

    return decorator


def is_zero(clamp: bool = False) -> Callable:
    """
    Creates a decorator to check if a property's value is zero.

    Args:
        clamp (bool): If True, the decorator will return 0 instead of raising an error. Defaults to False.

    Returns:
        Callable: Decorator function.

    Raises:
        ValueError: If the property's value is not zero.
    """

    def decorator(func: Callable) -> Callable:
        def wrapper(instance: object, *args: Any, **kwargs: Any) -> Any:
            value = func(instance, *args, **kwargs)
            if value != 0:
                if clamp:
                    return 0
                raise ValueError("Value must be zero")
            return value

        return wrapper

    return decorator


def is_within_range(min_value: float, max_value: float, clamp: bool = False) -> Callable:
    """
    Creates a decorator to check if a property's value is within a given range.

    Args:
        min_value: Minimum value of the range.
        max_value: Maximum value of the range.
        clamp: If True, the decorator will return the clamped value instead of raising an error. Defaults to False.

    Raises:
        ValueError: If the maximum value is less than or equal to the minimum value.

    Returns:
        Callable: Decorator function.
    """

    if max_value <= min_value:
        raise ValueError("Maximum value must be greater than minimum value of range")

    def decorator(func: Callable) -> Callable:
        def wrapper(instance: object, *args: Any, **kwargs: Any) -> Any:
            value = func(instance, *args, **kwargs)
            if value < min_value or value > max_value:
                if clamp:
                    return min(max_value, max(min_value, value))
                raise ValueError(f"Value must be within {min_value} and {max_value}")
            return value

        return wrapper

    return decorator


def is_greater_than(min_value: float, clamp: bool = False, equality: bool = False) -> Callable:
    """
    Creates a decorator to check if a property's value is greater than a given value. Gives user
    choice to implement is_greater_than_or_equal_to with equality bool

    Args:
        min_value: Minimum value to check against.
        clamp: If True, the decorator will return the clamped value instead of raising an error. Defaults to False.
        equality: If True, the decorator will check for is greater than or equal to, instead of is greater than.

    Returns:
        Callable: Decorator function.

    Raises:
        ValueError: If the property's value is less than or equal to the minimum value
    """

    def decorator(func: Callable) -> Callable:
        def wrapper(instance: object, *args: Any, **kwargs: Any) -> Any:
            value = func(instance, *args, **kwargs)
            if equality:
                if value < min_value:
                    if clamp:
                        return min_value
                    raise ValueError(f"Value must be greater than or equal to {min_value}")
            else:
                if value <= min_value:
                    if clamp:
                        return min_value
                    raise ValueError(f"Value must be greater than {min_value}")
            return value

        return wrapper

    return decorator


def is_less_than(max_value: float, clamp: bool = False, equality: bool = False) -> Callable:
    """
    Creates a decorator to check if a property's value is less than a given value. Gives user
    choice to implement is_less_than_or_equal_to with equality bool

    Args:
        max_value: Maximum value to check against.
        clamp: If True, the decorator will return the clamped value instead of raising an error. Defaults to False.
        equality: If True, the decorator will check for is less than or equal to, instead of is less than.

    Returns:
        Callable: Decorator function.

    Raises:
        ValueError: If the property's value is greater than or equal to the maximum value
    """

    def decorator(func: Callable) -> Callable:
        def wrapper(instance: object, *args: Any, **kwargs: Any) -> Any:
            value = func(instance, *args, **kwargs)
            if equality:
                if value > max_value:
                    if clamp:
                        return max_value
                    raise ValueError(f"Value must be less than or equal to {max_value}")
            else:
                if value >= max_value:
                    if clamp:
                        return max_value
                    raise ValueError(f"Value must be less than {max_value}")
            return value

        return wrapper

    return decorator


def custom_criteria(criteria: Callable) -> Callable:
    """
    Creates a decorator to check if a property's value meets a custom criteria. The criteria is a
    function that takes the property's value as an argument and returns a boolean.

    Args:
        criteria (Callable): Custom criteria function.

    Returns:
        Callable: Decorator function.

    Raises:
        ValueError: If the property's value does not meet the custom criteria.
    """

    def decorator(func: Callable) -> Callable:
        def wrapper(instance: object, *args: Any, **kwargs: Any) -> Any:
            value = func(instance, *args, **kwargs)
            if not criteria(value):
                raise ValueError("Value does not meet custom criteria")
            return value

        return wrapper

    return decorator


@dataclass
class SafetyDecorators:
    """
    Dataclass that contains all safety decorators.

    Attributes:
        is_changing: Decorator to check if a property's value is changing.
        is_negative: Decorator to check if a property's value is negative.
        is_positive: Decorator to check if a property's value is positive.
        is_zero: Decorator to check if a property's value is zero.
        is_within_range: Decorator to check if a property's value is within a given range.
        is_greater_than: Decorator to check if a property's value is greater than a given value.
        is_less_than: Decorator to check if a property's value is less than a given value.
        custom_criteria: Decorator to check if a property's value meets a custom criteria.
    """

    is_changing = is_changing
    is_negative = is_negative
    is_positive = is_positive
    is_zero = is_zero
    is_within_range = is_within_range
    is_greater_than = is_greater_than
    is_less_than = is_less_than
    custom_criteria = custom_criteria


class SafetyManager:
    """
    The SafetyManager class enables the addition of safety decorators to an object's properties,
    specifically to their getters. When the 'start' method is invoked, these decorators are applied
    to the properties of the objects stored in the 'safe_objects' dictionary. The original objects
    are then replaced with subclasses that incorporate the decorated properties.
    Invoking the 'update' method accesses the properties of the objects in the 'safe_objects' dictionary,
    thereby triggering the decorators.
    """

    def __init__(self) -> None:
        self._safe_objects: dict[object, dict[str, list[Callable]]] = {}

    def add_safety(self, instance: object, attribute: str, decorator: Callable) -> None:
        """
        Adds a safety decorator to the given object's attribute. The decorator will be applied to the property's getter.

        Args:
            instance (object): Object that contains the attribute.
            attribute (str): Name of the attribute.
            decorator (Callable): Safety decorator to be applied to the attribute.

        Raises:
            AttributeError: If the attribute does not exist in the given object.
            Warning: If the attribute is not a property. The SafetyManager only works on properties.
        """

        if not hasattr(instance, attribute):
            print(f"Error: The attribute '{attribute}' does not exist in the given object.")
            return

        original_attribute = getattr(instance.__class__, attribute, None)
        if not isinstance(original_attribute, property):
            print(
                f"Warning: The attribute '{attribute}' is not a property. The SafetyManager only works on properties."
            )
            return

        if instance in self._safe_objects:
            if attribute in self._safe_objects[instance]:
                self._safe_objects[instance][attribute].append(decorator)
            else:
                self._safe_objects[instance][attribute] = [decorator]
        else:
            self._safe_objects[instance] = {attribute: [decorator]}

    def start(self) -> None:
        """
        Applies all decorators to the properties of the objects in the safe_objects dictionary.

        Example:
        >>> safety_manager = SafetyManager()
        >>> safety_manager.add_safety(sensor, "value", SafetyDecorators.is_changing("value"))
        >>> safety_manager.add_safety(sensor, "a", SafetyDecorators.is_positive())
        >>> safety_manager.start()
        """
        for container, safe_attributes in self.safe_objects.items():
            container_subclass = type(f"{container.__class__.__name__}:SAFE", (container.__class__,), {})
            for attribute_name, attribute_decorators in safe_attributes.items():
                original_property = getattr(container.__class__, attribute_name)
                decorated_getter = original_property.fget
                for attribute_decorator in reversed(attribute_decorators):
                    decorated_getter = attribute_decorator(decorated_getter)

                setattr(
                    container_subclass,
                    attribute_name,
                    property(decorated_getter, original_property.fset, original_property.fdel),
                )

            container.__class__ = container_subclass

    def update(self) -> None:
        """
        Accesses the properties of the objects in the safe_objects dictionary, thereby triggering the decorators.

        Example:
        TODO: Add example
        """
        for container, safe_attributes in self.safe_objects.items():
            for attribute_name, _ in safe_attributes.items():
                getattr(container, attribute_name)

    @property
    def safe_objects(self) -> dict[object, dict[str, list[Callable]]]:
        return self._safe_objects


if __name__ == "__main__":

    class Sensor:
        def __init__(self, value: float) -> None:
            self._value: float = value
            self._a: float = 10

        @property
        def value(self) -> float:
            return self._value

        @value.setter
        def value(self, value: float) -> None:
            self._value = value

        @property
        def a(self) -> float:
            return self._a

        @a.setter
        def a(self, value: float) -> None:
            self._a = value

    sensor = Sensor(100)
    safety_manager = SafetyManager()

    safety_manager.add_safety(sensor, "value", SafetyDecorators.is_changing("value"))
    safety_manager.add_safety(sensor, "a", SafetyDecorators.is_positive())
    safety_manager.start()

    for _i in range(10):
        sensor.value = 1
        sensor.a = 5
        print(sensor.__dir__())
        safety_manager.update()
