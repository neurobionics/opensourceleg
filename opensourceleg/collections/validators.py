"""
Module for attribute validation using descriptor classes.

This module provides an abstract base class `Validator` for creating attribute validators
that can be used as descriptors. It also includes a concrete implementation, `Number`,
which validates that a given value is a number (int or float) and optionally enforces
minimum and/or maximum limits.
"""

from abc import ABC, abstractmethod
from typing import Any, Optional, Union


class Validator(ABC):
    """
    Abstract base class for attribute validators.

    This descriptor class implements the __set_name__, __get__, and __set__ methods to
    automatically manage a private attribute name and to perform validation when setting
    an attribute's value. Subclasses must implement the validate() method.
    """

    def __set_name__(self, owner: Any, name: str) -> None:
        """
        Automatically called to set the name of the attribute.

        This method is invoked at class creation time to assign a private name
        to the attribute being managed by the validator.

        Args:
            owner (Any): The owner class where the descriptor is defined.
            name (str): The name of the attribute.
        """
        self.private_name = f"_{name}"

    def __get__(self, instance: Any, objtype: Any = None) -> Any:
        """
        Retrieve the value of the managed attribute.

        Args:
            instance (Any): The instance from which the attribute is accessed.
            objtype (Any, optional): The type of the owner class. Defaults to None.

        Returns:
            Any: The current value of the attribute.
        """
        return getattr(instance, self.private_name)

    def __set__(self, instance: Any, value: Any) -> None:
        """
        Set the value of the managed attribute after validating it.

        The method calls the validate() function to ensure that the provided value meets
        the criteria defined by the validator. If valid, the value is stored under the
        private attribute name.

        Args:
            instance (Any): The instance on which the attribute is being set.
            value (Any): The new value to set.

        Raises:
            Any: Any exception raised by the validate() method.
        """
        self.validate(value)
        setattr(instance, self.private_name, value)

    @abstractmethod
    def validate(self, value: Any) -> None:
        """
        Validate the value being assigned to the attribute.

        Subclasses must implement this method to enforce specific validation rules.

        Args:
            value (Any): The value to validate.

        Raises:
            Exception: If the value does not meet the validation criteria.
        """
        pass


class Number(Validator):
    """
    Validator for numerical values.

    Ensures that the value is an integer or a float and optionally enforces a minimum
    and/or maximum limit.
    """

    def __init__(
        self,
        min_value: Optional[Union[int, float]] = None,
        max_value: Optional[Union[int, float]] = None,
    ) -> None:
        """
        Initialize the Number validator.

        Args:
            min_value (Optional[Union[int, float]]): The minimum acceptable value (inclusive).
                Defaults to None, meaning no lower bound.
            max_value (Optional[Union[int, float]]): The maximum acceptable value (inclusive).
                Defaults to None, meaning no upper bound.
        """
        self.min_value: Optional[Union[int, float]] = min_value
        self.max_value: Optional[Union[int, float]] = max_value

    def validate(self, value: Union[int, float]) -> None:
        """
        Validate that the value is a number and within the specified range.

        Args:
            value (Union[int, float]): The number to validate.

        Raises:
            TypeError: If the value is not an int or float.
            ValueError: If the value is less than min_value or greater than max_value.
        """
        if not isinstance(value, (int, float)):
            raise TypeError("Value must be an int or float")

        if self.min_value is not None and value < self.min_value:
            raise ValueError(f"Value must be at least {self.min_value}")

        if self.max_value is not None and value > self.max_value:
            raise ValueError(f"Value must be at most {self.max_value}")


if __name__ == "__main__":
    # Example usage demonstrating the Number validator.

    class Gains:
        """
        Example class using the Number validator to validate gain values.

        The attribute 'kp' is validated to ensure it is a number between 0 and 100.
        """

        kp = Number(0, 100)

        def __init__(self, kp_value: int) -> None:
            """
            Initialize the Gains instance.

            Args:
                kp_value (int): The gain value to be validated and assigned to 'kp'.
            """
            self.kp = kp_value

    # This instantiation will raise a ValueError because 200 is greater than the allowed maximum of 100.
    g = Gains(200)
