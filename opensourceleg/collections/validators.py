from abc import ABC, abstractmethod
from dataclasses import dataclass


class Validator(ABC):
    def __set_name__(self, owner, name):
        self.private_name = f"_{name}"

    def __get__(self, object, objtype=None):
        return getattr(object, self.private_name)

    def __set__(self, object, value):
        self.validate(value)
        setattr(object, self.private_name, value)

    @abstractmethod
    def validate(self, value):
        pass


class Number(Validator):
    def __init__(self, min_value=None, max_value=None) -> None:
        self.min_value = min_value
        self.max_value = max_value

    def validate(self, value):
        if not isinstance(value, (int, float)):
            raise ValueError("Value must be an int or float")

        if self.min_value is not None and value < self.min_value:
            raise ValueError(f"Value must be at least {self.min_value}")

        if self.max_value is not None and value > self.max_value:
            raise ValueError(f"Value must be at most {self.max_value}")


if __name__ == "__main__":

    class Gains:
        kp = Number(0, 100)

        def __init__(self, price):
            self.kp = price

    g = Gains(200)
