from abc import ABC, abstractmethod
from typing import Any, Optional, Union


class Validator(ABC):
    def __set_name__(self, name: str) -> None:
        self.private_name = f"_{name}"

    def __get__(self, instance: Any, objtype: Any = None) -> Any:
        return getattr(instance, self.private_name)

    def __set__(self, instance: Any, value: Any) -> None:
        self.validate(value)
        setattr(instance, self.private_name, value)

    @abstractmethod
    def validate(self, value: Any) -> None:
        pass


class Number(Validator):
    def __init__(
        self, min_value: Optional[Union[int, float]] = None, max_value: Optional[Union[int, float]] = None
    ) -> None:
        self.min_value: Optional[Union[int, float]] = min_value
        self.max_value: Optional[Union[int, float]] = max_value

    def validate(self, value: Union[int, float]) -> None:
        if not isinstance(value, (int, float)):
            raise TypeError("Value must be an int or float")

        if self.min_value is not None and value < self.min_value:
            raise ValueError(f"Value must be at least {self.min_value}")

        if self.max_value is not None and value > self.max_value:
            raise ValueError(f"Value must be at most {self.max_value}")


if __name__ == "__main__":

    class Gains:
        kp = Number(0, 100)

        def __init__(self, price: int) -> None:
            self.kp = price

    g = Gains(200)
