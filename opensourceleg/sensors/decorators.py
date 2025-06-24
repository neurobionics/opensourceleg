from functools import wraps
from typing import Any, Callable, TypeVar, cast

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
