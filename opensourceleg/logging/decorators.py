from functools import wraps
from typing import Any, Callable

from opensourceleg.logging import LOGGER


def deprecated(func: Callable) -> Callable:
    """
    Decorator to mark a function as deprecated.
    """

    @wraps(func)
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        LOGGER.warning(f"Function `{func.__name__}` is deprecated.")
        return func(*args, **kwargs)

    return wrapper


def deprecated_with_suggestion(alternative_func: Callable) -> Callable:
    """
    Decorator to provide an alternative function for a deprecated function.
    """

    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            LOGGER.warning(
                f"Function `{func.__name__}` is deprecated. Please use `{alternative_func.__name__}` instead."
            )
            return func(*args, **kwargs)

        return wrapper

    return decorator


def deprecated_with_routing(alternative_func: Callable) -> Callable:
    """
    Decorator to provide an alternative function for a deprecated function. The alternative function will be called
    instead of the deprecated function.
    """

    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            LOGGER.warning(
                f"Function `{func.__name__}` is deprecated. Please use `{alternative_func.__name__}` instead,"
                "which will be called automatically now."
            )
            return alternative_func(*args, **kwargs)

        return wrapper

    return decorator


if __name__ == "__main__":

    def add(a: int, b: int) -> int:
        return a + b

    @deprecated_with_suggestion(add)
    def add_old(a: int, b: int) -> int:
        return b

    @deprecated_with_routing(add)
    def add_renamed(a: int, b: int) -> int:
        return a

    print(add_renamed(1, 2))
    print(add_old(1, 2))
    print(add(1, 2))
