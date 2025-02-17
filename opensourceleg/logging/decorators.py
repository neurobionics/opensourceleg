"""
Module providing decorators to mark functions as deprecated.

This module contains three decorators that can be used to mark functions as deprecated,
optionally providing a suggested alternative or automatically routing calls to an alternative
function. When a deprecated function is called, a warning is logged using the LOGGER from
opensourceleg.logging.

Decorators:
    deprecated: Marks a function as deprecated.
    deprecated_with_suggestion: Marks a function as deprecated and suggests an alternative.
    deprecated_with_routing: Marks a function as deprecated and routes calls to an alternative function.
"""

from functools import wraps
from typing import Any, Callable

from opensourceleg.logging import LOGGER


def deprecated(func: Callable) -> Callable:
    """
    Decorator that marks a function as deprecated.

    When the decorated function is called, a warning is logged indicating that the function
    is deprecated, and then the original function is executed with the provided arguments.

    Args:
        func (Callable): The function to be marked as deprecated.

    Returns:
        Callable: A wrapped version of the original function that logs a deprecation warning.
    """

    @wraps(func)
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        LOGGER.warning(f"Function `{func.__name__}` is deprecated.")
        return func(*args, **kwargs)

    return wrapper


def deprecated_with_suggestion(alternative_func: Callable) -> Callable:
    """
    Decorator factory that marks a function as deprecated and suggests an alternative function.

    When the decorated function is called, a warning is logged indicating that the function is deprecated
    and suggesting that the alternative function should be used instead. The deprecated function is then executed.

    Args:
        alternative_func (Callable): The alternative function that should be used instead.

    Returns:
        Callable: A decorator which, when applied to a function, marks it as deprecated with a suggestion.
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
    Decorator factory that marks a function as deprecated and automatically routes calls to an alternative function.

    When the decorated function is called, a warning is logged indicating that the function is deprecated and
    that the alternative function will be called automatically instead. The alternative function is then invoked
    with the provided arguments.

    Args:
        alternative_func (Callable): The function that will be called instead of the deprecated function.

    Returns:
        Callable: A decorator which, when applied to a function, replaces its behavior with that of the alternative.
    """

    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            LOGGER.warning(
                f"Function `{func.__name__}` is deprecated. Please use `{alternative_func.__name__}` instead, "
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
