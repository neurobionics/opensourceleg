from functools import wraps

from opensourceleg.logging import LOGGER


def deprecated(func):
    """
    Decorator to mark a function as deprecated.
    """

    @wraps(func)
    def wrapper(*args, **kwargs):
        LOGGER.warning(f"Function `{func.__name__}` is deprecated.")
        return func(*args, **kwargs)

    return wrapper


def deprecated_with_suggestion(alternative_func):
    """
    Decorator to provide an alternative function for a deprecated function.
    """

    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            LOGGER.warning(
                f"Function `{func.__name__}` is deprecated. Please use `{alternative_func.__name__}` instead."
            )
            return func(*args, **kwargs)

        return wrapper

    return decorator


def deprecated_with_routing(alternative_func):
    """
    Decorator to provide an alternative function for a deprecated function. The alternative function will be called
    instead of the deprecated function.
    """

    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            LOGGER.warning(
                f"Function `{func.__name__}` is deprecated. Please use `{alternative_func.__name__}` instead, which will be called automatically now."
            )
            return alternative_func(*args, **kwargs)

        return wrapper

    return decorator


if __name__ == "__main__":

    def add(a, b):
        return a + b

    @deprecated_with_suggestion(add)
    def add_old(a, b):
        return b

    @deprecated_with_routing(add)
    def add_renamed(a, b):
        return a

    print(add_renamed(1, 2))
    print(add_old(1, 2))
    print(add(1, 2))
