import timeit
from functools import wraps
from typing import Any, Callable


def profile_time(iterations: int) -> Callable:
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> None:
            timer = timeit.Timer(lambda: func(*args, **kwargs))
            execution_time = timer.timeit(number=iterations)
            print(f"Execution time for {func.__name__} over {iterations} iterations: {execution_time} seconds")
            return None

        return wrapper

    return decorator
