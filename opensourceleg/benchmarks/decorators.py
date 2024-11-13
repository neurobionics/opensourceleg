import timeit
from functools import wraps


def profile_time(iterations):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            timer = timeit.Timer(lambda: func(*args, **kwargs))
            execution_time = timer.timeit(number=iterations)
            print(f"Execution time for {func.__name__} over {iterations} iterations: {execution_time} seconds")
            return None

        return wrapper

    return decorator
