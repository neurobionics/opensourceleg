import time
from math import sqrt
from typing import Any, Callable, Union

import numpy as np

__all__ = ["Profiler"]


class Profiler:
    """
    A class to profile the execution speed of real-time code.

    Can use in four ways:
        1. tic/toc
        2. lambda
        3. decorator
        4. context manager

    See examples in the main block of this file.

    Based on original implementation in https://github.com/UM-LoCoLab/NeuroLocoMiddleware

    **Note**:
        This profiler bases its calculation on the system time and includes in its calculations
        any time spent waiting for user input, interfacing with I/O, etc.
        Use caution when profiling any code with such components, as differences in user response
        or external systems can affect the result.

    Author:
        - Dr. Gray Cortright Thomas <gthomas@tamu.edu>
        - Kevin Best <tkevinbest@gmail.com>

    Args:
        name (str): The name of the profiler instance.
    """

    def __init__(self, name: str):
        self._N: int = 0
        self._agg: float = 0.0
        self._aggvar: float = 0.0
        self.name: str = name
        self._t0: Union[float, None] = None

    def __del__(self) -> None:
        """
        Destructor to print profiling results when the instance is deleted.
        """
        try:
            mean: float = self.agg / self.N
            stddev: float = sqrt((self.aggvar - self.N * mean**2) / (self.N - 1))
        except ZeroDivisionError:
            mean = float("NaN")
            stddev = float("NaN")
        print(
            f"Profiler Results - {self.name}: N = {self.N}, avg: {mean * 1e3} ms, "
            f"stddev: {stddev * 1e3} ms, total: {self.agg} s"
        )

    def __enter__(self) -> None:
        self.tic()

    def __exit__(self, *args: Any) -> None:
        self.toc()

    @property
    def N(self) -> int:
        """
        Get the number of recorded intervals.

        Returns:
            int: The number of recorded intervals.
        """
        return self._N

    @property
    def agg(self) -> float:
        """
        Get the aggregate time of all intervals.

        Returns:
            float: The aggregate time.
        """
        return self._agg

    @property
    def aggvar(self) -> float:
        """
        Get the aggregate variance of all intervals.

        Returns:
            float: The aggregate variance.
        """
        return self._aggvar

    def tic(self) -> None:
        """
        Start a new timing interval.

        Example:
            profiler = Profiler("example")
            profiler.tic()
            time.sleep(0.1)
            profiler.toc()
        """
        self._t0 = time.perf_counter()

    def toc(self) -> float:
        """
        End the current timing interval and record its duration.

        Returns:
            float: The duration of the interval.

        Example:
            profiler = Profiler("example")
            profiler.tic()
            time.sleep(0.1)
            duration = profiler.toc()
        """
        if self._t0 is not None:
            t: float = time.perf_counter() - self._t0
            self._N += 1
            self._agg += t
            self._aggvar += t**2
            return t
        return 0.0

    def profile(self, func: Callable[[], Any]) -> Any:
        """
        Profile a function by measuring its execution time.

        Args:
            func (callable): The function to profile.

        Returns:
            Any: The return value of the function.

        Example:
            profiler = Profiler("example")
            result = profiler.profile(lambda: time.sleep(0.1))
        """
        self.tic()
        x: Any = func()
        self.toc()
        return x

    def decorate(self, func: Callable[..., Any]) -> Callable[..., Any]:
        """
        Decorate a function to automatically profile its execution time.

        Args:
            func (callable): The function to decorate.

        Returns:
            callable: The decorated function.

        Example:
            profiler = Profiler("example")

            @profiler.decorate
            def my_function():
                time.sleep(0.1)

            my_function()
        """

        def ret(*args: Any, **kwargs: Any) -> Any:
            self.tic()
            x: Any = func(*args, **kwargs)
            self.toc()
            return x

        return ret


if __name__ == "__main__":
    # Example tic toc usage
    A = np.random.random((5, 5))
    profiler_tt = Profiler("tic_toc_example")
    profiler_tt.tic()
    result = np.linalg.eig(A)
    profiler_tt.toc()
    profiler_tt.tic()
    A = np.random.random((5, 5))
    result = np.linalg.eig(A)
    t = profiler_tt.toc()

    # Example decorator usage
    @Profiler("decorator_example").decorate
    def my_decorated_function() -> None:
        A = np.random.random((5, 5))
        np.linalg.eig(A)

    for _i in range(100):
        my_decorated_function()

    # Example lambda usage
    for _i in range(3):
        A = np.random.random((5, 5))
        f = lambda local_A=A: np.linalg.eig(local_A)
        Profiler("lambda_example").profile(f)

    # Example with context usage
    with Profiler("context_example") as p:
        A = np.random.random((5, 5))
        np.linalg.eig(A)
        A = np.random.random((5, 5))
        np.linalg.eig(A)
