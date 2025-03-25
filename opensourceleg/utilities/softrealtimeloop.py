import os
import signal
import sys
import time
from math import sqrt
from typing import Any, Callable

# Calculate the precision of sleep. Python 3.11 has nanosecond sleep
# Older versions have microsecond sleep
python_version = sys.version_info.minor
PRECISION_OF_SLEEP = 1e-09 if python_version >= 11 else 1e-06

__all__ = ["LoopKiller", "SoftRealtimeLoop"]


class LoopKiller:
    """
    Soft Realtime Loop---a class designed to allow clean exits from infinite loops
    with the potential for post-loop cleanup operations executing.

    The Loop Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI)
    when it detects a shutdown signal, it sets a flag, which is used by the Soft Realtime Loop to stop iterating.
    Typically, it detects the CTRL-C from your keyboard, which sends a SIGTERM signal.

    the function_in_loop argument to the Soft Realtime Loop's blocking_loop method is the function to be run every loop.
    A typical usage would set function_in_loop to be a method of an object, so that the object
    could store program state. See the 'ifmain' for two examples.

    # This library will soon be hosted as a PIP module and added as a python dependency.
    # https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/main/SoftRealtimeLoop.py

    Author: Gray C. Thomas, Ph.D
    https://github.com/GrayThomas, https://graythomas.github.io

    """

    def __init__(self, fade_time: float = 0.0):
        if os.name == "posix":
            self.signals = [signal.SIGTERM, signal.SIGINT, signal.SIGHUP]
        else:
            self.signals = [signal.SIGTERM, signal.SIGINT]

        for sig in self.signals:
            signal.signal(sig, self.handle_signal)

        self._fade_time: float = fade_time
        self._soft_kill_time: float = 0.0

    def __repr__(self) -> str:
        return "LoopKiller"

    def handle_signal(self, signum: Any, frame: Any) -> None:
        """
        Method to handle the signal from the operating system.
        This method is called when the operating system sends a signal to the process.
        The signal is typically a shutdown signal, such as SIGTERM, SIGINT, or SIGHUP.

        Args:
            signum (Any): The signal number.
            frame (Any): The frame object.

        Returns:
            None

        Example:
            >>> killer = LoopKiller()
            >>> killer.handle_signal(signal.SIGTERM, None)
        """
        self.kill_now = True

    def get_fade(self) -> float:
        """
        Interpolates from 1 to zero with soft fade out.

        Returns:
            float: The fade value.

        Example:
            >>> killer = LoopKiller()
            >>> killer.get_fade()
        """
        if self._kill_soon:
            t = time.monotonic() - self._soft_kill_time
            if t >= self._fade_time:
                return 0.0
            return 1.0 - (t / self._fade_time)
        return 1.0

    _kill_now = False
    _kill_soon = False

    @property
    def kill_now(self) -> bool:
        """
        Property to get the kill_now value.
        If the kill_now value is True, the loop will stop iterating.
        If the kill_now value is False, the loop will continue iterating.

        Returns:
            bool: The kill_now value.

        Example:
            >>> killer = LoopKiller()
            >>> killer.kill_now
        """

        if self._kill_now:
            return True
        if self._kill_soon:
            t = time.monotonic() - self._soft_kill_time
            if t > self._fade_time:
                self._kill_now = True
        return self._kill_now

    @kill_now.setter
    def kill_now(self, val: bool) -> None:
        """
        Setter for the kill_now value. If true is set twice, then the loop will stop iterating immediately.

        Args:
            val (bool): The value to set the kill_now value to.

        Returns:
            None
        """
        if val:
            if self._kill_soon:  # if you kill twice, then it becomes immediate
                self._kill_now = True
            else:
                if self._fade_time > 0.0:
                    self._kill_soon = True
                    self._soft_kill_time = time.monotonic()
                else:
                    self._kill_now = True
        else:
            self._kill_now = False
            self._kill_soon = False
            self._soft_kill_time = 0.0


class SoftRealtimeLoop:
    """
    Soft Realtime Loop---a class designed to allow clean exits from infinite loops
    with the potential for post-loop cleanup operations executing.

    The Loop Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI).
    When it detects a shutdown signal, it sets a flag, which is used by the Soft Realtime Loop to stop iterating.
    Typically, it detects the CTRL-C from your keyboard, which sends a SIGTERM signal.

    The `function_in_loop` argument to the Soft Realtime Loop's `run` method is the function to be run every loop.
    A typical usage would set `function_in_loop` to be a method of an object, so that the object could store
    program state. See the `if __name__ == "__main__"` section for examples.

    This library is based on the original implementation in:
    https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/main/SoftRealtimeLoop.py

    Author:
        - Gray C. Thomas, Ph.D <https://graythomas.github.io>
        - José Montes Pérez <https://github.com/jmontp>
        - Senthur Ayyappan <https://github.com/senthurayyappan>
        - Kevin Best <https://github.com/tkevinbest>
    """

    def __init__(
        self, dt: float = 0.001, report: bool = True, fade: float = 0.0, maintain_original_phase: bool = False
    ):
        """
        Initializes the SoftRealtimeLoop.

        Args:
            dt (float): The time step of the loop in seconds. Default is 0.001 (1ms).
            report (bool): If True, the loop will print a report at the end of the loop. Default is True.
            fade (float): The time in seconds to fade out the loop when it is killed. Default is 0.0.
            maintain_original_phase (bool): If True, the iterator will try to keep the time elapsed
                equal to (loop_number * dt). If False, the iterator will only look at the difference
                between the current loop and the previous loop. Default is False.
        """
        self._fade_time: float = fade
        self.dt: float = dt
        self.report: bool = report
        self._maintain_original_phase: bool = maintain_original_phase  # Can only be configured at init
        self.loop_start_time: float = time.monotonic()
        self.loop_deadline: float = self.loop_start_time + self.dt
        self.iteration_start_time: float = self.loop_start_time
        self.sum_err: float = 0.0
        self.sum_var: float = 0.0
        self.sleep_t_agg: float = 0.0
        self.n: int = 0
        self.killer = LoopKiller(fade_time=self._fade_time)

    def __repr__(self) -> str:
        """
        Returns a string representation of the SoftRealtimeLoop.

        Returns:
            str: The string representation of the loop.
        """
        return "SoftRealtimeLoop"

    def __del__(self) -> None:
        """
        Destructor for the SoftRealtimeLoop.
        Prints the performance report if reporting is enabled.
        """
        self.print_report()

    def print_report(self) -> None:
        """
        Prints a performance report for the loop, including average error,
        standard deviation of error, and percentage of time spent sleeping.
        """
        if self.report and self.n > 0:
            print("In %d cycles at %.2f Hz:" % (self.n, 1.0 / self.dt))
            print("\tavg error: %.3f milliseconds" % (1e3 * self.sum_err / self.n))
            if self.n > 1:
                print(
                    "\tstddev error: %.3f milliseconds"
                    % (1e3 * sqrt(max(0, (self.sum_var - self.sum_err**2 / self.n) / (self.n - 1))))
                )
            else:
                print("\tstddev error: N/A (need at least 2 samples)")

            total_time = self.time_since_start
            print(f"\ttotal time: {total_time:.1f} s")
            if total_time > 0:
                print("\tpercent of time sleeping: %.1f %%" % (self.sleep_t_agg / total_time * 100.0))
            else:
                print("\tpercent of time sleeping: N/A (total time is zero)")

    def reset(self) -> None:
        """
        Resets the loop state and signal handlers to their initial state.
        This allows reusing the same loop instance instead of creating a new one.

        Example:
            >>> loop = SoftRealtimeLoop()
            >>> loop.run(some_function)
            >>> loop.reset()  # Reset for reuse
            >>> loop.run(another_function)
        """
        if self.n > 0:
            self.print_report()
        self.loop_start_time = time.monotonic()
        self.loop_deadline = self.loop_start_time + self.dt
        self.iteration_start_time = self.loop_start_time
        self.sum_err = 0.0
        self.sum_var = 0.0
        self.sleep_t_agg = 0.0
        self.n = 0
        self.killer = LoopKiller(fade_time=self._fade_time)

    def run(self, function_to_run: Callable[[], int]) -> None:
        """
        Runs the loop with the specified function.

        Args:
            function_to_run (Callable[[], int]): The function to execute in each loop iteration.
                The function should return 0 to stop the loop.

        Example:
            >>> loop = SoftRealtimeLoop()
            >>> loop.run(some_function)
        """
        for _t in self:
            result = function_to_run()
            if result == 0:
                self.stop()

    @property
    def fade(self) -> float:
        """
        Gets the fade value, which interpolates from 1 to 0 during a fade-out.

        Returns:
            float: The fade value.

        Example:
            >>> loop = SoftRealtimeLoop()
            >>> loop.fade
        """
        return self.killer.get_fade()

    def stop(self) -> None:
        """
        Stops the loop by setting the kill flag.

        Example:
            >>> loop = SoftRealtimeLoop()
            >>> loop.stop()
        """
        self.killer.kill_now = True

    def __iter__(self) -> "SoftRealtimeLoop":
        """
        Resets the loop and returns the iterator object.

        Returns:
            SoftRealtimeLoop: The iterator object.
        """
        self.reset()
        return self

    def __next__(self) -> float:
        """
        Advances the loop to the next iteration.

        Returns:
            float: The time since the loop started.

        Raises:
            StopIteration: If the loop is stopped.
        """
        if self._maintain_original_phase:
            return self._next_original_phase()
        else:
            return self._next_consistent_dt()

    def _next_original_phase(self) -> float:
        """
        Advances the loop using the maintain original phase method.

        Returns:
            float: The time since the loop started.

        Raises:
            StopIteration: If the loop is stopped.
        """
        if self.killer.kill_now:
            raise StopIteration

        if self.n == 0:
            self.n += 1
            return self.time_since_start

        ## Sleep the amount we need to satisfy the dt.
        sleep_curr_loop = 0.0
        # Calculate the time we need to sleep
        sleep_time = self.loop_deadline - 2 * PRECISION_OF_SLEEP - time.monotonic()

        while time.monotonic() < self.loop_deadline - 2 * PRECISION_OF_SLEEP and not self.killer.kill_now:
            t_pre_sleep = time.monotonic()
            time.sleep(max(PRECISION_OF_SLEEP, self.loop_deadline - time.monotonic() - PRECISION_OF_SLEEP))
            self.sleep_t_agg += time.monotonic() - t_pre_sleep

        while sleep_time > 0 and not self.killer.kill_now:
            # Calculate the time spent sleeping
            t_pre_sleep = time.monotonic()
            # Sleep for the time we need to satisfy the dt
            time.sleep(max(PRECISION_OF_SLEEP, sleep_time + PRECISION_OF_SLEEP))
            # Update the time spent sleeping to calculate the sleep percentage
            sleep_curr_loop += time.monotonic() - t_pre_sleep
            # Recalculate if we still need to sleep
            sleep_time = self.loop_deadline - 2 * PRECISION_OF_SLEEP - time.monotonic()

        # Update the time slept
        self.sleep_t_agg += sleep_curr_loop

        # Busy wait until the time we should be running at
        while time.monotonic() < self.loop_deadline and not self.killer.kill_now:
            if os.name == "posix" and signal.sigtimedwait(self.killer.signals, 0):
                self.stop()

        # If the loop is killed while we were waiting, raise a StopIteration
        if self.killer.kill_now:
            raise StopIteration

        error = time.monotonic() - self.loop_deadline  # seconds
        self.sum_err += error
        self.sum_var += error**2
        self.n += 1

        # Increase the dt naively based on the time that we should have slept
        self.loop_deadline += self.dt

        return self.loop_deadline - self.loop_start_time

    def _next_consistent_dt(self) -> float:
        """
        Advances the loop with a consistent time step.

        Returns:
            float: The time since the loop started.

        Raises:
            StopIteration: If the loop is stopped.
        """
        if self.n == 0:
            self.n += 1
            return self.time_since_start

        # If the loop is killed, raise a StopIteration
        if self.killer.kill_now:
            raise StopIteration

        time_since_last_loop = time.monotonic() - self.iteration_start_time
        sleep_time = max(self.dt - time_since_last_loop - 2 * PRECISION_OF_SLEEP, 0)
        actual_time_to_sleep = max(PRECISION_OF_SLEEP, sleep_time)
        time.sleep(actual_time_to_sleep)
        # Update the time slept
        self.sleep_t_agg += actual_time_to_sleep

        # Busy wait to compensate for sleep durations precision
        time_to_busy_wait = time.monotonic() + PRECISION_OF_SLEEP
        while time.monotonic() < time_to_busy_wait and not self.killer.kill_now:
            if os.name == "posix" and signal.sigtimedwait(self.killer.signals, 0):
                self.stop()
                raise StopIteration

        ## Handle how much error that we have in a given loop
        # Calculate the error for the loop and update the max errors
        error = (self.current_time - self.iteration_start_time) - self.dt
        # Update the statistics for the error
        self.sum_err += abs(error)
        self.sum_var += abs(error) ** 2
        self.n += 1

        # Update the previous loop time
        self.iteration_start_time = self.current_time

        return self.time_since_start

    @property
    def time_since_start(self) -> float:
        """
        Gets the time elapsed since the loop started.

        Returns:
            float: The time since the loop started.
        """
        return self.current_time - self.loop_start_time

    @property
    def current_time(self) -> float:
        """
        Gets the current monotonic time.

        Returns:
            float: The current monotonic time.
        """
        return time.monotonic()


if __name__ == "__main__":
    # Simple demonstration of the SRT loop with run method
    rt_loop = SoftRealtimeLoop(dt=0.1, maintain_original_phase=False)  # 10Hz loop

    class DemoClass:
        def __init__(self) -> None:
            self.x = 5

        def update(self) -> int:
            time.sleep(0.05)
            self.x -= 1
            print(f"x is {self.x}")
            return self.x

    demo_class = DemoClass()

    rt_loop.run(demo_class.update)

    print("Example Loop 1 Completed")

    rt_loop.reset()

    print("\n\n")

    # Simple demonstration of the SRT loop with iterator
    for t in rt_loop:
        print(f"Time: {t:.1f}s")
        time.sleep(0.025)
        if t > 1.0:  # Auto-stop after 1 second if no Ctrl+C
            rt_loop.stop()
