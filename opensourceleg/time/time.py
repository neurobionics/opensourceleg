import signal
import time
from math import sqrt
from typing import Any, Callable, Optional

PRECISION_OF_SLEEP = 0.0001


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
        signal.signal(signal.SIGTERM, self.handle_signal)
        signal.signal(signal.SIGINT, self.handle_signal)
        signal.signal(signal.SIGHUP, self.handle_signal)
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

    The Loop Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI)
    when it detects a shutdown signal, it sets a flag, which is used by the Soft Realtime Loop to stop iterating.
    Typically, it detects the CTRL-C from your keyboard, which sends a SIGTERM signal.

    the function_in_loop argument to the Soft Realtime Loop's blocking_loop method is the function to be run every loop.
    A typical usage would set function_in_loop to be a method of an object, so that the object could store
    program state. See the 'ifmain' for two examples.

    This library will soon be hosted as a PIP module and added as a python dependency.
    https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/main/SoftRealtimeLoop.py

    # Author: Gray C. Thomas, Ph.D
    # https://github.com/GrayThomas, https://graythomas.github.io

    """

    def __init__(self, dt: float = 0.001, report: bool = True, fade: float = 0.0):
        self.t0: float = time.monotonic()
        self.t1: float = self.t0
        self.killer: LoopKiller = LoopKiller(fade_time=fade)
        self.dt: float = dt
        self.ttarg: Any = None
        self.sum_err: float = 0.0
        self.sum_var: float = 0.0
        self.sleep_t_agg: float = 0.0
        self.n: int = 0
        self.report: bool = report

    def __repr__(self) -> str:
        return "SoftRealtimeLoop"

    def __del__(self) -> None:
        if self.report:
            print("In %d cycles at %.2f Hz:" % (self.n, 1.0 / self.dt))
            print("\tavg error: %.3f milliseconds" % (1e3 * self.sum_err / self.n))
            print(
                "\tstddev error: %.3f milliseconds"
                % (1e3 * sqrt((self.sum_var - self.sum_err**2 / self.n) / (self.n - 1)))
            )
            print("\tpercent of time sleeping: %.1f %%" % (self.sleep_t_agg / self.time() * 100.0))

    @property
    def fade(self) -> float:
        """
        Property to get the fade value.

        Returns:
            float: The fade value.

        Example:
            >>> loop = SoftRealtimeLoop()
            >>> loop.fade
        """
        return self.killer.get_fade()

    def run(self, function_in_loop: Callable, dt: Optional[float] = None) -> None:
        """
        Method to run the function within the time loop.

        Args:
            function_in_loop (Callable): The function to run within the time loop.
            dt (Optional[float]): The time delta. Defaults to None.

        Returns:
            None

        Example:
            >>> loop = SoftRealtimeLoop()
            >>> loop.run(function_in_loop)
            TODO: Better example here.
        """
        if dt is None:
            dt = self.dt
        self.t0 = self.t1 = time.monotonic() + dt
        while not self.killer.kill_now:
            ret = function_in_loop()
            if ret == 0:
                self.stop()
            while time.monotonic() < self.t1 and not self.killer.kill_now:
                if signal.sigtimedwait([signal.SIGTERM, signal.SIGINT, signal.SIGHUP], 0):
                    self.stop()
            self.t1 += dt

    def stop(self) -> None:
        """
        Method to stop the loop.

        Returns:
            None

        Example:
            >>> loop = SoftRealtimeLoop()
            >>> loop.start()
            ... Running loop ...
            >>> loop.stop()
        """
        self.killer.kill_now = True

    def time(self) -> float:
        """
        Method to get the current time since the start of the time loop.

        Returns:
            float: The time since the start of the time loop.

        Example:
            >>> loop = SoftRealtimeLoop()
            >>> loop.time()
        """
        return time.monotonic() - self.t0

    def time_since(self) -> float:
        """
        Method to get the time since the last loop. TODO: Is this true?

        Returns:
            float: The time since the last loop.

        Example:
            >>> loop = SoftRealtimeLoop()
            >>> loop.time_since()
        """
        return time.monotonic() - self.t1

    def __iter__(self) -> "SoftRealtimeLoop":
        self.t0 = self.t1 = time.monotonic() + self.dt
        return self

    def __next__(self) -> float:
        if self.killer.kill_now:
            raise StopIteration

        while time.monotonic() < self.t1 - 2 * PRECISION_OF_SLEEP and not self.killer.kill_now:
            t_pre_sleep = time.monotonic()
            time.sleep(max(PRECISION_OF_SLEEP, self.t1 - time.monotonic() - PRECISION_OF_SLEEP))
            self.sleep_t_agg += time.monotonic() - t_pre_sleep

        while time.monotonic() < self.t1 and not self.killer.kill_now:
            try:
                if signal.sigtimedwait([signal.SIGTERM, signal.SIGINT, signal.SIGHUP], 0):
                    self.stop()
            except AttributeError:
                pass

        if self.killer.kill_now:
            raise StopIteration
        self.t1 += self.dt
        if self.ttarg is None:
            # inits ttarg on first call
            self.ttarg = time.monotonic() + self.dt
            # then skips the first loop
            return self.t1 - self.t0
        error = time.monotonic() - self.ttarg  # seconds
        self.sum_err += error
        self.sum_var += error**2
        self.n += 1
        self.ttarg += self.dt
        return self.t1 - self.t0


if __name__ == "__main__":
    pass
