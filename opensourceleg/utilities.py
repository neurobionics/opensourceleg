import csv
import glob
import sys

import numpy as np
import scipy
import serial
from flexsea import flexsea as flex
from flexsea import fxEnums as fxe
from flexsea import fxUtils as fxu

from smbus2 import SMBus

# ---------------------------------------------------------------------------
# Gray C. Thomas, Ph.D's Soft Real Time Loop
# This library will soon be hosted as a PIP module and added as a python dependency.
# https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/main/SoftRealtimeLoop.py

"""
Soft Realtime Loop---a class designed to allow clean exits from infinite loops
with the potential for post-loop cleanup operations executing.

The Loop Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI)
when it detects a shutdown signal, it sets a flag, which is used by the Soft Realtime Loop to stop iterating.
Typically, it detects the CTRL-C from your keyboard, which sends a SIGTERM signal.

the function_in_loop argument to the Soft Realtime Loop's blocking_loop method is the function to be run every loop.
A typical usage would set function_in_loop to be a method of an object, so that the object could store program state.
See the 'ifmain' for two examples.

Author: Gray C. Thomas, Ph.D
https://github.com/GrayThomas, https://graythomas.github.io
"""

import signal
import time
from math import sqrt

PRECISION_OF_SLEEP = 0.0001

# Version of the SoftRealtimeLoop library
__version__ = "1.0.0"


class LoopKiller:
    def __init__(self, fade_time=0.0):
        signal.signal(signal.SIGTERM, self.handle_signal)
        signal.signal(signal.SIGINT, self.handle_signal)
        signal.signal(signal.SIGHUP, self.handle_signal)
        self._fade_time = fade_time
        self._soft_kill_time = None

    def handle_signal(self, signum, frame):
        self.kill_now = True

    def get_fade(self):
        # interpolates from 1 to zero with soft fade out
        if self._kill_soon:
            t = time.time() - self._soft_kill_time
            if t >= self._fade_time:
                return 0.0
            return 1.0 - (t / self._fade_time)
        return 1.0

    _kill_now = False
    _kill_soon = False

    @property
    def kill_now(self):
        if self._kill_now:
            return True
        if self._kill_soon:
            t = time.time() - self._soft_kill_time
            if t > self._fade_time:
                self._kill_now = True
        return self._kill_now

    @kill_now.setter
    def kill_now(self, val):
        if val:
            if self._kill_soon:  # if you kill twice, then it becomes immediate
                self._kill_now = True
            else:
                if self._fade_time > 0.0:
                    self._kill_soon = True
                    self._soft_kill_time = time.time()
                else:
                    self._kill_now = True
        else:
            self._kill_now = False
            self._kill_soon = False
            self._soft_kill_time = None


class SoftRealtimeLoop:
    def __init__(self, dt=0.001, report=False, fade=0.0):
        self.t0 = self.t1 = time.time()
        self.killer = LoopKiller(fade_time=fade)
        self.dt = dt
        self.ttarg = None
        self.sum_err = 0.0
        self.sum_var = 0.0
        self.sleep_t_agg = 0.0
        self.n = 0
        self.report = report

    def __del__(self):
        if self.report:
            print("In %d cycles at %.2f Hz:" % (self.n, 1.0 / self.dt))
            print("\tavg error: %.3f milliseconds" % (1e3 * self.sum_err / self.n))
            print(
                "\tstddev error: %.3f milliseconds"
                % (
                    1e3
                    * sqrt((self.sum_var - self.sum_err**2 / self.n) / (self.n - 1))
                )
            )
            print(
                "\tpercent of time sleeping: %.1f %%"
                % (self.sleep_t_agg / self.time() * 100.0)
            )

    @property
    def fade(self):
        return self.killer.get_fade()

    def run(self, function_in_loop, dt=None):
        if dt is None:
            dt = self.dt
        self.t0 = self.t1 = time.time() + dt
        while not self.killer.kill_now:
            ret = function_in_loop()
            if ret == 0:
                self.stop()
            while time.time() < self.t1 and not self.killer.kill_now:
                if signal.sigtimedwait(
                    [signal.SIGTERM, signal.SIGINT, signal.SIGHUP], 0
                ):
                    self.stop()
            self.t1 += dt
        print("Soft realtime loop has ended successfully.")

    def stop(self):
        self.killer.kill_now = True

    def time(self):
        return time.time() - self.t0

    def time_since(self):
        return time.time() - self.t1

    def __iter__(self):
        self.t0 = self.t1 = time.time() + self.dt
        return self

    def __next__(self):
        if self.killer.kill_now:
            raise StopIteration

        while (
            time.time() < self.t1 - 2 * PRECISION_OF_SLEEP and not self.killer.kill_now
        ):
            t_pre_sleep = time.time()
            time.sleep(
                max(PRECISION_OF_SLEEP, self.t1 - time.time() - PRECISION_OF_SLEEP)
            )
            self.sleep_t_agg += time.time() - t_pre_sleep

        while time.time() < self.t1 and not self.killer.kill_now:
            if signal.sigtimedwait([signal.SIGTERM, signal.SIGINT, signal.SIGHUP], 0):
                self.stop()
        if self.killer.kill_now:
            raise StopIteration
        self.t1 += self.dt
        if self.ttarg is None:
            # inits ttarg on first call
            self.ttarg = time.time() + self.dt
            # then skips the first loop
            return self.t1 - self.t0
        error = time.time() - self.ttarg  # seconds
        self.sum_err += error
        self.sum_var += error**2
        self.n += 1
        self.ttarg += self.dt
        return self.t1 - self.t0


# ---------------------------------------------------------------------------


class CSVLog:
    """
    Logging class to make writing to a CSV file easier.
    See if __name__ == "__main__" for an example.
    At instantiation, pass a list of lists corresponding to the variable names you wish to log, as well as the name of their containers.
    The container name is prepended in the log so you know which object the variable came from.
    These variables should live as attributes within some object accessible to the main loop.
    To update the log, simply call log.update((obj1, obj2, ...)).

    Author: Kevin Best
    https://github.com/tkevinbest
    """

    def __init__(self, file_name, variable_name, container_names) -> None:
        """
        Args:
            file_name (_type_): _description_
            variable_name (_type_): _description_
            container_names (_type_): _description_
        """
        self.file_name = file_name
        self.var_names = variable_name
        column_headers = [
            cont_name + "_" + item
            for (sublist, cont_name) in zip(variable_name, container_names)
            for item in sublist
        ]
        self._write_row(column_headers)

    def update(self, data_containers_in):
        row = []
        for (var_group, container) in zip(self.var_names, data_containers_in):
            if type(container) is dict:
                for var in var_group:
                    row.append(container.get(var))
            else:
                for var in var_group:
                    row.append(getattr(container, var))
        self._write_row(row)

    def _write_row(self, val_list):
        with open(self.file_name, "a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(val_list)


class EdgeDetector:
    """
    Used to calculate rising and falling edges of a digital signal in real time.
    Call edgeDetector.update(digitalSignal) to update the detector.
    Then read edgeDetector.rising_edge or falling edge to know if the event occurred.

    Author: Kevin Best
    https://github.com/tkevinbest
    """

    def __init__(self, bool_in):
        self.cur_state = bool_in
        self.rising_edge = False
        self.falling_edge = False

    def update(self, bool_in):
        self.rising_edge = bool_in and not self.cur_state
        self.falling_edge = not bool_in and self.cur_state
        self.cur_state = bool_in


class SaturatingRamp:
    """
    Creates a signal that ramps between 0 and 1 at the specified rate.
    Looks like a trapezoid in the time domain
    Used to slowly enable joint torque for smooth switching at startup.
    Call saturatingRamp.update() to update the value of the ramp and return the value.
    Can also access saturatingRamp.value without updating.

    Example usage:
        ramp = saturatingRamp(100, 1.0)

        # In loop
            torque = torque * ramp.update(enable_ramp)

    Author: Kevin Best
    https://github.com/tkevinbest
    """

    def __init__(self, loop_frequency=100, ramp_time=1.0) -> None:
        """
        Args:
            loop_frequency (int, optional): Rate in Hz (default 100 Hz). Defaults to 100.
            ramp_time (float, optional): Time to complete the ramp. Defaults to 1.0.
        """
        self.delta_per_update = 1.0 / (loop_frequency * ramp_time)
        self.value = 0.0

    def update(self, enable_ramp=False):
        """
        Updates the ramp value and returns it as a float.
        If enable_ramp is true, ramp value increases
        Otherwise decreases.

        Example usage:
            torque = torque * ramp.update(enable_ramp)

        Args:
            enable_ramp (bool, optional): If enable_ramp is true, ramp value increases. Defaults to False.

        Returns:
            value (float): Scalar between 0 and 1.
        """
        if enable_ramp:
            delta = self.delta_per_update
        else:
            delta = -1 * self.delta_per_update
        self.value += delta

        self.value = min(max(self.value, 0), 1)
        return self.value


def get_active_ports():
    """
    Lists active serial ports.
    """
    if sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        ports = glob.glob("/dev/tty[A-Za-z]*")
    elif sys.platform.startswith("darwin"):
        ports = glob.glob("/dev/tty.*")
    elif sys.platform.startswith("win"):
        ports = ["COM%s" % (i + 1) for i in range(256)]
    else:
        raise OSError("Unsupported platform.")

    serial_ports = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            serial_ports.append(port)
        except (OSError, serial.SerialException):
            pass

    return serial_ports

# could add more functionality/specific protocols to this, but really just need a nice
# way to create the I2C bus in the other interfaces
class I2CManager:
    """
    A singleton wrapper for the SMBus I2C bus, so that we can instantiate it only once.
    """
    # Note, defining singletons in this way means that you cannot inherit
    # from this class, as apparently __init__ for the subclass will be called twice
    _instance = None
    """
    Used to keep track of one instantation of the class to make a singleton object
    """
    
    def __new__(cls, bus):
        """
        Makes a singleton object to manage a socketcan_native CAN bus.
        """
        if not cls._instance:
            cls._instance = super(I2CManager, cls).__new__(cls)
            print("Initializing I2C Manager")
            cls._instance.bus = SMBus(bus)
            # create a python-can notifier object, which motors can later subscribe to
            print("Connected on: " + str(cls._instance.bus))

        return cls._instance

    def __init__(self, bus):
        """
        ALl initialization happens in __new__
        """
        pass
        
    def __del__(self):
        """
        # shut down the bus when the object is deleted
        # This may not ever get called, so keep a reference and explicitly delete if this is important.
        """
        self.bus.close()














