from time import time
import numpy as np


class SimpleTimer:

    def __init__(self):
        self.start_time = 0
        self.end_time = 0
        self._checked_end = False

    def start(self, timer_duration):
        self.start_time = time()
        self.end_time = self.start_time + timer_duration
        self._checked_end = False

    @property
    def just_done(self) -> bool:
        if self.is_done and not self._checked_end:
            self._checked_end = True
            return True
        else:
            return False

    @property
    def is_done(self) -> bool:
        return time() >= self.end_time


class LimitVelocity:

    def __init__(self, velocity, update_freq):
        self.velocity = velocity # [_/sec], absolute (pos) maximum change in parameter per second
        self.update_freq = update_freq # [Hz], expected frequency at which parameter will be updated
        self.on_target = 1
        self._step = velocity/update_freq # absolute (pos) step in parameter per cycle
        self._prev_val = 0.0
        self.target = 0.0

    def update(self, target):
        self.target = target
        if target > self._prev_val and target > self._prev_val + self._step: # parameter increasing
            value = self._prev_val + self._step
            self.on_target = 0
        elif target < self._prev_val and target < self._prev_val - self._step: # parameter decreasing
            value = self._prev_val - self._step
            self.on_target = 0
        else:
            value = target
            self.on_target = 1
        self._prev_val = value
        return value

    @property
    def is_stopped(self) -> bool:
        return self._prev_val==0.0 and self.target==0.0






