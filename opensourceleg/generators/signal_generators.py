import math
from typing import Any, Optional

import numpy as np

from .base import SignalGenerator
from .expression_evaluator import ExpressionEvaluator


class StepGenerator(SignalGenerator):
    def __init__(self, step_time: float = 0.0, step_value: float = 1.0, **kwargs: Any) -> None:
        """
        Step generator

        Args:
            step_time: Time when impulse occurs (seconds)
            step_value: Amplitude of impulse
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.step_time = step_time
        self.step_value = step_value

    def generate(self, t: float) -> float:
        value = self.step_value if t >= self.step_time else 0.0
        return value


class SineGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, phase: float = 0.0, amplitude: float = 1.0, **kwargs: Any) -> None:
        """
        Sine wave generator

        Args:
            frequency: Oscillation frequency (Hz)
            phase: Initial phase offset (radians)
            amplitude: Amplitude of the sine wave
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.frequency = frequency
        self.phase = phase
        self.amplitude = amplitude

    def generate(self, t: float) -> float:
        value = self.amplitude * math.sin(2 * math.pi * self.frequency * t + self.phase)
        return value


class RampGenerator(SignalGenerator):
    def __init__(self, slope: float = 1.0, settling_value: float = 1.0, **kwargs: Any) -> None:
        """
        Linear ramp generator that starts at zero and saturates at settling_value.

        Args:
            slope: Rate of change (units/second)
            settling_value: Value at which the ramp saturates (stops increasing)
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.slope = slope
        self.settling_value = settling_value

    def generate(self, t: float) -> float:
        ramp_value = self.slope * t

        if self.slope > 0:
            value = min(ramp_value, self.settling_value)
        elif self.slope < 0:
            value = max(ramp_value, self.settling_value)
        else:
            value = 0.0

        return value


class SquareGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, duty_cycle: float = 0.5, amplitude: float = 1.0, **kwargs: Any) -> None:
        """
        Square wave generator.

        Args:
            frequency: Oscillation frequency (Hz)
            duty_cycle: High duration fraction (0.0-1.0)
            amplitude: Amplitude of the square wave
            **kwargs: Base class parameters
        """
        if not 0 <= duty_cycle <= 1:
            raise ValueError("Duty cycle must be between 0 and 1")

        super().__init__(**kwargs)
        self.frequency = frequency
        self.duty_cycle = duty_cycle
        self.amplitude = amplitude

    def generate(self, t: float) -> float:
        period = 1 / self.frequency
        phase_time = t % period
        high_duration = period * self.duty_cycle
        value = self.amplitude * (1.0 if phase_time < high_duration else -1.0)
        return value


class SawtoothGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, amplitude: float = 1.0, **kwargs: Any) -> None:
        """
        Sawtooth wave generator (rising ramp with repeated resets).

        Args:
            frequency: Oscillation frequency (Hz)
            amplitude: Amplitude of the sawtooth wave
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.frequency = frequency
        self.amplitude = amplitude

    def generate(self, t: float) -> float:
        period = 1 / self.frequency
        position_in_cycle = t % period  # 0 → 1 over each period
        normalized = position_in_cycle / period  # Scale to [-A, +A)
        value = self.amplitude * (2 * normalized - 1)
        return value


class TriangleGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, symmetry: float = 0.5, amplitude: float = 1.0, **kwargs: Any) -> None:
        """
        Triangle wave generator.

        Args:
            frequency: Oscillation frequency (Hz)
            symmetry: Rise/fall ratio (0.0-1.0)
            amplitude: Amplitude of the triangle wave
            **kwargs: Base class parameters
        """
        if not 0 <= symmetry <= 1:
            raise ValueError("Symmetry must be between 0 and 1")

        super().__init__(**kwargs)
        self.frequency = frequency
        self.symmetry = symmetry
        self.amplitude = amplitude

    def generate(self, t: float) -> float:
        period = 1 / self.frequency
        position_in_cycle = t % period
        normalized = position_in_cycle / period

        if normalized < self.symmetry:
            value = normalized / self.symmetry
        else:
            value = 1 - (normalized - self.symmetry) / (1 - self.symmetry)

        value = self.amplitude * (2 * value - 1)
        return value


class ExponentialGenerator(SignalGenerator):
    def __init__(
        self, time_constant: float = 1.0, saturation_value: float = 1.0, decay: bool = False, **kwargs: Any
    ) -> None:
        """
        Exponential generator that approaches saturation_value asymptotically.

        Args:
            time_constant: Time constant (τ) in seconds - controls rate of approach
            saturation_value: Final value that the exponential approaches
            decay: True for decay (starts at saturation_value, decays to 0),
                   False for growth (starts at 0, grows to saturation_value)
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.time_constant = time_constant
        self.saturation_value = saturation_value
        self.decay = decay

    def generate(self, t: float) -> float:
        # Calculate exponential decay factor
        decay_factor = math.exp(-t / self.time_constant)

        # Use ternary operator for cleaner code
        value = self.saturation_value * decay_factor if self.decay else self.saturation_value * (1 - decay_factor)

        return value


class DataReplayGenerator(SignalGenerator):
    def __init__(
        self,
        data: np.ndarray,
        sample_rate: float,
        loop: bool = True,
        interpolation: str = "linear",
        **kwargs: Any,
    ) -> None:
        """
        Replay recorded data from a list.

        Args:
            data: Pre-recorded signal values
            sample_rate: Original replay rate (Hz)
            loop: Loop data when reaching end
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        if interpolation not in {"linear", "nearest", "zoh"}:
            raise ValueError("interpolation must be 'linear', 'nearest', or 'zoh'")

        self.data = np.asarray(data, dtype=float)
        self.replay_rate = float(sample_rate)
        self.loop = loop
        self.interpolation = interpolation

        # Empty data check (avoid division/modulo by zero)
        if len(self.data) == 0:
            raise IndexError("Data list is empty")

        self._len = len(self.data)
        self._index = 0  # last integer index accessed for backward-compatibility

    def generate(self, t: float) -> float:
        # TODO: add support for better interpolation
        if t < 0.0:
            raise ValueError("Time must be non-negative")

        index_float = t * self.replay_rate  # fractional sample index

        if self.loop:
            # Allow time to wrap seamlessly
            index_float = index_float % self._len
        elif index_float >= self._len:
            return float(self.data[-1])

        # Zero-order hold / nearest / linear interpolation
        if self.interpolation == "zoh":
            self._index = math.floor(index_float)
            return float(self.data[self._index])

        if self.interpolation == "nearest":
            rounded = round(index_float)
            if self.loop:
                self._index = rounded % self._len
            else:
                self._index = rounded if rounded < self._len else (self._len - 1)
            return float(self.data[self._index])

        # Default: linear interpolation
        idx0 = math.floor(index_float)
        idx1 = idx0 + 1
        frac = index_float - idx0

        if idx1 >= self._len:
            if self.loop:
                idx1 = 0  # wrap to start
            else:
                idx1 = self._len - 1
                frac = 0.0  # clamp at last sample

        self._index = idx0
        return float((1.0 - frac) * self.data[idx0] + frac * self.data[idx1])


class CustomGenerator(SignalGenerator):
    def __init__(self, expression: str, variables: Optional[list[str]] = None, **kwargs: Any) -> None:
        """
        Custom signal generator from mathematical expression.

        Args:
            expression: Mathematical expression using variables and 't' for time
            variables: List of variable names used in expression (excluding 't' which is automatic)
            **kwargs: Base class parameters

        Example:
            gen = CustomGenerator(
                expression="A * sin(2*pi*f*t)",
                variables=['A', 'f'],
            )
            # Then call: gen.update(1.0, A=2.0, f=0.5)
        """
        super().__init__(**kwargs)
        self.expression = expression
        self._variable_names = variables or []

        # 't' is always available and goes first in the argument list
        all_variables = ["t", *self._variable_names]
        self._evaluator = ExpressionEvaluator(expression, all_variables)

    def generate(self, t: float, **kwargs: Any) -> float:
        """
        Generate signal value with time and optional variable values.

        Args:
            t: Current time in seconds
            **kwargs: Values for variables declared in constructor

        Returns:
            Generated signal value
        """
        # Build argument list: t first, then variables in declared order
        args = [t]
        for var_name in self._variable_names:
            if var_name not in kwargs:
                raise ValueError(f"Missing value for variable: {var_name}")
            args.append(kwargs[var_name])

        # This will automatically error if wrong number of args due to ExpressionEvaluator validation
        return self._evaluator.evaluate(*args)
