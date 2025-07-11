import math
from typing import Any, Optional

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
        position_in_cycle = t % period
        normalized = position_in_cycle / period
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
    def __init__(self, data: list[float], sample_rate: float, loop: bool = True, **kwargs: Any) -> None:
        """
        Replay recorded data from a list.

        Args:
            data: Pre-recorded signal values
            sample_rate: Original replay rate (Hz)
            loop: Loop data when reaching end
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.data = data
        self.replay_rate = sample_rate
        self.loop = loop
        self._index = 0

    def generate(self, t: float) -> float:
        self._index = int(t * self.replay_rate)  # time based index
        if t * self.replay_rate < 0:
            raise ValueError("Time/Sample_Rate cannot be negative")

        if self._index >= len(self.data):
            if self.loop:
                self._index = self._index % len(self.data)
            else:
                return float(self.data[-1])

        return float(self.data[self._index])


class CustomGenerator(SignalGenerator):
    def __init__(self, expression: str, variables: Optional[dict[str, Any]] = None, **kwargs: Any) -> None:
        """
        Custom signal generator from mathematical expression.

        Args:
            expression: Mathematical expression using variables
            variables: Dictionary of variables used in expression
            **kwargs: Base class parameters

        Example:
            gen = CustomGenerator(
                expression="A * sin(2*pi*f*t)",
                variables={'A': 1.0, 'f': 0.5},
            )
        """
        super().__init__(**kwargs)
        self.expression = expression
        self.variables = variables or {}

        self.variables["t"] = 0.0

        self._evaluator = ExpressionEvaluator(expression, self.variables)

    def generate(self, t: float) -> float:
        self.variables["t"] = t
        return self._evaluator.evaluate(self.variables)
