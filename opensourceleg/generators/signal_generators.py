import math
import warnings
from typing import Any, Callable, Optional, Union

from .base import SignalGenerator


class ConstantGenerator(SignalGenerator):
    def __init__(self, constant: float = 0.0, **kwargs):
        """
        Constant value generator.

        Args:
            constant: Fixed output value
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.constant = constant

    def _generate(self) -> float:
        return self.constant * self.amplitude


class SineGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, phase: float = 0.0, **kwargs):
        """
        Sine wave generator.

        Args:
            frequency: Oscillation frequency (Hz)
            phase: Initial phase offset (radians)
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.frequency = frequency
        self.phase = phase

    def _generate(self) -> float:
        return self.amplitude * math.sin(2 * math.pi * self.frequency * self._time + self.phase)


class RampGenerator(SignalGenerator):
    def __init__(self, slope: float = 1.0, reset_at_duration: bool = False, **kwargs):
        """
        Linear ramp generator.

        Args:
            slope: Rate of change (units/second)
            reset_at_duration: Reset to zero when duration reached
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.slope = slope
        self.reset_at_duration = reset_at_duration

    def _generate(self) -> float:
        value = self.slope * self._time
        if self.reset_at_duration and self.duration is not None:
            value %= self.duration * self.slope
        return self.amplitude * value


class SquareGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, duty_cycle: float = 0.5, **kwargs):
        """
        Square wave generator.

        Args:
            frequency: Oscillation frequency (Hz)
            duty_cycle: High duration fraction (0.0-1.0)
            **kwargs: Base class parameters
        """
        if not 0 <= duty_cycle <= 1:
            raise ValueError("Duty cycle must be between 0 and 1")

        super().__init__(**kwargs)
        self.frequency = frequency
        self.duty_cycle = duty_cycle

    def _generate(self) -> float:
        period = 1 / self.frequency
        phase_time = self._time % period
        high_duration = period * self.duty_cycle
        value = 1.0 if phase_time < high_duration else -1.0
        return self.amplitude * value


class SawtoothGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, **kwargs):
        """
        Sawtooth wave generator (rising ramp with reset).

        Args:
            frequency: Oscillation frequency (Hz)
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.frequency = frequency

    def _generate(self) -> float:
        period = 1 / self.frequency
        position_in_cycle = self._time % period
        normalized = position_in_cycle / period
        return self.amplitude * (2 * normalized - 1)


class TriangleGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, symmetry: float = 0.5, **kwargs):
        """
        Triangle wave generator.

        Args:
            frequency: Oscillation frequency (Hz)
            symmetry: Rise/fall ratio (0.0-1.0)
            **kwargs: Base class parameters
        """
        if not 0 <= symmetry <= 1:
            raise ValueError("Symmetry must be between 0 and 1")

        super().__init__(**kwargs)
        self.frequency = frequency
        self.symmetry = symmetry

    def _generate(self) -> float:
        period = 1 / self.frequency
        position_in_cycle = self._time % period
        normalized = position_in_cycle / period

        if normalized < self.symmetry:
            value = normalized / self.symmetry
        else:
            value = 1 - (normalized - self.symmetry) / (1 - self.symmetry)

        return self.amplitude * (2 * value - 1)


class ExponentialGenerator(SignalGenerator):
    def __init__(self, time_constant: float = 1.0, growth: bool = False, **kwargs):
        """
        Exponential decay/growth generator.

        Args:
            time_constant: Time constant (τ) in seconds
            growth: True for growth, False for decay
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.time_constant = time_constant
        self.growth = growth

    def _generate(self) -> float:
        exponent = -self._time / self.time_constant
        if self.growth:
            return self.amplitude * (math.exp(exponent))
        else:
            return self.amplitude * (1 - math.exp(exponent))


class ImpulseGenerator(SignalGenerator):
    def __init__(self, impulse_time: float = 1.0, impulse_value: float = 1.0, **kwargs):
        """
        Impulse generator (single step at specified time).

        Args:
            impulse_time: Time when impulse occurs (seconds)
            impulse_value: Amplitude of impulse
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.impulse_time = impulse_time
        self.impulse_value = impulse_value

    def _generate(self) -> float:
        return self.amplitude * (self.impulse_value if self._time >= self.impulse_time else 0)


class UniformNoiseGenerator(SignalGenerator):
    def __init__(self, **kwargs):
        """Uniform noise generator"""
        super().__init__(**kwargs)

    def _generate(self) -> float:
        return self.amplitude * self._rng.uniform(-1, 1)


class GaussianNoiseGenerator(SignalGenerator):
    def __init__(self, std_dev: float = 1.0, **kwargs):
        """
        Gaussian noise generator.

        Args:
            std_dev: Standard deviation of distribution
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.std_dev = std_dev

    def _generate(self) -> float:
        return self.amplitude * self._rng.gauss(0, self.std_dev)


class DataReplayGenerator(SignalGenerator):
    def __init__(self, data: list[float], sample_rate: float, loop: bool = True, **kwargs):
        """
        Replay recorded data from a list.

        Args:
            data: Pre-recorded signal values
            sample_rate: Original sample rate (Hz)
            loop: Loop data when reaching end
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.data = data
        self.sample_rate = sample_rate
        self.loop = loop
        self._index = 0

    def reset(self) -> None:
        super().reset()
        self._index = 0

    def _generate(self) -> float:
        if self._index >= len(self.data):
            if self.loop:
                self._index = 0
            else:
                return None

        value = self.data[self._index]
        self._index += 1
        return value


class CompositeGenerator(SignalGenerator):
    def __init__(
        self, generators: list[SignalGenerator], operation: Union[str, Callable[[list[float]], float]] = "add", **kwargs
    ):
        super().__init__(**kwargs)
        self.generators = generators
        self.operation = operation

        if not callable(operation) and operation not in ["add", "multiply"]:
            raise ValueError("Operation must be 'add', 'multiply', or a callable")

    def reset(self) -> None:
        """Reset all child generators"""
        super().reset()
        for gen in self.generators:
            gen.reset()

    def _generate(self) -> float:
        """
        Generate composite value with synchronized timing

        Uses the same time for all child generators to keep them in sync
        """
        # Generate values from all child generators
        values = []
        for gen in self.generators:
            original_time = gen._time

            # Sync child time with composite time
            gen._time = self._time

            # Generate value from child
            value = gen._generate()

            gen._time = original_time

            values.append(value)

        # Apply the combination operation
        if callable(self.operation):
            return self.operation(values)
        elif self.operation == "add":
            return sum(values)
        elif self.operation == "multiply":
            result = 1.0
            for v in values:
                result *= v
            return result


def plot_signals(
    generators: list[tuple[SignalGenerator, str]],
    duration: float = 5.0,
    dt: Optional[float] = None,
    title: str = "Signal Comparison",
    show: bool = True,
) -> Any:
    """
    Plot multiple signals for comparison.

    Args:
        generators: List of (generator, label) tuples
        duration: Time to plot (seconds)
        dt: Time step between samples (seconds)
        title: Plot title
        show: Whether to immediately show the plot

    Returns:
        Matplotlib figure and axes objects
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        warnings.warn("Plotting requires matplotlib. Install with 'pip install matplotlib'", stacklevel=2)
        return None, None

    fig, ax = plt.subplots(figsize=(12, 6))

    for gen, label in generators:
        time_points, signal_values = gen.generate_sequence(duration, dt)
        ax.plot(time_points, signal_values, label=label)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Amplitude")
    ax.set_title(title)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    if show:
        plt.show()

    return fig, ax
