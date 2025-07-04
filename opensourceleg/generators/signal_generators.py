import math
import warnings
from typing import Any, Callable, Optional, Union

from .base import SignalGenerator
from .expression_evaluator import ExpressionEvaluator


class StepGenerator(SignalGenerator):
    def __init__(self, step_time: float = 0.0, step_value: float = 1.0, **kwargs: Any) -> None:
        """
        Step generator (constant after specified time).

        Args:
            step_time: Time when impulse occurs (seconds)
                 If None, defaults to 0.0 (Constant signal)
            step_value: Amplitude of impulse
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.step_time = step_time
        self.step_value = step_value

    def _generate(self, t: float) -> float:
        return self.amplitude * (self.step_value if t >= self.step_time else 0.0)


class SineGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, phase: float = 0.0, **kwargs: Any) -> None:
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

    def _generate(self, t: float) -> float:
        return self.amplitude * math.sin(2 * math.pi * self.frequency * t + self.phase)


class RampGenerator(SignalGenerator):
    def __init__(self, slope: float = 1.0, reset_time: Optional[float] = None, **kwargs: Any) -> None:
        """
        Linear ramp generator.

        Args:
            slope: Rate of change (units/second)
            reset_time: Reset to zero when the specified time is reached
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.slope = slope
        self.reset_time = reset_time

    def _generate(self, t: float) -> float:
        value = self.slope * t
        if self.reset_time is not None:
            value %= self.reset_time * self.slope
        return self.amplitude * value


class SquareGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, duty_cycle: float = 0.5, **kwargs: Any) -> None:
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

    def _generate(self, t: float) -> float:
        period = 1 / self.frequency
        phase_time = t % period
        high_duration = period * self.duty_cycle
        value = 1.0 if phase_time < high_duration else -1.0
        return self.amplitude * value


class SawtoothGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, **kwargs: Any) -> None:
        """
        Sawtooth wave generator (rising ramp with repeated resets).

        Args:
            frequency: Oscillation frequency (Hz)
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.frequency = frequency

    def _generate(self, t: float) -> float:
        period = 1 / self.frequency
        position_in_cycle = t % period
        normalized = position_in_cycle / period
        return self.amplitude * (2 * normalized - 1)


class TriangleGenerator(SignalGenerator):
    def __init__(self, frequency: float = 1.0, symmetry: float = 0.5, **kwargs: Any) -> None:
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

    def _generate(self, t: float) -> float:
        period = 1 / self.frequency
        position_in_cycle = t % period
        normalized = position_in_cycle / period

        if normalized < self.symmetry:
            value = normalized / self.symmetry
        else:
            value = 1 - (normalized - self.symmetry) / (1 - self.symmetry)

        return self.amplitude * (2 * value - 1)


class ExponentialGenerator(SignalGenerator):
    def __init__(self, time_constant: float = 1.0, growth: bool = False, **kwargs: Any) -> None:
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

    def _generate(self, t: float) -> float:
        exponent = -t / self.time_constant
        if self.growth:
            return self.amplitude * (math.exp(exponent))
        else:
            return self.amplitude * (1 - math.exp(exponent))


class UniformNoiseGenerator(SignalGenerator):
    def __init__(self, **kwargs: Any) -> None:
        """Uniform noise generator"""
        super().__init__(**kwargs)

    def _generate(self, t: float) -> float:
        return self.amplitude * self._rng.uniform(-1, 1)


class GaussianNoiseGenerator(SignalGenerator):
    def __init__(self, std_dev: float = 1.0, **kwargs: Any) -> None:
        """
        Gaussian noise generator.

        Args:
            std_dev: Standard deviation of distribution
            **kwargs: Base class parameters
        """
        super().__init__(**kwargs)
        self.std_dev = std_dev

    def _generate(self, t: float) -> float:
        return self.amplitude * self._rng.gauss(0, self.std_dev)


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

    def reset(self) -> None:
        super().reset()
        self._index = 0

    def _generate(self, t: float) -> float:
        self._index = int(t * self.replay_rate)  # time based index
        if t * self.replay_rate < 0:
            raise ValueError("Time/Sample_Rate cannot be negative")

        if self._index >= len(self.data):
            if self.loop:
                self._index = self._index % len(self.data)
            else:
                return self.data[-1]

        value = self.data[self._index]
        return value


class CompositeGenerator(SignalGenerator):
    def __init__(
        self,
        generators: list[SignalGenerator],
        operation: Union[str, Callable[[list[float]], float]] = "add",
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self.generators = generators
        self.operation = operation

        if not callable(operation) and operation not in ["add", "multiply"]:
            raise ValueError("Operation must be 'add', 'multiply', or a callable")

    def _generate(self, t: float) -> float:
        """Generate composite value at given time"""
        values = []
        for gen in self.generators:
            signal = gen._generate(t)
            noise = self._rng.uniform(-gen.noise_amplitude, gen.noise_amplitude) if gen.noise_amplitude > 0 else 0.0
            values.append(signal + noise + gen.offset)

        # Combination operation
        if callable(self.operation):
            return self.operation(values)
        elif self.operation == "add":
            return sum(values)
        elif self.operation == "multiply":
            result = 1.0
            for v in values:
                result *= v
            return result
        else:
            raise ValueError(f"Unsupported operation: {self.operation}")


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
                expression="A * sin(2*pi*f*t) + noise",
                variables={'A': 1.0, 'f': 0.5},
                noise_amplitude=0.1
            )
        """
        super().__init__(**kwargs)
        self.expression = expression
        self.variables = variables or {}

        # Add time to variables
        self.variables["t"] = 0.0

        # Compile expression
        self._evaluator = ExpressionEvaluator(expression, self.variables)

    def _generate(self, t: float) -> float:
        # Update time variable
        self.variables["t"] = t
        return self._evaluator.evaluate(self.variables)


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
