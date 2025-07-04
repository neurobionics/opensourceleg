import random
import warnings
from abc import ABC, abstractmethod
from collections.abc import Iterator
from typing import Any, Optional


class SignalGenerator(ABC):
    def __init__(
        self,
        amplitude: float = 1.0,
        offset: float = 0.0,
        duration: Optional[float] = None,
        noise_amplitude: float = 0.0,
        seed: Optional[int] = None,
        sample_rate: float = 1000.0,
    ):
        """
        Base class for signal generators.

        Args:
            amplitude: Scaling factor for the signal
            offset: DC offset added to output
            duration: Fixed runtime duration in seconds (None = infinite)
            noise_amplitude: Amplitude of additive uniform noise
            seed: Random seed for reproducible noise
            sample_rate: Nominal sample rate for time tracking (Hz)
        """
        if amplitude < 0:
            raise ValueError("Amplitude must be non-negative")
        if noise_amplitude < 0:
            raise ValueError("Noise amplitude must be non-negative")
        if sample_rate <= 0:
            raise ValueError("Sample rate must be positive")

        self.amplitude = amplitude
        self.offset = offset
        self.duration = duration
        self.noise_amplitude = noise_amplitude
        self.sample_rate = sample_rate
        self._rng = random.Random(seed)  # noqa: S311
        self._precomputed: list[float] = []
        self._current_index = 0

    def reset(self) -> None:
        """Reset internal state (if applicable)"""
        self._current_index = 0

    def update(self, t: float) -> Optional[float]:
        """
        Generate signal value for given time.

        Args:
            t: Current time in seconds

        Returns:
            Generated signal value or None if duration exceeded
        """
        # Validate time input
        if t < 0:
            raise ValueError("Time must be non-negative")

        # Check duration limit
        if self.duration is not None and t > self.duration:
            return None

        # Generate signal
        signal = self._generate(t)
        noise = self._rng.uniform(-self.noise_amplitude, self.noise_amplitude) if self.noise_amplitude > 0 else 0.0

        return signal + noise + self.offset

    @abstractmethod
    def _generate(self, t: float) -> float:
        """Core signal generation logic (to implement in subclasses)"""
        pass

    def generate_sequence(self, duration: float, dt: Optional[float] = None) -> tuple[list[float], list[float]]:
        """
        Generate a sequence of values for given duration.

        Args:
            duration: Time length of sequence (seconds)
            dt: Time step between samples (seconds).
                 If None, uses 1/sample_rate.

        Returns:
            (time_points, signal_values) tuple
        """
        if dt is None:
            dt = 1.0 / self.sample_rate

        time_points = []
        signal_values = []
        num_samples = int(duration / dt)

        for i in range(num_samples):
            t = i * dt
            value = self.update(t)
            if value is None:  # Handle finite-duration signals
                break
            time_points.append(t)
            signal_values.append(value)

        return time_points, signal_values

    def precompute(self, duration: float, dt: Optional[float] = None) -> None:
        """
        Precompute signal values for given duration and time step.

        Args:
            duration: Time length to precompute (seconds)
            dt: Time step between samples (seconds).
                 If None, uses 1/sample_rate.
        """
        _, self._precomputed = self.generate_sequence(duration, dt)
        self._current_index = 0

    def __iter__(self) -> Iterator[float]:
        """Initialize iterator for precomputed values"""
        self._current_index = 0
        return self

    def __next__(self) -> float:
        """Get the next precomputed value"""
        if self._current_index >= len(self._precomputed):
            raise StopIteration
        value = self._precomputed[self._current_index]
        self._current_index += 1
        return value

    def plot(
        self, duration: float = 5.0, dt: Optional[float] = None, title: Optional[str] = None, show: bool = True
    ) -> Any:
        """
        Plot the signal waveform (requires matplotlib).

        Args:
            duration: Time length to plot (seconds)
            dt: Time step between samples (seconds)
            title: Custom plot title
            show: Whether to immediately show the plot

        Returns:
            Matplotlib figure and axes objects
        """
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            warnings.warn("Plotting requires matplotlib. Install with 'pip install matplotlib'", stacklevel=2)
            return None, None

        time_points, signal_values = self.generate_sequence(duration, dt)

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(time_points, signal_values)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Amplitude")
        ax.set_title(title or f"{self.__class__.__name__} Waveform")
        ax.grid(True)
        fig.tight_layout()

        if show:
            plt.show()

        return fig, ax
