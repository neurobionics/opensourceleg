import random
from abc import ABC, abstractmethod
from typing import Any, Optional


class SignalGenerator(ABC):
    def __init__(
        self,
        duration: Optional[float] = None,
        add_noise: bool = True,
        noise_amplitude: float = 0.0,
        noise_type: str = "uniform",
        seed: Optional[int] = None,
        sample_rate: float = 1000.0,
        **kwargs: Any,
    ):
        """
        Base class for signal generators.

        Args:
            duration: Fixed runtime duration in seconds (None = infinite) - used only for generate_sequence
            add_noise: Whether noise can be added (even if noise_amplitude > 0)
            noise_amplitude: Amplitude of additive noise (range for uniform, std dev for gaussian if sigma not provided)
            noise_type: Type of noise distribution ("uniform" or "gaussian")
            seed: Random seed for reproducible noise
            sample_rate: Nominal sample rate for time tracking (Hz)
            **kwargs: Additional parameters specific to signal generator subclasses
                     For gaussian noise: mu (mean, default=0.0), sigma (std dev, default=noise_amplitude)
        """
        if noise_amplitude < 0:
            raise ValueError("Noise amplitude must be non-negative")
        if sample_rate <= 0:
            raise ValueError("Sample rate must be positive")
        if noise_type not in ("uniform", "gaussian"):
            raise ValueError("Noise type must be 'uniform' or 'gaussian'")

        self.duration = duration
        self.noise_amplitude = noise_amplitude
        self.noise_type = noise_type
        self.add_noise = add_noise
        self.sample_rate = sample_rate

        # Extract gaussian noise parameters from kwargs
        self.mu = kwargs.pop("mu", 0.0)  # Default mean = 0
        self.sigma = kwargs.pop("sigma", noise_amplitude)  # Default std dev = noise_amplitude

        self._has_noise = add_noise and noise_amplitude > 0
        self._noise_range = noise_amplitude

        # Initialize RNG
        self._rng = random.Random(seed)  # noqa: S311

        # Store additional parameters for subclasses
        for key, value in kwargs.items():
            setattr(self, key, value)

    @abstractmethod
    def generate(self, t: float) -> float:
        """
        Core signal generation logic (to implement in subclasses).

        Args:
            t: Current time in seconds

        Returns:
            Generated signal value
        """
        pass

    def update(self, t: float, **kwargs: Any) -> float:
        """
        Generate signal value for given time.

        Args:
            t: Current time in seconds

        Returns:
            Generated signal value
        """
        if t < 0:
            raise ValueError("Time must be non-negative")

        # Generate signal
        signal = self.generate(t, **kwargs)

        # Add noise if configured
        if self._has_noise:
            if self.noise_type == "uniform":
                signal += self._rng.uniform(-self._noise_range, self._noise_range)
            else:  # gaussian
                signal += self._rng.gauss(self.mu, self.sigma)

        return signal

    def generate_sequence(
        self, duration: float, dt: Optional[float] = None, **kwargs: Any
    ) -> tuple[list[float], list[float]]:
        """
        Generate a sequence of values for given duration.

        Args:
            duration: Time length of sequence (seconds)
            dt: Time step between samples (seconds).
                 If None, uses 1/sample_rate.
            **kwargs: Additional variables for CustomGenerator (e.g., A=1.0, f=2.0)

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

            # Check duration limit only for automated sequence generation
            if self.duration is not None and t > self.duration:
                break

            value = self.update(t, **kwargs)
            time_points.append(t)
            signal_values.append(value)

        return time_points, signal_values
