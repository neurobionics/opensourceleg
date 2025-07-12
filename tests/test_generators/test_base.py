import random

import pytest

from opensourceleg.generators.base import SignalGenerator


class ConcreteSignalGenerator(SignalGenerator):
    """Concrete implementation for testing the abstract SignalGenerator."""

    def generate(self, t: float) -> float:
        """Simple linear signal for testing: f(t) = t"""
        return t


def test_init_default_parameters():
    """Test initialization with default parameters."""
    generator = ConcreteSignalGenerator()

    assert generator.duration is None
    assert generator.add_noise is True
    assert generator.noise_amplitude == 0.0
    assert generator.noise_type == "uniform"
    assert generator.sample_rate == 1000.0
    assert generator.mu == 0.0
    assert generator.sigma == 0.0
    assert generator._has_noise is False  # add_noise=True but noise_amplitude=0


def test_init_custom_parameters():
    """Test initialization with custom parameters."""
    generator = ConcreteSignalGenerator(
        duration=5.0, add_noise=False, noise_amplitude=0.1, noise_type="gaussian", seed=42, sample_rate=500.0
    )

    assert generator.duration == 5.0
    assert generator.add_noise is False
    assert generator.noise_amplitude == 0.1
    assert generator.noise_type == "gaussian"
    assert generator.sample_rate == 500.0
    assert generator._has_noise is False  # add_noise=False


def test_init_gaussian_noise_parameters():
    """Test initialization with gaussian noise parameters."""
    generator = ConcreteSignalGenerator(noise_type="gaussian", noise_amplitude=0.5, mu=1.0, sigma=0.3)

    assert generator.mu == 1.0
    assert generator.sigma == 0.3
    assert generator.noise_amplitude == 0.5


def test_init_gaussian_noise_default_sigma():
    """Test that sigma defaults to noise_amplitude for gaussian noise."""
    generator = ConcreteSignalGenerator(noise_type="gaussian", noise_amplitude=0.4, mu=0.5)

    assert generator.mu == 0.5
    assert generator.sigma == 0.4  # Should default to noise_amplitude


def test_init_kwargs_stored_as_attributes():
    """Test that additional kwargs are stored as attributes."""
    generator = ConcreteSignalGenerator(custom_param1="value1", custom_param2=42, custom_param3=True)

    assert hasattr(generator, "custom_param1")
    assert hasattr(generator, "custom_param2")
    assert hasattr(generator, "custom_param3")
    assert generator.custom_param1 == "value1"
    assert generator.custom_param2 == 42
    assert generator.custom_param3 is True


def test_init_has_noise_logic():
    """Test _has_noise is set correctly based on add_noise and noise_amplitude."""
    # Case 1: add_noise=True, noise_amplitude > 0
    gen1 = ConcreteSignalGenerator(add_noise=True, noise_amplitude=0.1)
    assert gen1._has_noise is True

    # Case 2: add_noise=True, noise_amplitude = 0
    gen2 = ConcreteSignalGenerator(add_noise=True, noise_amplitude=0.0)
    assert gen2._has_noise is False

    # Case 3: add_noise=False, noise_amplitude > 0
    gen3 = ConcreteSignalGenerator(add_noise=False, noise_amplitude=0.1)
    assert gen3._has_noise is False

    # Case 4: add_noise=False, noise_amplitude = 0
    gen4 = ConcreteSignalGenerator(add_noise=False, noise_amplitude=0.0)
    assert gen4._has_noise is False


def test_init_validation_negative_noise_amplitude():
    """Test that negative noise amplitude raises ValueError."""
    with pytest.raises(ValueError, match="Noise amplitude must be non-negative"):
        ConcreteSignalGenerator(noise_amplitude=-0.1)


def test_init_validation_non_positive_sample_rate():
    """Test that non-positive sample rate raises ValueError."""
    with pytest.raises(ValueError, match="Sample rate must be positive"):
        ConcreteSignalGenerator(sample_rate=0)

    with pytest.raises(ValueError, match="Sample rate must be positive"):
        ConcreteSignalGenerator(sample_rate=-100)


def test_init_validation_invalid_noise_type():
    """Test that invalid noise type raises ValueError."""
    with pytest.raises(ValueError, match="Noise type must be 'uniform' or 'gaussian'"):
        ConcreteSignalGenerator(noise_type="invalid")


def test_update_without_noise():
    """Test update method without noise."""
    generator = ConcreteSignalGenerator(add_noise=False)

    # Test that update returns the same as generate for various times
    assert generator.update(0.0) == 0.0
    assert generator.update(1.5) == 1.5
    assert generator.update(10.0) == 10.0


def test_update_negative_time_raises_error():
    """Test that negative time raises ValueError."""
    generator = ConcreteSignalGenerator()

    with pytest.raises(ValueError, match="Time must be non-negative"):
        generator.update(-1.0)


def test_update_with_uniform_noise():
    """Test update method with uniform noise."""
    generator = ConcreteSignalGenerator(add_noise=True, noise_amplitude=0.5, noise_type="uniform", seed=42)

    # Test that noise is added
    signal_without_noise = 2.0  # t=2.0 generates signal value 2.0
    signal_with_noise = generator.update(2.0)

    # Signal should be different due to noise
    assert signal_with_noise != signal_without_noise
    # Should be within expected range: [2.0 - 0.5, 2.0 + 0.5]
    assert 1.5 <= signal_with_noise <= 2.5


def test_update_with_gaussian_noise():
    """Test update method with gaussian noise."""
    generator = ConcreteSignalGenerator(
        add_noise=True, noise_amplitude=0.1, noise_type="gaussian", mu=0.0, sigma=0.1, seed=42
    )

    # Test that noise is added
    signal_without_noise = 3.0  # t=3.0 generates signal value 3.0
    signal_with_noise = generator.update(3.0)

    # Signal should be different due to noise
    assert signal_with_noise != signal_without_noise


def test_update_reproducible_with_seed():
    """Test that update produces reproducible results with same seed."""
    gen1 = ConcreteSignalGenerator(add_noise=True, noise_amplitude=0.1, noise_type="uniform", seed=123)
    gen2 = ConcreteSignalGenerator(add_noise=True, noise_amplitude=0.1, noise_type="uniform", seed=123)

    # Same seed should produce same results
    assert gen1.update(1.0) == gen2.update(1.0)
    assert gen1.update(2.0) == gen2.update(2.0)


def test_generate_sequence_default_dt():
    """Test generate_sequence with default dt (1/sample_rate)."""
    generator = ConcreteSignalGenerator(sample_rate=100.0, add_noise=False)

    time_points, signal_values = generator.generate_sequence(duration=0.1)

    expected_dt = 1.0 / 100.0  # 0.01
    expected_samples = int(0.1 / expected_dt)  # 10 samples

    assert len(time_points) == expected_samples
    assert len(signal_values) == expected_samples

    # Check time points are correct
    for i, t in enumerate(time_points):
        assert abs(t - i * expected_dt) < 1e-10

    # Check signal values (should equal time for our concrete implementation)
    for _, (t, signal) in enumerate(zip(time_points, signal_values)):
        assert abs(signal - t) < 1e-10


def test_generate_sequence_custom_dt():
    """Test generate_sequence with custom dt."""
    generator = ConcreteSignalGenerator(add_noise=False)

    duration = 0.5
    dt = 0.1
    time_points, signal_values = generator.generate_sequence(duration=duration, dt=dt)

    expected_samples = int(duration / dt)  # 5 samples

    assert len(time_points) == expected_samples
    assert len(signal_values) == expected_samples

    # Check time points
    expected_times = [0.0, 0.1, 0.2, 0.3, 0.4]
    for expected_t, actual_t in zip(expected_times, time_points):
        assert abs(actual_t - expected_t) < 1e-10


def test_generate_sequence_with_duration_limit():
    """Test generate_sequence respects self.duration limit."""
    generator = ConcreteSignalGenerator(
        duration=0.3,  # Limit duration to 0.3 seconds
        add_noise=False,
    )

    # Try to generate for 0.5 seconds with dt=0.1
    time_points, signal_values = generator.generate_sequence(duration=0.5, dt=0.1)

    # Should stop at 0.3 seconds due to self.duration limit
    # Expected samples: t=0.0, 0.1, 0.2, then stop (0.3 > 0.3 is false, so 0.3 is not included)
    expected_samples = 3  # [0.0, 0.1, 0.2]

    assert len(time_points) == expected_samples
    assert len(signal_values) == expected_samples
    assert max(time_points) <= 0.3


def test_generate_sequence_no_duration_limit():
    """Test generate_sequence without self.duration limit."""
    generator = ConcreteSignalGenerator(
        duration=None,  # No duration limit
        add_noise=False,
    )

    time_points, signal_values = generator.generate_sequence(duration=0.3, dt=0.1)

    # Should generate full requested duration
    expected_samples = int(0.3 / 0.1)  # 3 samples
    assert len(time_points) == expected_samples
    assert len(signal_values) == expected_samples


def test_generate_sequence_with_noise():
    """Test generate_sequence includes noise when configured."""
    generator = ConcreteSignalGenerator(add_noise=True, noise_amplitude=0.1, noise_type="uniform", seed=42)

    time_points, signal_values = generator.generate_sequence(duration=0.02, dt=0.01)

    assert len(time_points) == 2
    assert len(signal_values) == 2

    # Values should be different from clean signal due to noise
    for t, signal in zip(time_points, signal_values):
        clean_signal = t  # Our concrete implementation returns t
        assert signal != clean_signal  # Should have noise


def test_abstract_generate_method():
    """Test that SignalGenerator is abstract and can't be instantiated directly."""
    with pytest.raises(TypeError):
        SignalGenerator()  # This should raise TypeError


def test_rng_initialization():
    """Test that random number generator is properly initialized."""
    generator = ConcreteSignalGenerator(seed=456)

    # Check that _rng is a Random instance
    assert isinstance(generator._rng, random.Random)

    # Test reproducibility
    gen1 = ConcreteSignalGenerator(seed=789, add_noise=True, noise_amplitude=0.1)
    gen2 = ConcreteSignalGenerator(seed=789, add_noise=True, noise_amplitude=0.1)

    # Should produce same random values
    assert gen1._rng.random() == gen2._rng.random()
