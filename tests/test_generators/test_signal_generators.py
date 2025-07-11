import math

import pytest

from opensourceleg.generators.signal_generators import (
    CustomGenerator,
    DataReplayGenerator,
    ExponentialGenerator,
    RampGenerator,
    SawtoothGenerator,
    SineGenerator,
    SquareGenerator,
    StepGenerator,
    TriangleGenerator,
)


class TestStepGenerator:
    """Test suite for StepGenerator."""

    def test_step_generator_basic(self):
        """Test basic step generator functionality."""
        generator = StepGenerator(step_time=2.0, step_value=5.0)

        # Before step time
        assert generator.update(0.0) == 0.0
        assert generator.update(1.5) == 0.0
        assert generator.update(1.99) == 0.0

        # At and after step time
        assert generator.update(2.0) == 5.0
        assert generator.update(2.1) == 5.0
        assert generator.update(10.0) == 5.0

    def test_step_generator_default_values(self):
        """Test step generator with default parameters."""
        generator = StepGenerator()

        # Default: step_time=0.0, step_value=1.0
        assert generator.update(0.0) == 1.0
        assert generator.update(1.0) == 1.0

    def test_step_generator_negative_step_time(self):
        """Test step generator with negative step time."""
        generator = StepGenerator(step_time=-1.0, step_value=3.0)

        # All positive times should be after step
        assert generator.update(0.0) == 3.0
        assert generator.update(1.0) == 3.0

    def test_step_generator_with_noise(self):
        """Test step generator with noise."""
        generator = StepGenerator(step_time=1.0, step_value=2.0, add_noise=True, noise_amplitude=0.1, seed=42)

        # Values should be different due to noise
        value1 = generator.update(2.0)
        value2 = generator.update(3.0)

        # Should be around 2.0 but with noise
        assert 1.8 <= value1 <= 2.2
        assert 1.8 <= value2 <= 2.2
        assert value1 != value2  # Different noise each time


class TestSineGenerator:
    """Test suite for SineGenerator."""

    def test_sine_generator_basic(self):
        """Test basic sine generator functionality."""
        generator = SineGenerator(frequency=1.0, amplitude=2.0, phase=0.0)

        # Test known values
        assert abs(generator.update(0.0) - 0.0) < 1e-10  # sin(0) = 0
        assert abs(generator.update(0.25) - 2.0) < 1e-10  # sin(π/2) = 1, amplitude=2
        assert abs(generator.update(0.5) - 0.0) < 1e-10  # sin(π) = 0
        assert abs(generator.update(0.75) - (-2.0)) < 1e-10  # sin(3π/2) = -1

    def test_sine_generator_frequency(self):
        """Test sine generator with different frequencies."""
        generator = SineGenerator(frequency=2.0, amplitude=1.0)

        # f=2Hz means period=0.5s
        assert abs(generator.update(0.0) - 0.0) < 1e-10
        assert abs(generator.update(0.125) - 1.0) < 1e-10  # Quarter period = π/2
        assert abs(generator.update(0.25) - 0.0) < 1e-10  # Half period = π

    def test_sine_generator_phase(self):
        """Test sine generator with phase offset."""
        generator = SineGenerator(frequency=1.0, amplitude=1.0, phase=math.pi / 2)

        # With π/2 phase, sin becomes cos
        assert abs(generator.update(0.0) - 1.0) < 1e-10  # cos(0) = 1
        assert abs(generator.update(0.25) - 0.0) < 1e-10  # cos(π/2) = 0

    def test_sine_generator_default_values(self):
        """Test sine generator with default parameters."""
        generator = SineGenerator()

        # Default: frequency=1.0, phase=0.0, amplitude=1.0
        assert abs(generator.update(0.0) - 0.0) < 1e-10
        assert abs(generator.update(0.25) - 1.0) < 1e-10


class TestRampGenerator:
    """Test suite for RampGenerator."""

    def test_ramp_generator_positive_slope(self):
        """Test ramp generator with positive slope."""
        generator = RampGenerator(slope=2.0, settling_value=5.0)

        # Before saturation
        assert generator.update(0.0) == 0.0
        assert generator.update(1.0) == 2.0
        assert generator.update(2.0) == 4.0

        # At and after saturation
        assert generator.update(2.5) == 5.0
        assert generator.update(10.0) == 5.0

    def test_ramp_generator_negative_slope(self):
        """Test ramp generator with negative slope."""
        generator = RampGenerator(slope=-1.0, settling_value=-3.0)

        # Before saturation
        assert generator.update(0.0) == 0.0
        assert generator.update(1.0) == -1.0
        assert generator.update(2.0) == -2.0

        # At and after saturation
        assert generator.update(3.0) == -3.0
        assert generator.update(10.0) == -3.0

    def test_ramp_generator_zero_slope(self):
        """Test ramp generator with zero slope."""
        generator = RampGenerator(slope=0.0, settling_value=5.0)

        # Should always be zero
        assert generator.update(0.0) == 0.0
        assert generator.update(10.0) == 0.0

    def test_ramp_generator_default_values(self):
        """Test ramp generator with default parameters."""
        generator = RampGenerator()

        # Default: slope=1.0, settling_value=1.0
        assert generator.update(0.0) == 0.0
        assert generator.update(0.5) == 0.5
        assert generator.update(1.0) == 1.0
        assert generator.update(2.0) == 1.0  # Saturated


class TestSquareGenerator:
    """Test suite for SquareGenerator."""

    def test_square_generator_basic(self):
        """Test basic square generator functionality."""
        generator = SquareGenerator(frequency=1.0, duty_cycle=0.5, amplitude=2.0)

        # Period = 1.0s, duty = 50% means high for 0.5s, low for 0.5s
        assert generator.update(0.0) == 2.0  # High
        assert generator.update(0.25) == 2.0  # Still high
        assert generator.update(0.5) == -2.0  # Low
        assert generator.update(0.75) == -2.0  # Still low
        assert generator.update(1.0) == 2.0  # Next period, high again

    def test_square_generator_duty_cycle(self):
        """Test square generator with different duty cycles."""
        # 25% duty cycle
        generator = SquareGenerator(frequency=1.0, duty_cycle=0.25, amplitude=1.0)

        assert generator.update(0.0) == 1.0  # High
        assert generator.update(0.1) == 1.0  # Still high
        assert generator.update(0.25) == -1.0  # Low (after 25% of period)
        assert generator.update(0.5) == -1.0  # Still low

    def test_square_generator_validation(self):
        """Test square generator parameter validation."""
        # Valid duty cycles
        SquareGenerator(duty_cycle=0.0)  # Should not raise
        SquareGenerator(duty_cycle=1.0)  # Should not raise

        # Invalid duty cycles
        with pytest.raises(ValueError, match="Duty cycle must be between 0 and 1"):
            SquareGenerator(duty_cycle=-0.1)

        with pytest.raises(ValueError, match="Duty cycle must be between 0 and 1"):
            SquareGenerator(duty_cycle=1.1)

    def test_square_generator_default_values(self):
        """Test square generator with default parameters."""
        generator = SquareGenerator()

        # Default: frequency=1.0, duty_cycle=0.5, amplitude=1.0
        assert generator.update(0.0) == 1.0
        assert generator.update(0.5) == -1.0


class TestSawtoothGenerator:
    """Test suite for SawtoothGenerator."""

    def test_sawtooth_generator_basic(self):
        """Test basic sawtooth generator functionality."""
        generator = SawtoothGenerator(frequency=1.0, amplitude=2.0)

        # Period = 1.0s, sawtooth goes from -amplitude to +amplitude
        assert generator.update(0.0) == -2.0  # Start of cycle
        assert generator.update(0.5) == 0.0  # Middle of cycle
        assert abs(generator.update(1.0) - (-2.0)) < 1e-10  # Next cycle starts

    def test_sawtooth_generator_frequency(self):
        """Test sawtooth generator with different frequency."""
        generator = SawtoothGenerator(frequency=2.0, amplitude=1.0)

        # Period = 0.5s
        assert generator.update(0.0) == -1.0
        assert generator.update(0.25) == 0.0  # Middle of cycle
        assert abs(generator.update(0.5) - (-1.0)) < 1e-10  # Next cycle

    def test_sawtooth_generator_default_values(self):
        """Test sawtooth generator with default parameters."""
        generator = SawtoothGenerator()

        # Default: frequency=1.0, amplitude=1.0
        assert generator.update(0.0) == -1.0
        assert generator.update(0.5) == 0.0


class TestTriangleGenerator:
    """Test suite for TriangleGenerator."""

    def test_triangle_generator_symmetric(self):
        """Test triangle generator with symmetric rise/fall."""
        generator = TriangleGenerator(frequency=1.0, symmetry=0.5, amplitude=2.0)

        # Period = 1.0s, symmetry = 0.5 means rise for 0.5s, fall for 0.5s
        assert generator.update(0.0) == -2.0  # Start (minimum)
        assert generator.update(0.25) == 0.0  # Quarter way up
        assert generator.update(0.5) == 2.0  # Peak
        assert generator.update(0.75) == 0.0  # Quarter way down
        assert abs(generator.update(1.0) - (-2.0)) < 1e-10  # Back to minimum

    def test_triangle_generator_asymmetric(self):
        """Test triangle generator with asymmetric rise/fall."""
        generator = TriangleGenerator(frequency=1.0, symmetry=0.25, amplitude=1.0)

        # Rise for 25% of period, fall for 75%
        assert generator.update(0.0) == -1.0  # Minimum
        assert generator.update(0.125) == 0.0  # Half way up (12.5% of period)
        assert generator.update(0.25) == 1.0  # Peak (25% of period)
        assert generator.update(0.625) == 0.0  # Half way down

    def test_triangle_generator_validation(self):
        """Test triangle generator parameter validation."""
        # Valid symmetry values
        TriangleGenerator(symmetry=0.0)  # Should not raise
        TriangleGenerator(symmetry=1.0)  # Should not raise

        # Invalid symmetry values
        with pytest.raises(ValueError, match="Symmetry must be between 0 and 1"):
            TriangleGenerator(symmetry=-0.1)

        with pytest.raises(ValueError, match="Symmetry must be between 0 and 1"):
            TriangleGenerator(symmetry=1.1)

    def test_triangle_generator_default_values(self):
        """Test triangle generator with default parameters."""
        generator = TriangleGenerator()

        # Default: frequency=1.0, symmetry=0.5, amplitude=1.0
        assert generator.update(0.0) == -1.0
        assert generator.update(0.5) == 1.0


class TestExponentialGenerator:
    """Test suite for ExponentialGenerator."""

    def test_exponential_generator_growth(self):
        """Test exponential generator for growth (decay=False)."""
        generator = ExponentialGenerator(time_constant=1.0, saturation_value=1.0, decay=False)

        # Growth: starts at 0, approaches 1
        assert abs(generator.update(0.0) - 0.0) < 1e-10
        assert 0.0 < generator.update(0.5) < 1.0  # Partial growth
        assert 0.0 < generator.update(1.0) < 1.0  # More growth

        # Should approach but not exceed saturation value
        value_large_t = generator.update(10.0)
        assert 0.99 < value_large_t < 1.0

    def test_exponential_generator_decay(self):
        """Test exponential generator for decay (decay=True)."""
        generator = ExponentialGenerator(time_constant=1.0, saturation_value=1.0, decay=True)

        # Decay: starts at saturation_value, approaches 0
        assert abs(generator.update(0.0) - 1.0) < 1e-10
        assert 0.0 < generator.update(0.5) < 1.0  # Partial decay
        assert 0.0 < generator.update(1.0) < 1.0  # More decay

        # Should approach but not reach 0
        value_large_t = generator.update(10.0)
        assert 0.0 < value_large_t < 0.01

    def test_exponential_generator_time_constant(self):
        """Test exponential generator with different time constants."""
        # Faster response (smaller time constant)
        gen_fast = ExponentialGenerator(time_constant=0.5, saturation_value=1.0, decay=False)

        # Slower response (larger time constant)
        gen_slow = ExponentialGenerator(time_constant=2.0, saturation_value=1.0, decay=False)

        # At same time, faster should be closer to saturation
        t = 1.0
        fast_value = gen_fast.update(t)
        slow_value = gen_slow.update(t)

        assert fast_value > slow_value

    def test_exponential_generator_default_values(self):
        """Test exponential generator with default parameters."""
        generator = ExponentialGenerator()

        # Default: time_constant=1.0, saturation_value=1.0, decay=False
        assert abs(generator.update(0.0) - 0.0) < 1e-10
        assert generator.update(10.0) < 1.0


class TestDataReplayGenerator:
    """Test suite for DataReplayGenerator."""

    def test_data_replay_generator_basic(self):
        """Test basic data replay functionality."""
        data = [1.0, 2.0, 3.0, 4.0]
        generator = DataReplayGenerator(data=data, sample_rate=2.0)  # 2 Hz

        # dt = 1/2 = 0.5s between samples
        assert generator.update(0.0) == 1.0  # Index 0
        assert generator.update(0.5) == 2.0  # Index 1
        assert generator.update(1.0) == 3.0  # Index 2
        assert generator.update(1.5) == 4.0  # Index 3

    def test_data_replay_generator_loop(self):
        """Test data replay with looping enabled."""
        data = [10.0, 20.0]
        generator = DataReplayGenerator(data=data, sample_rate=1.0, loop=True)

        # Should loop back to beginning
        assert generator.update(0.0) == 10.0  # Index 0
        assert generator.update(1.0) == 20.0  # Index 1
        assert generator.update(2.0) == 10.0  # Index 0 (looped)
        assert generator.update(3.0) == 20.0  # Index 1 (looped)

    def test_data_replay_generator_no_loop(self):
        """Test data replay with looping disabled."""
        data = [5.0, 15.0, 25.0]
        generator = DataReplayGenerator(data=data, sample_rate=1.0, loop=False)

        # Should hold last value when reaching end
        assert generator.update(0.0) == 5.0  # Index 0
        assert generator.update(1.0) == 15.0  # Index 1
        assert generator.update(2.0) == 25.0  # Index 2
        assert generator.update(3.0) == 25.0  # Hold last value
        assert generator.update(10.0) == 25.0  # Still hold last value

    def test_data_replay_generator_negative_time_error(self):
        """Test data replay generator with negative time."""
        data = [1.0, 2.0]
        generator = DataReplayGenerator(data=data, sample_rate=1.0)

        with pytest.raises(ValueError, match="Time must be non-negative"):
            generator.update(-1.0)

    def test_data_replay_generator_empty_data(self):
        """Test data replay generator with empty data."""
        data = []

        # Should raise IndexError when trying to access empty list
        with pytest.raises(IndexError, match="Data list is empty"):
            DataReplayGenerator(data=data, sample_rate=1.0)

    def test_data_replay_generator_fractional_time(self):
        """Test data replay generator with fractional time indexing."""
        data = [100.0, 200.0, 300.0]
        generator = DataReplayGenerator(data=data, sample_rate=10.0)  # 10 Hz

        # dt = 0.1s between samples
        assert generator.update(0.05) == 150.0  # Linear between 100 and 200
        assert generator.update(0.15) == 250.0  # Linear between 200 and 300


class TestCustomGenerator:
    """Test suite for CustomGenerator."""

    def test_custom_generator_time_only(self):
        """Test custom generator with time-only expression."""
        generator = CustomGenerator("2 * t")

        # No additional variables needed
        assert generator.update(0.0) == 0.0
        assert generator.update(1.0) == 2.0
        assert generator.update(2.5) == 5.0

    def test_custom_generator_with_constants(self):
        """Test custom generator with built-in constants."""
        generator = CustomGenerator("pi * t")

        assert abs(generator.update(1.0) - math.pi) < 1e-10
        assert abs(generator.update(2.0) - 2 * math.pi) < 1e-10

    def test_custom_generator_with_variables(self):
        """Test custom generator with additional variables."""
        generator = CustomGenerator("A * sin(2*pi*f*t)", ["A", "f"])

        # Should raise error if called without variable values
        with pytest.raises(ValueError, match="Missing value for variable"):
            generator.update(1.0)

    def test_custom_generator_with_variables_update(self):
        """Test custom generator with variables using update method."""
        generator = CustomGenerator("A * sin(2*pi*f*t)", ["A", "f"])

        # Test with A=2, f=1 (1 Hz sine with amplitude 2)
        result = generator.update(0.0, A=2.0, f=1.0)
        assert abs(result - 0.0) < 1e-10  # sin(0) = 0

        result = generator.update(0.25, A=2.0, f=1.0)
        assert abs(result - 2.0) < 1e-10  # sin(π/2) = 1, A=2

        result = generator.update(0.5, A=2.0, f=1.0)
        assert abs(result - 0.0) < 1e-10  # sin(π) = 0

    def test_custom_generator_missing_variables(self):
        """Test custom generator with missing variable values."""
        generator = CustomGenerator("A * t + B", ["A", "B"])

        # Missing B
        with pytest.raises(ValueError, match="Missing value for variable: B"):
            generator.update(1.0, A=2.0)

        # Missing A
        with pytest.raises(ValueError, match="Missing value for variable: A"):
            generator.update(1.0, B=3.0)

    def test_custom_generator_negative_time_error(self):
        """Test custom generator with negative time."""
        generator = CustomGenerator("A * t", ["A"])

        with pytest.raises(ValueError, match="Time must be non-negative"):
            generator.update(-1.0, A=2.0)

    def test_custom_generator_with_noise(self):
        """Test custom generator with noise."""
        generator = CustomGenerator("t", add_noise=True, noise_amplitude=0.1, seed=42)

        # Values should be different due to noise
        value1 = generator.update(2.0)
        value2 = generator.update(2.0)

        # Should be around 2.0 but with noise
        assert 1.8 <= value1 <= 2.2
        assert 1.8 <= value2 <= 2.2
        assert value1 != value2  # Different noise each time

    def test_custom_generator_complex_expression(self):
        """Test custom generator with complex mathematical expression."""
        # Quadratic: y = a*t^2 + b*t + c
        generator = CustomGenerator("a*t**2 + b*t + c", ["a", "b", "c"])

        # Test with a=1, b=2, c=3: y = t^2 + 2t + 3
        result = generator.update(0.0, a=1.0, b=2.0, c=3.0)
        assert result == 3.0  # 0 + 0 + 3

        result = generator.update(1.0, a=1.0, b=2.0, c=3.0)
        assert result == 6.0  # 1 + 2 + 3

        result = generator.update(2.0, a=1.0, b=2.0, c=3.0)
        assert result == 11.0  # 4 + 4 + 3


class TestGeneratorSequences:
    """Test sequence generation for all generators."""

    def test_step_generator_sequence(self):
        """Test step generator sequence generation."""
        generator = StepGenerator(step_time=0.5, step_value=2.0)

        time_points, values = generator.generate_sequence(duration=1.0, dt=0.25)

        expected_times = [0.0, 0.25, 0.5, 0.75]
        expected_values = [0.0, 0.0, 2.0, 2.0]

        assert time_points == expected_times
        assert values == expected_values

    def test_sine_generator_sequence(self):
        """Test sine generator sequence generation."""
        generator = SineGenerator(frequency=1.0, amplitude=1.0)

        time_points, values = generator.generate_sequence(duration=1.0, dt=0.25)

        expected_times = [0.0, 0.25, 0.5, 0.75]

        assert time_points == expected_times
        assert len(values) == 4
        assert abs(values[0] - 0.0) < 1e-10  # sin(0)
        assert abs(values[1] - 1.0) < 1e-10  # sin(π/2)
        assert abs(values[2] - 0.0) < 1e-10  # sin(π)
        assert abs(values[3] - (-1.0)) < 1e-10  # sin(3π/2)

    def test_data_replay_sequence(self):
        """Test data replay generator sequence generation."""
        data = [10.0, 20.0, 30.0, 40.0]
        generator = DataReplayGenerator(data=data, sample_rate=2.0)

        time_points, values = generator.generate_sequence(duration=2.0, dt=0.5)

        expected_times = [0.0, 0.5, 1.0, 1.5]
        expected_values = [10.0, 20.0, 30.0, 40.0]

        assert time_points == expected_times
        assert values == expected_values

    def test_custom_generator_sequence_time_only(self):
        """Test custom generator sequence generation with time-only expression."""
        generator = CustomGenerator("2 * t")

        time_points, values = generator.generate_sequence(duration=1.0, dt=0.5)

        expected_times = [0.0, 0.5]
        expected_values = [0.0, 1.0]  # 2*0, 2*0.5

        assert time_points == expected_times
        assert values == expected_values

    def test_custom_generator_sequence_with_variables(self):
        """Test custom generator sequence generation with variables."""
        generator = CustomGenerator("A * t + B", ["A", "B"])

        time_points, values = generator.generate_sequence(duration=1.0, dt=0.25, A=2.0, B=1.0)

        expected_times = [0.0, 0.25, 0.5, 0.75]
        expected_values = [1.0, 1.5, 2.0, 2.5]  # 2*t + 1

        assert time_points == expected_times
        assert values == expected_values

    def test_custom_generator_sequence_missing_variables(self):
        """Test custom generator sequence generation with missing variables."""
        generator = CustomGenerator("A * t", ["A"])

        # Should raise error if variables not provided
        with pytest.raises(ValueError, match="Missing value for variable: A"):
            generator.generate_sequence(duration=1.0, dt=0.5)
