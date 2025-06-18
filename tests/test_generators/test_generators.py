import math
from unittest.mock import patch

import pytest

from opensourceleg.generators import (
    CompositeGenerator,
    ConstantGenerator,
    CustomGenerator,
    DataReplayGenerator,
    ExponentialGenerator,
    ExpressionEvaluator,
    GaussianNoiseGenerator,
    ImpulseGenerator,
    RampGenerator,
    SawtoothGenerator,
    SignalGenerator,
    SineGenerator,
    SquareGenerator,
    TriangleGenerator,
    UniformNoiseGenerator,
    plot_signals,
)

# Common test parameters
DEFAULT_DT = 0.01
DEFAULT_DURATION = 2.0
TOLERANCE = 1e-5
SEED = 42


@pytest.fixture
def constant_generator():
    return ConstantGenerator(constant=2.5, amplitude=1.0, offset=0.5)


@pytest.fixture
def sine_generator():
    return SineGenerator(frequency=1.0, amplitude=2.0, offset=1.0, phase=math.pi / 2)


@pytest.fixture
def ramp_generator():
    return RampGenerator(slope=1.5, amplitude=2.0)


@pytest.fixture
def composite_generator():
    return CompositeGenerator(
        generators=[SineGenerator(frequency=1.0), ConstantGenerator(constant=0.5)], operation="add"
    )


@pytest.fixture
def custom_generator():
    return CustomGenerator(expression="a * sin(2*pi*f*t) + noise", variables={"a": 2.0, "f": 0.5, "noise": 0.1})


class TestSignalGeneratorBase:
    def test_initialization(self):
        gen = SignalGenerator(amplitude=2.0, offset=1.0)
        assert gen.amplitude == 2.0
        assert gen.offset == 1.0
        assert gen.duration is None
        assert gen.noise_amplitude == 0.0
        assert gen.sample_rate == 1000.0

    def test_reset(self):
        gen = SignalGenerator()
        gen(0.1)
        gen.reset()
        assert gen._time == 0.0

    def test_duration_limit(self):
        gen = SignalGenerator(duration=0.5)
        assert gen(0.1) is not None
        assert gen(0.5) is None

    def test_sequence_generation(self):
        gen = SignalGenerator()
        times, values = gen.generate_sequence(duration=0.1, dt=0.01)
        assert len(times) == 10
        assert len(values) == 10
        assert times[-1] == pytest.approx(0.09, abs=TOLERANCE)

    @patch("matplotlib.pyplot.show")
    def test_plotting(self, mock_show):
        gen = SignalGenerator()
        fig, ax = gen.plot(duration=0.1, show=False)
        assert fig is not None
        assert ax is not None


class TestConstantGenerator:
    def test_value(self, constant_generator):
        assert constant_generator(0.1) == pytest.approx(3.0, abs=TOLERANCE)
        assert constant_generator(1.0) == pytest.approx(3.0, abs=TOLERANCE)

    def test_noise(self):
        gen = ConstantGenerator(constant=2.0, noise_amplitude=0.1, seed=SEED)
        values = [gen(0.1) for _ in range(3)]
        assert values == pytest.approx([2.075, 1.975, 2.025], abs=0.05)


class TestSineGenerator:
    def test_key_points(self, sine_generator):
        assert sine_generator(0.0) == pytest.approx(3.0, abs=TOLERANCE)
        assert sine_generator(0.25) == pytest.approx(1.0, abs=0.1)
        assert sine_generator(0.5) == pytest.approx(-1.0, abs=0.1)

    def test_phase(self):
        gen = SineGenerator(frequency=1.0, phase=math.pi)
        assert gen(0.0) == pytest.approx(0.0, abs=TOLERANCE)


class TestRampGenerator:
    def test_slope(self, ramp_generator):
        assert ramp_generator(0.0) == pytest.approx(0.0, abs=TOLERANCE)
        assert ramp_generator(1.0) == pytest.approx(3.0, abs=TOLERANCE)

    def test_reset_at_duration(self):
        gen = RampGenerator(slope=1.0, duration=2.0, reset_at_duration=True)
        assert gen(1.0) == pytest.approx(1.0, abs=TOLERANCE)
        assert gen(1.0) == pytest.approx(1.0, abs=TOLERANCE)  # Reset at 2.0
        assert gen(1.0) == pytest.approx(2.0, abs=TOLERANCE)  # Now at 1.0 again


class TestSquareGenerator:
    def test_duty_cycle(self):
        gen = SquareGenerator(frequency=1.0, duty_cycle=0.3, amplitude=5.0, offset=1.0)
        assert gen(0.0) == pytest.approx(6.0, abs=TOLERANCE)
        assert gen(0.2) == pytest.approx(6.0, abs=TOLERANCE)
        assert gen(0.31) == pytest.approx(-4.0, abs=TOLERANCE)


class TestSawtoothGenerator:
    def test_waveform(self):
        gen = SawtoothGenerator(frequency=1.0, amplitude=2.0, offset=1.0)
        assert gen(0.0) == pytest.approx(-1.0, abs=TOLERANCE)
        assert gen(0.25) == pytest.approx(1.0, abs=TOLERANCE)
        assert gen(0.5) == pytest.approx(3.0, abs=TOLERANCE)
        assert gen(1.0) == pytest.approx(-1.0, abs=TOLERANCE)


class TestTriangleGenerator:
    def test_symmetry(self):
        gen = TriangleGenerator(frequency=1.0, symmetry=0.5, amplitude=2.0, offset=1.0)
        assert gen(0.0) == pytest.approx(-1.0, abs=TOLERANCE)
        assert gen(0.25) == pytest.approx(1.0, abs=TOLERANCE)
        assert gen(0.5) == pytest.approx(3.0, abs=TOLERANCE)
        assert gen(0.75) == pytest.approx(1.0, abs=TOLERANCE)


class TestExponentialGenerator:
    def test_decay(self):
        gen = ExponentialGenerator(time_constant=1.0, growth=False, amplitude=2.0)
        assert gen(0.0) == pytest.approx(0.0, abs=TOLERANCE)
        assert gen(1.0) == pytest.approx(2.0 * (1 - 1 / math.e), abs=0.01)

    def test_growth(self):
        gen = ExponentialGenerator(time_constant=1.0, growth=True, amplitude=3.0)
        assert gen(0.0) == pytest.approx(3.0, abs=TOLERANCE)
        assert gen(1.0) == pytest.approx(3.0 / math.e, abs=0.01)


class TestImpulseGenerator:
    def test_impulse(self):
        gen = ImpulseGenerator(impulse_time=1.0, impulse_value=5.0, amplitude=2.0)
        assert gen(0.5) == pytest.approx(0.0, abs=TOLERANCE)
        assert gen(1.0) == pytest.approx(10.0, abs=TOLERANCE)
        assert gen(2.0) == pytest.approx(10.0, abs=TOLERANCE)


class TestNoiseGenerators:
    def test_uniform_noise(self):
        gen = UniformNoiseGenerator(amplitude=1.0, seed=SEED)
        values = [gen(0.1) for _ in range(3)]
        assert all(-1.0 <= v <= 1.0 for v in values)

    def test_gaussian_noise(self):
        gen = GaussianNoiseGenerator(std_dev=0.5, amplitude=1.0, seed=SEED)
        values = [gen(0.1) for _ in range(3)]
        assert all(isinstance(v, float) for v in values)

    def test_reproducibility(self):
        gen1 = UniformNoiseGenerator(amplitude=1.0, seed=SEED)
        gen2 = UniformNoiseGenerator(amplitude=1.0, seed=SEED)
        assert gen1(0.1) == gen2(0.1)


class TestDataReplayGenerator:
    def test_sequence_replay(self):
        gen = DataReplayGenerator(data=[1, 2, 3], sample_rate=1.0)
        assert gen(1.0) == 1
        assert gen(1.0) == 2
        assert gen(1.0) == 3
        assert gen(1.0) == 1  # Looped

    def test_no_loop(self):
        gen = DataReplayGenerator(data=[10, 20], sample_rate=2.0, loop=False)
        assert gen(0.5) == 10
        assert gen(0.5) == 20
        assert gen(0.5) is None


class TestExpressionEvaluator:
    def test_simple_expression(self):
        evaluator = ExpressionEvaluator("2 + 3 * 4")
        assert evaluator.evaluate() == 14.0

    def test_variables(self):
        evaluator = ExpressionEvaluator("a * sin(2*pi*f*t)", {"a": 2.0, "f": 1.0})
        assert evaluator.evaluate({"t": 0.25}) == pytest.approx(2.0, abs=0.1)

    def test_math_constants(self):
        evaluator = ExpressionEvaluator("sin(pi/2) + log(e)")
        assert evaluator.evaluate() == pytest.approx(2.0, abs=0.1)

    def test_conditional(self):
        evaluator = ExpressionEvaluator("x if t > 0.5 else y", {"x": 10, "y": 5})
        assert evaluator.evaluate({"t": 0.4}) == 5
        assert evaluator.evaluate({"t": 0.6}) == 10

    def test_invalid_expression(self):
        with pytest.raises(ValueError):
            ExpressionEvaluator("invalid + syntax").evaluate()


class TestCustomGenerator:
    def test_simple_expression(self, custom_generator):
        assert custom_generator(0.0) == pytest.approx(0.1, abs=TOLERANCE)
        assert custom_generator(0.5) == pytest.approx(2.0 * math.sin(math.pi * 0.5) + 0.1, abs=0.1)

    def test_nested_generators(self):
        sine_gen = SineGenerator(frequency=1.0)
        gen = CustomGenerator("main(t) + offset", variables={"main": sine_gen, "offset": 0.5})
        assert gen(0.0) == pytest.approx(0.5, abs=TOLERANCE)
        assert gen(0.25) == pytest.approx(1.5, abs=0.1)


class TestCompositeGenerator:
    def test_add_operation(self, composite_generator):
        assert composite_generator(0.0) == pytest.approx(0.5, abs=TOLERANCE)
        assert composite_generator(0.25) == pytest.approx(1.5, abs=0.1)

    def test_multiply_operation(self):
        gen = CompositeGenerator(
            generators=[SineGenerator(frequency=1.0), ConstantGenerator(constant=0.5)], operation="multiply"
        )
        assert gen(0.0) == pytest.approx(0.0, abs=TOLERANCE)
        assert gen(0.25) == pytest.approx(0.5, abs=0.1)

    def test_reset_propagation(self, composite_generator):
        composite_generator(0.25)
        composite_generator.reset()
        assert composite_generator(0.0) == pytest.approx(0.5, abs=TOLERANCE)


class TestPlottingFunctions:
    @patch("matplotlib.pyplot.show")
    def test_single_plot(self, mock_show):
        gen = SineGenerator(frequency=1.0)
        fig, ax = gen.plot(duration=0.1, show=False)
        assert fig is not None
        assert ax is not None

    @patch("matplotlib.pyplot.show")
    def test_multi_plot(self, mock_show):
        gen1 = SineGenerator(frequency=1.0)
        gen2 = SquareGenerator(frequency=1.0)
        fig, ax = plot_signals([(gen1, "Sine"), (gen2, "Square")], duration=0.1, show=False)
        assert fig is not None
        assert ax is not None


class TestAllGenerators:
    @pytest.mark.parametrize(
        "generator_class,params",
        [
            (ConstantGenerator, {"constant": 1.0}),
            (SineGenerator, {"frequency": 1.0}),
            (RampGenerator, {"slope": 1.0}),
            (SquareGenerator, {"frequency": 1.0}),
            (SawtoothGenerator, {"frequency": 1.0}),
            (TriangleGenerator, {"frequency": 1.0}),
            (ExponentialGenerator, {"time_constant": 1.0}),
            (ImpulseGenerator, {"impulse_time": 0.5}),
            (UniformNoiseGenerator, {}),
            (GaussianNoiseGenerator, {}),
            (DataReplayGenerator, {"data": [1, 2, 3], "sample_rate": 10}),
            (CustomGenerator, {"expression": "t"}),
            (CompositeGenerator, {"generators": [SineGenerator(), ConstantGenerator()]}),
        ],
    )
    def test_sequence_generation(self, generator_class, params):
        gen = generator_class(**params)
        times, values = gen.generate_sequence(duration=0.1, dt=0.01)
        assert len(times) == 10
        assert len(values) == 10
        assert isinstance(values[0], float)

    @pytest.mark.parametrize(
        "generator_class,params",
        [
            (ConstantGenerator, {"constant": 1.0}),
            (SineGenerator, {"frequency": 1.0}),
            (RampGenerator, {"slope": 1.0}),
            # Add other generators as needed
        ],
    )
    def test_reset_functionality(self, generator_class, params):
        gen = generator_class(**params)
        value1 = gen(0.5)
        gen.reset()
        value2 = gen(0.5)
        assert value1 == pytest.approx(value2, abs=TOLERANCE)


class TestEdgeCases:
    def test_zero_duration(self):
        gen = SineGenerator(duration=0)
        assert gen(0.1) is None

    def test_negative_time_step(self):
        gen = ConstantGenerator()
        with pytest.raises(ValueError):
            gen(-0.1)

    def test_large_noise_amplitude(self):
        gen = ConstantGenerator(constant=1.0, noise_amplitude=10.0)
        assert -9.0 <= gen(0.1) <= 11.0

    def test_custom_expression_security(self):
        with pytest.raises(ValueError):
            CustomGenerator("__import__('os').system('rm -rf /')")
