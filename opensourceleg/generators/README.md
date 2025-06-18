# Signal Generators Module

This module provides an extensible framework for generating mock signals and waveforms for simulation, testing, and offline sensor/actuator data spoofing.
It supports built-in signals (sine, ramp, noise, etc.), custom-expression signals, composite signals, and recorded data replay.
A secure expression engine is included for user-defined mathematical waveforms.

---

## **Table of Contents**

- [Quick Start Example](#quick-start-example)
- [Signal Generator Types](#signal-generator-types)
- [Using Built-in Generators](#using-built-in-generators)
- [Custom Expression Generator](#custom-expression-generator)
- [Composite and Replay Generators](#composite-and-replay-generators)
- [Plotting Signals](#plotting-signals)
- [Extending the Module](#extending-the-module)
- [API Reference](#api-reference)
- [Best Practices](#best-practices)

---

## **Quick Start Example**

```python
from generators import SineGenerator

# Generate a 2 Hz sine wave of amplitude 1.0 for 5 seconds
gen = SineGenerator(amplitude=1.0, frequency=2.0)
gen.plot(duration=5.0)
```

---

## **Signal Generator Types**

- **SineGenerator**: Sine wave
- **RampGenerator**: Linear ramp
- **ConstantGenerator**: Constant value
- **SquareGenerator**: Square wave
- **SawtoothGenerator**: Sawtooth wave
- **TriangleGenerator**: Triangle wave
- **ExponentialGenerator**: Exponential decay/growth
- **ImpulseGenerator**: Single impulse at a time
- **UniformNoiseGenerator**: Uniform random noise
- **GaussianNoiseGenerator**: Gaussian random noise
- **CompositeGenerator**: Combine multiple generators (add, multiply, or custom function)
- **DataReplayGenerator**: Replay recorded data
- **CustomGenerator**: User-defined mathematical expression

---

## **Using Built-in Generators**

### Example: Generate a Sine Wave

```python
from generators import SineGenerator

gen = SineGenerator(frequency=1.0, amplitude=2.0, offset=1.0, sample_rate=1000)
time_pts, values = gen.generate_sequence(duration=3.0)
```

---

## **Custom Expression Generator**

Define any mathematical formula safely using variables and math functions.

```python
from generators import CustomGenerator

expr = "A * sin(2*pi*f*t) + 0.5 * cos(2*pi*0.1*t)"
variables = {'A': 1.5, 'f': 2.0}
gen = CustomGenerator(expression=expr, variables=variables)
```

- **Supported functions:** `sin`, `cos`, `tan`, `exp`, `log`, `sqrt`, `abs`, `ceil`, `floor`, etc.
- **Variables:** Use any variable name you want, e.g., `A`, `f`, and always `t` (for time).

---

## **Composite and Replay Generators**

### CompositeGenerator

Combine any number of generators:

```python
from generators import SineGenerator, RampGenerator, CompositeGenerator

sine = SineGenerator(frequency=2)
ramp = RampGenerator(slope=0.2)
composite = CompositeGenerator([sine, ramp], operation='add')
composite.plot()
```

### DataReplayGenerator

Replay previously recorded data:

```python
from generators import DataReplayGenerator

data = [0.1, 0.2, 0.5, 0.3, 0.0, -0.2]
gen = DataReplayGenerator(data=data, sample_rate=100, loop=True)
gen.plot(duration=0.1)
```

---

## **Plotting Signals**

All generators have a `.plot()` method:

```python
gen.plot(duration=2.0)
```

Or plot multiple signals for comparison:

```python
from generators import plot_signals, SineGenerator, RampGenerator

sine = SineGenerator()
ramp = RampGenerator(slope=0.5)
plot_signals([(sine, "Sine"), (ramp, "Ramp")], duration=2.0)
```

---

## **Extending the Module**

To add a new generator:

1. **Subclass `SignalGenerator` from `base.py`:**

```python
from opensourceleg.generators.base import SignalGenerator

class MyCustomGen(SignalGenerator):
    def __init__(self, myparam=1.0, **kwargs):
        super().__init__(**kwargs)
        self.myparam = myparam

    def _generate(self):
        # Use self._time for current time
        return self.amplitude * (self.myparam * self._time)
```

2. **Register or import it in `signal_generators.py` for easy access.**

---

## **API Reference**

### SignalGenerator (base class)

**Init params:**

- `amplitude`: Scale factor for output
- `offset`: Additive offset
- `duration`: Time limit (seconds)
- `noise_amplitude`: Uniform noise added to output
- `sample_rate`: Used for default dt in sequence generation

**Methods:**

- `.reset()`: Reset internal time
- `.generate_sequence(duration, dt=None)`: Generate time and value arrays
- `.plot(duration, dt=None)`: Plot waveform
- `.__call__(dt=None)`: Get next value (optionally at real-time rate)

### CustomGenerator (expression signals)

**Init params:**

- `expression`: Python string (see above)
- `variables`: Dict of variable names and values

---

## **Best Practices**

- Use a small `dt` (e.g., 0.001) for smooth, accurate signals.
- For real-time output, call the generator in a loop with `time.sleep(dt)`.
- Use the CustomGenerator for any mathematical formulae.
- Use CompositeGenerator for adding noise, or combining signals.
- For deterministic output with noise, set the `seed` parameter.

---

## **Issues?**

Open an issue or PR for bugs, feature requests, or questions!
