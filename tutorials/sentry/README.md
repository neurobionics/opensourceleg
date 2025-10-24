# Sentry Integration Guide

**Optimized for Raspberry Pi real-time robotics applications**

## Simple Philosophy: Profile Hardware I/O Only

The opensourceleg library uses Sentry to monitor **hardware I/O operations** - the operations that directly affect real-time performance:

- **Component updates**: `actuator.update()`, `sensor.update()`
- **Motor commands**: `set_motor_position()`, `set_motor_current()`, `set_motor_voltage()`, etc.
- **Hardware communication**: Any method that talks to physical hardware

**Not profiled**: Initialization, user code, non-hardware operations

## Privacy Guarantees

- ✅ **Only internal library code tracked** - User code paths completely filtered out
- ✅ **No PII** - `send_default_pii=False`
- ✅ **User file paths scrubbed** - Stack traces only include opensourceleg library frames
- ✅ **Complete opt-out** - `OPENSOURCELEG_DISABLE_SENTRY=1`

## Configuration (Simple!)

### Profiling: On or Off

```bash
# Off (DEFAULT for Raspberry Pi)
# - Only error tracking
# - <1% overhead
# - Recommended for production robots
export OPENSOURCELEG_ENABLE_PROFILING=0  # or just don't set it

# On
# - Error tracking + hardware I/O profiling
# - ~1-2% overhead
# - Monitors if components meet timing deadlines
export OPENSOURCELEG_ENABLE_PROFILING=1
```

### Complete Disable

```bash
# Turn off Sentry entirely
export OPENSOURCELEG_DISABLE_SENTRY=1
```

## Performance Overhead

| Configuration | Overhead | What You Get |
|--------------|----------|--------------|
| **Default** (profiling off) | <1% CPU | Error tracking and stack traces |
| Profiling on | ~1-2% CPU | Error tracking + hardware I/O timing data |

**Key point**: Overhead is from CPU sampling, NOT network I/O (which is async and non-blocking).

## What Gets Profiled (When Enabled)

When `OPENSOURCELEG_ENABLE_PROFILING=1`:

- 10% of `actuator.update()` calls
- 10% of `sensor.update()` calls
- 10% of `set_motor_*()` calls (`set_motor_position`, `set_motor_current`, etc.)
- 10% of other hardware I/O operations

**Result**: You see if components meet timing requirements (e.g., <1ms for 1kHz control loop).

## Threading Model (Non-Blocking)

Sentry won't interfere with your control loops:

- **Profiling thread**: Separate thread samples call stacks at 100Hz
- **Transport worker**: Background thread sends data asynchronously
- **Network I/O**: Completely non-blocking
- **Queue**: Drops events if network is slow (no backpressure)

## How It Works

### Already Instrumented

The library already instruments key hardware I/O methods:

```python
# In opensourceleg/actuators/dephy.py
@trace_performance(op="actuator.update", description="Update Dephy actuator")
def update(self):
    # Hardware I/O: read from device, update thermal model
    pass

@trace_performance(op="actuator.set_motor_current", description="Set motor current")
def set_motor_current(self, value):
    # Hardware I/O: send command to device
    pass

# In opensourceleg/sensors/loadcell.py
@trace_performance(op="sensor.update", description="Update loadcell sensor")
def update(self, ...):
    # Hardware I/O: read from I2C, process data
    pass
```

### Using Instrumented Methods

Just use the library normally:

```python
from opensourceleg.actuators import DephyActuator

# Enable profiling to monitor timing
# export OPENSOURCELEG_ENABLE_PROFILING=1

actuator = DephyActuator(port="/dev/ttyACM0")
actuator.start()

# These calls are being profiled (10% sample rate)
actuator.update()  # Are we meeting our timing deadline?
actuator.set_motor_current(1000)  # How long does this take?
```

Check Sentry dashboard to see:
- Average time for `actuator.update()` (should be <1ms for 1kHz loop)
- P95/P99 latencies
- Bottlenecks in hardware communication

### Custom Components

If you create custom actuators or sensors, add the decorator:

```python
from opensourceleg.logging.profiling import trace_performance

class MyCustomActuator:
    @trace_performance(op="actuator.update", description="Update custom actuator")
    def update(self):
        # Your hardware I/O code
        pass

    @trace_performance(op="actuator.set_motor_position", description="Set position")
    def set_motor_position(self, position):
        # Your hardware command code
        pass
```

## Use Cases

### Production Deployment (Default)

```bash
# No environment variables needed - profiling is off by default
# Only error tracking, <1% overhead
```

**You get**: Error reports and stack traces when things go wrong.

### Development/Testing

```bash
export OPENSOURCELEG_ENABLE_PROFILING=1
```

**You get**:
- Error reports
- Hardware I/O timing data
- Verification that components meet deadlines
- ~1-2% overhead

### Debugging Performance Issues

```bash
export OPENSOURCELEG_ENABLE_PROFILING=1
# Run your control loop and check Sentry dashboard
```

Look for:
- `actuator.update()` timing → Should be <1ms for 1kHz
- `sensor.update()` timing → Should be <0.5ms
- `set_motor_*()` timing → Should be <0.1ms

If any component is too slow, you know where to optimize.

## FAQs

### Why only hardware I/O?

These are the operations that affect real-time performance. Everything else (initialization, user code, etc.) doesn't impact control loop timing.

### Will profiling affect my control loop?

No! Profiling is:
- **Non-blocking**: Happens on background thread
- **Asynchronous**: Network I/O doesn't block your code
- **Sampled**: Only 10% of calls are profiled
- **Low overhead**: ~1-2% CPU when enabled

### What if I see high overhead?

1. Check: `echo $OPENSOURCELEG_ENABLE_PROFILING`
2. Disable profiling: `unset OPENSOURCELEG_ENABLE_PROFILING` (or set to 0)
3. Restart your application
4. Overhead should drop to <1%

### Do I need to profile my control loop?

**No!** The library profiles component methods. If `actuator.update()` meets its deadline, your control loop will too. You don't need to instrument your own code.

### What operations should I profile?

Only hardware I/O:
- ✅ Component updates (`update()`)
- ✅ Motor commands (`set_motor_*()`)
- ✅ Hardware reads/writes
- ❌ Initialization
- ❌ User control logic
- ❌ Math/calculations

## Testing Overhead

Measure actual impact on your Raspberry Pi:

```python
from opensourceleg.utilities.profile import Profiler

# 1. Baseline (profiling off)
profiler = Profiler("control_loop")
for i in range(1000):
    profiler.tic()
    actuator.update()
    # ... your control code ...
    profiler.toc()

# Check mean/stddev

# 2. With profiling (set OPENSOURCELEG_ENABLE_PROFILING=1 and restart)
# Run again and compare
# Should be <2% slower
```

## Summary

**Default (production)**:
- Profiling: OFF
- Overhead: <1%
- You get: Error tracking

**Development (enable profiling)**:
```bash
export OPENSOURCELEG_ENABLE_PROFILING=1
```
- Profiling: ON (hardware I/O only)
- Overhead: ~1-2%
- You get: Error tracking + timing verification

**Simple, focused, and production-ready for Raspberry Pi! 🤖**
