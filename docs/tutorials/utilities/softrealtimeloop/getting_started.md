# Time Module Tutorial

The Time module provides tools for creating soft real-time loops in Python, which is particularly useful for robotics and control applications. The module consists of two main classes:

- `LoopKiller`: Handles graceful shutdown of loops
- `SoftRealtimeLoop`: Manages timing-precise execution of functions

## Core Features

- Precise timing control with configurable time steps
- Graceful shutdown handling (including CTRL+C)
- Fade-out capability for smooth transitions
- Performance statistics reporting
- Iterator-based interface

## Basic Usage

The simplest way to use the SoftRealtimeLoop is to create a loop with a specific time step and run a function. See the [Running Functions](running_functions.md) tutorial for more details.

```python
from opensourceleg.utilities import SoftRealtimeLoop

rt_loop = SoftRealtimeLoop(dt=0.001)  # 1ms time step
rt_loop.run(your_function)
```

## Iterator Interface

The module provides an iterator interface for more flexible control. See the [Using Iterators](using_iterators.md) tutorial for more details.

```python
rt_loop = SoftRealtimeLoop(dt=0.1)  # 10Hz loop
for t in rt_loop:
    print(f"Time: {t:.1f}s")
```

## Advanced Features

The module supports advanced features like:

- Fade-out capabilities
- Performance monitoring

Check out the [Real-time Control](realtime_control.md) tutorial for more details.

## Key Parameters

When creating a `SoftRealtimeLoop`, you can specify:

- `dt`: Time step in seconds (default: 0.001)
- `report`: Enable/disable performance reporting (default: True)
- `fade`: Fade-out duration in seconds (default: 0.0)
- `maintain_original_phase`: Flag to try to maintain the original loop schedule created at startup (default: False)

## Interrupt Handling

The loop provides built-in handling for:

- `SIGTERM` signals
- `SIGINT` signals (Ctrl+C)
- `SIGHUP` signals (where available)

## Performance Monitoring

When reporting is enabled, the loop provides statistics on:

- Average timing error
- Standard deviation of timing error
- Percentage of time spent sleeping

## Note on Maintaining Original Phase

The `SoftRealtimeLoop` can operate in two modes, controlled by the `maintain_original_phase` parameter:

1. **Consistent Time Step (`maintain_original_phase=False` (default))**:
    - In this mode, the loop focuses on maintaining a consistent time step between iterations, regardless of any delays or errors in previous iterations.
    - This mode can be helpful in real-time control where you don't want previous loop errors to impact the current execution rate.
    - Example:
      ```python
      rt_loop = SoftRealtimeLoop(dt=0.01, maintain_original_phase=False)
      for t in rt_loop:
          print(f"Time: {t:.3f}s")
      ```
2. **Maintain Original Phase (`maintain_original_phase=True`)**:
    - In this mode, the loop attempts to ensure that the time elapsed since the start of the loop aligns with the expected schedule `(loop_number * dt)`.
    - This is useful for applications where precise timing relative to the start of the loop is critical, such as synchronized control systems where the total time is important to maintain.
    - If the loop falls behind schedule, it will attempt to catch up by reducing sleep time in subsequent iterations.
    - Example:
      ```python
      rt_loop = SoftRealtimeLoop(dt=0.01, maintain_original_phase=True)
      for t in rt_loop:
          print(f"Time: {t:.3f}s")
      ```

### Key Differences:
- **Original Phase Mode**: Prioritizes alignment with the overall schedule, even if it means adjusting the timing of individual iterations.
- **Consistent Time Step Mode**: Prioritizes uniform spacing between iterations, even if it deviates from the overall schedule.

Choose the mode that best suits your application's requirements. Most often, we use the default mode for OSL controllers.

## Next Steps

1. Start with the [Running Functions](running_functions.md) to learn fundamental concepts
2. Explore the [Using Iterators](using_iterators.md) for more flexible control
3. Check out the [Real-time Control](realtime_control.md) for complex examples
