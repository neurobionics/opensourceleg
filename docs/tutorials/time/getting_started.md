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

The simplest way to use the SoftRealtimeLoop is to create a loop with a specific time step and run a function. See the [Basic Usage Tutorial](basic_usage.md) tutorial for more details.

```python
from opensourceleg.time import SoftRealtimeLoop

rt_loop = SoftRealtimeLoop(dt=0.001)  # 1ms time step
rt_loop.run(your_function)
```

## Iterator Interface

The module provides an iterator interface for more flexible control. See the [Iterator Usage Tutorial](iterator_usage.md) tutorial for more details.

```python
rt_loop = SoftRealtimeLoop(dt=0.1)  # 10Hz loop
for t in rt_loop:
    print(f"Time: {t:.1f}s")
```

## Advanced Features

The module supports advanced features like:

- Fade-out capabilities
- Performance monitoring

Check out the [Advanced Usage Tutorial](advanced_usage.md) tutorial for more details.

## Key Parameters

When creating a `SoftRealtimeLoop`, you can specify:

- `dt`: Time step in seconds (default: 0.001)
- `report`: Enable/disable performance reporting (default: True)
- `fade`: Fade-out duration in seconds (default: 0.0)

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

## Next Steps

1. Start with the [Basic Usage Tutorial](basic_usage.md) to learn fundamental concepts
2. Explore the [Iterator Usage Tutorial](iterator_usage.md) for more flexible control
3. Check out the [Advanced Usage Tutorial](advanced_usage.md) for complex examples
