# Iterator Usage Tutorial

This tutorial demonstrates how to use SoftRealtimeLoop as an iterator.

## Basic Iterator Pattern

The iterator interface provides a more Pythonic way to work with the loop:

```python
--8<-- "tutorials/utilities/using_iterators.py:5:13"
```

## Key Concepts

1. The `t` value represents the time since the loop started (in seconds)
2. Each iteration occurs at precise time intervals
3. The loop can be stopped by breaking the for loop, calling `rt_loop.stop()`, or sending an interrupt signal (Ctrl+C)

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.discourse.group).
