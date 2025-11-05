# Basic Usage Tutorial

This tutorial explains the fundamental usage of the SoftRealtimeLoop class.

## Simple One-Shot Function

The most basic usage is running a function once in the loop. The function returns 0 to stop the loop:

```python
--8<-- "tutorials/utilities/running_functions.py:7:16"
```

## Timed Execution

You can run a function for a specific duration by tracking elapsed time:

```python
--8<-- "tutorials/utilities/running_functions.py:20:29"
```

## Key Concepts

1. The `run()` method is blocking and continues until the function returns or a signal interrupts the loop.
2. The time step (`dt`) determines the loop frequency
3. The loop maintains precise timing automatically

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.discourse.group).
