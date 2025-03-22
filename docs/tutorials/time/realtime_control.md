# Advanced Usage Tutorial

This tutorial covers advanced features of SoftRealtimeLoop

## Sensor Data Acquisition

Example of reading from a simulated sensor at precise intervals:

```python
--8<-- "tutorials/time/realtime_control.py:7:35"
```

## Graceful Shutdown with Fade-out

Demonstration of the fade-out feature for a smooth shutdown:

```python
--8<-- "tutorials/time/realtime_control.py:38:57"
```

## Understanding Performance Statistics

When a SoftRealtimeLoop completes (either through normal termination or interruption), it prints detailed timing statistics if reporting is enabled (default). Let's understand what these metrics mean and how to use them:

### Available Statistics

The loop reports three key metrics:

```python
# Example output:
In 1000 cycles at 100.00 Hz:
    avg error: 0.123 milliseconds
    stddev error: 0.045 milliseconds
    percent of time sleeping: 85.2 %
```

1. **Average Error**: Shows how far off your timing is from the target on average

    - Values under 1ms are typically good for most applications
    - Higher values indicate your loop might be overloaded

2. **Standard Deviation Error**: Shows how consistent your timing is

    - Lower values mean more consistent timing
    - High variance might indicate interference from other processes

3. **Percent Time Sleeping**: Shows how much CPU headroom you have

    - Higher values (>50%) indicate your loop has plenty of processing time
    - Low values suggest your loop might be struggling to keep up

### Improving Performance

If your statistics show timing issues, here are some general ways to improve your real-time performance:

   - Reduce other system load
   - Increase process priority
   - Move computationally intensive operations outside the loop

### Monitoring Performance

You can access timing statistics programmatically:

```python
rt_loop = SoftRealtimeLoop(dt=0.001)
# ... run your loop ...
avg_error = rt_loop.sum_err / rt_loop.n if rt_loop.n > 0 else 0
sleep_percentage = (rt_loop.sleep_t_agg / rt_loop.time_since_start) * 100 if rt_loop.time_since_start > 0 else 0
```

By monitoring these statistics, you can ensure your real-time applications maintain precise timing and reliable performance. The statistics help identify when your loop needs tuning or when system resources are becoming constrained.

### Best Practices

1. **Choose Appropriate Time Steps**:

    - Faster loops (smaller dt) require more CPU resources
    - Balance between control requirements and system capabilities

2. **Monitor Resource Usage**:

    - Watch for sleep percentages below 20%
    - Consider operating at a lower frequency if the system is struggling

3. **Use Fade-out for Clean Graceful Shutdown**:

    ```python
    rt_loop = SoftRealtimeLoop(dt=0.01, fade=1.0)  # 1-second fade-out
    ```

4. **Reset for Multiple Uses**:

    ```python
    rt_loop.reset()  # Clears statistics for new measurements
    ```

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
