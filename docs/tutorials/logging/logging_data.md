# Logging Data

This guide explains how to use the Logger for data collection and variable tracking. Whether you're debugging your robot's behavior, collecting experimental data, or monitoring system performance, proper data logging is essential. All examples can be found in the [`logging_data.py`](https://github.com/neurobionics/opensourceleg/tree/main/tutorials/logging/logging_data.py) script.

## Variable Tracking

The Logger provides powerful variable tracking capabilities that let you monitor values over time. This is particularly useful for:

- Recording sensor readings
- Tracking control variables
- Monitoring system state
- Collecting experimental data

Here's how to set it up:

```python
--8<-- "tutorials/logging/logging_data.py:60:78"
```

In this example:

- `track_functions()` tells the logger to monitor a specific variable
    - This function takes one argument, a dictionary where the key defines the identifier/tag, the value defines the function to be called when `Logger.record()` is called
- `trace_variables()` tells the logger to monitor variables.
    - This function takes one argument, a dictionary where the key defines the identifier/tag, the value defines what will be logged
- Variables are added to a buffer every time `record()` is called


```python
--8<-- "tutorials/logging/logging_data.py:75:80"
```

## More Examples

### 1. Recording Sensor Data

This example demonstrates how to log data from a sensor in real-time. While this uses a simulated sensor, the pattern works for any sensor type:

```python
--8<-- "tutorials/logging/logging_data.py:6:16"
```

Key features demonstrated:

- Continuous data collection
- Real-time logging
- Error handling for sensor failures
- Proper resource cleanup

Example usage showing how to integrate this into your application:

```python
--8<-- "tutorials/logging/logging_data.py:87:101"
```

### 2. Experiment Data Collection

A more complex example showing how to structure data collection for an experiment, including proper resource management and error handling.

```python
--8<-- "tutorials/logging/logging_data.py:18:54"
```

Example usage:

```python
--8<-- "tutorials/logging/logging_data.py:110:112"
```

## Best Practices

1. **Variable Naming**

    - Use clear, descriptive names for tracked variables
    - Follow a consistent naming convention
    - Consider adding units to variable names (e.g., "angle_deg", "velocity_mps")

2. **Update Frequency**

    - Call `update()` at a consistent rate
    - Consider your data storage requirements
    - Balance logging frequency with system performance

3. **Resource Management**

    - Always use proper cleanup procedures
    - Consider using context managers (`with` statements)
    - Handle interruptions gracefully

4. **Data Organization**

    - Use meaningful file names
    - Structure your data logically
    - Include metadata when relevant

## Working with Logged Data

Your logged data can be easily analyzed using common data analysis packages like `pandas` and `matplotlib` or used with other programs like Matlab.

## Common Issues and Solutions

1. **High-Frequency Data**

    - Increase buffer size for better performance
    - Consider logging only essential variables
    - Use appropriate data types

2. **Large Files**

    - Split logs into manageable chunks
    - Clean up old logs regularly
    - Monitor disk space usage

3. **System Performance**

    - Profile your logging impact
    - Adjust buffer sizes as needed
    - Balance logging frequency with requirements

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
