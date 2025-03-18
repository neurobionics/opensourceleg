# Logging Data

This guide explains how to use the Logger for data collection and variable tracking. All examples can be found in the [`logging_data.py`](../tutorials/logging/logging_data.py) script.

## Variable Tracking

Track simple variables and class attributes with the Logger. This allows you to monitor values over time and automatically record them to your log files.

```python
--8<-- "tutorials/logging/logging_data.py:60:78"
```

### Basic CSV Logging

The Logger will automatically generate CSV files containing your tracked variables. Each update creates a new row in the CSV file, making it easy to analyze your data later.

```python
--8<-- "tutorials/logging/logging_data.py:79:81"
```

## Real-World Examples

### 1. Recording Sensor Data

This example shows how to log data from a simulated sensor, demonstrating real-time data collection and logging.

```python
--8<-- "tutorials/logging/logging_data.py:5:16"
```

Example usage:

```python
--8<-- "tutorials/logging/logging_data.py:90:108"
```

### 2. Experiment Data Collection

A more complex example showing how to structure data collection for an experiment, including proper resource management and error handling.

```python
--8<-- "tutorials/logging/logging_data.py:18:54"
```

Example usage:

```python
--8<-- "tutorials/logging/logging_data.py:116:118"
```

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
