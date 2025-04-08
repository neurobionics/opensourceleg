# Reading Sensors

This tutorial demonstrates how to read sensor data from a Dephy actuator using the `opensourceleg` library. You'll learn how to continuously monitor and log actuator sensor values.

## Overview

This example shows how to:

- Initialize a Dephy actuator for sensor reading
- Continuously monitor sensor values
- Log sensor data for analysis

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/actuators/dephy/reading_sensor_data.py) is organized into three main sections:

### 1. Configuration

```python
--8<-- "tutorials/actuators/dephy/reading_sensor_data.py:1:7"
```

Key parameters:

- `FREQUENCY`: Sensor reading rate (1000 Hz)
- `GEAR_RATIO`: Actuator gear ratio (1.0)

### 2. Initialization

```python
--8<-- "tutorials/actuators/dephy/reading_sensor_data.py:9:23"
```

This section:

- Creates a data logger for recording measurements
- Initializes the Dephy actuator with specified parameters
- Sets up a real-time loop for consistent timing
- Configures which sensor variables to track

### 3. Main Loop

```python
--8<-- "tutorials/actuators/dephy/reading_sensor_data.py:24:29"
```

The main loop:

1. Updates actuator state to get fresh sensor readings
2. Logs time and sensor values
3. Updates the logger

## Running the Example

1. Navigate to the tutorial directory:

      ```bash
      cd tutorials/actuators/dephy
      ```

2. Run the script:

      ```bash
      python reading_sensor_data.py
      ```

3. Expected behavior:

      - Continuous reading of sensor values
      - Data logged to `./logs/reading_sensor_data.csv`
      - No active control (read-only operation)

## Additional Notes

- This script operates in read-only mode
- No control commands are sent to the actuator, feel free to move the actuator output and see the sensor values change

## Available Sensor Readings

The Dephy actuator provides several sensor values that can be tracked, you can find the list of all the available sensor readings in the [Actuator API](../../api/actuators/actuators.md) documentation.

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
