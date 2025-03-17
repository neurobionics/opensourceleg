# Reading Sensors

This tutorial demonstrates how to read sensor data from a Dephy actuator using the `opensourceleg` library. You'll learn how to continuously monitor and log actuator sensor values.

## Overview

This example shows how to:

- Initialize a Dephy actuator for sensor reading
- Continuously monitor sensor values
- Log sensor data for analysis

## Prerequisites

- Raspberry Pi
- Python environment with `opensourceleg` package installed

## Hardware Setup

1. Dephy actuator connected via USB (typically at `/dev/ttyACM0`) and powered on

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/actuators/dephy/sensor_reading.py) is organized into three main sections:

### 1. Configuration

```python
--8<-- "tutorials/actuators/dephy/sensor_reading.py:1:7"
```

Key parameters:

- `FREQUENCY`: Sensor reading rate (1000 Hz)
- `GEAR_RATIO`: Actuator gear ratio (1.0)

### 2. Initialization

```python
--8<-- "tutorials/actuators/dephy/sensor_reading.py:9:23"
```

This section:

- Creates a data logger for recording measurements
- Initializes the Dephy actuator with specified parameters
- Sets up a real-time loop for consistent timing
- Configures which sensor variables to track

### 3. Main Loop

```python
--8<-- "tutorials/actuators/dephy/sensor_reading.py:24:29"
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
   python sensor_reading.py
   ```

3. Expected behavior:

   - Continuous reading of sensor values
   - Data logged to `./sensor_reading.log`
   - No active control (read-only operation)

## Analyzing the Results

The logged data includes:

- **Time**: Elapsed time in seconds
- **Motor Position**: Angular position of the motor in radians
- **Motor Current**: Current through the motor in mA

You can analyze this data to:

- Monitor actuator behavior
- Verify sensor functionality

## Additional Notes

- This script operates in read-only mode
- No control commands are sent to the actuator, feel free to move the actuator output and see the sensor values change

## Safety Considerations

⚠️ **Important Safety Notes**:

1. **Mechanical Safety**

   - Be aware that the actuator won't resist motion

2. **Electrical Safety**

   - Verify proper grounding

3. **Operation Safety**

   - Monitor sensor values for reasonable readings
   - Be ready to terminate the script (`Ctrl+C`)

## Troubleshooting

Common issues and solutions:

1. **No USB Connection**

   - Check if the device shows up in `/dev`
   - Verify USB permissions
   - Try unplugging and reconnecting

2. **Sensor Reading Issues**

   - Verify actuator power
   - Check USB connection stability
   - Ensure proper initialization

3. **Data Logging Problems**

   - Check write permissions in the log directory
   - Verify available disk space
   - Monitor logging frequency

## Available Sensor Readings

The Dephy actuator provides several sensor values that can be tracked, you can find the list of all the available sensor readings in the [Actuator API](https://neurobionics.github.io/opensourceleg/api/actuators/dephy/index.html) documentation.

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
