# Commanding Current

This tutorial demonstrates how to command current to a Dephy actuator using the `opensourceleg` library. You'll learn how to implement a basic step response test by commanding a current setpoint.

## Overview

Current control is fundamental for motor control applications. This example shows how to:

- Initialize a Dephy actuator in current control mode
- Command a current step input
- Log and monitor the actuator's response

## Prerequisites

- Raspberry Pi
- Python environment with `opensourceleg` package installed

## Hardware Setup

1. External power supply connected to the actuator
2. Dephy actuator connected via USB (typically at `/dev/ttyACM0`) and powered on
3. Ensure the actuator is securely mounted if testing with loads

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/actuators/dephy/current_control.py) is organized into four main sections:

### 1. Configuration

```python
--8<-- "tutorials/actuators/dephy/current_control.py:1:10"
```

Key parameters:

- `TIME_TO_STEP`: Delay before applying current (1.0 second)
- `FREQUENCY`: Control loop rate (1000 Hz)
- `CURRENT_SETPOINT`: Target current (600 mA)

### 2. Initialization

```python
--8<-- "tutorials/actuators/dephy/current_control.py:13:20"
```

This section:

- Creates a data logger for recording measurements
- Initializes the Dephy actuator with specified parameters
- Sets up a real-time loop for consistent timing

### 3. Control Setup

```python
--8<-- "tutorials/actuators/dephy/current_control.py:25:32"
```

Before the main loop, we:

- Configure the actuator for current control mode
- Initialize current control gains
- Set up variables for tracking commanded and measured current

### 4. Control Loop

```python
--8<-- "tutorials/actuators/dephy/current_control.py:34:45"
```

The main loop:

1. Starts with zero current
2. After `TIME_TO_STEP`, commands `CURRENT_SETPOINT`
3. Updates actuator state
4. Logs time, commanded current, and measured current

## Running the Example

1. Navigate to the tutorial directory:

   ```bash
   cd tutorials/actuators/dephy
   ```

2. Run the script:

   ```bash
   python current_control.py
   ```

3. Expected behavior:

   - t < 1.0s: Motor maintains 0 mA
   - t ≥ 1.0s: Motor steps to 600 mA
   - Data is continuously logged to `./logs/current_control.csv`

## Analyzing the Results

The logged data includes:

- **Time**: Elapsed time in seconds
- **Command Current**: Desired current in mA
- **Motor Current**: Actual measured current in mA

You can analyze this data to:

- Verify current tracking performance
- Measure step response characteristics
- Identify any control issues

## Safety Considerations

⚠️ **Important Safety Notes**:

1. **Mechanical Safety**

   - Ensure the actuator is properly mounted
   - Keep clear of moving parts
   - Have an emergency stop plan

2. **Electrical Safety**

   - Don't exceed rated current limits
   - Ensure proper power supply connection

3. **Operation Safety**

   - Start with lower currents when testing
   - Be ready to terminate the script (`Ctrl+C`)
   - Verify sensor readings are reasonable

## Troubleshooting

Common issues and solutions:

1. **No USB Connection**

   - Check if the device shows up in `/dev`
   - Verify USB permissions
   - Try unplugging and reconnecting

2. **Unexpected Current Readings**

   - Verify power supply connection
   - Confirm gain settings

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
