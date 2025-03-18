# Commanding Voltage

This tutorial demonstrates how to command voltage to a Dephy actuator using the `opensourceleg` library. You'll learn how to implement a basic step response test by commanding a voltage setpoint.

## Overview

Voltage control is the most basic form of motor control. This example shows how to:

- Initialize a Dephy actuator in voltage control mode
- Command a voltage step input
- Log and monitor the actuator's response

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/actuators/dephy/commanding_voltage.py) is organized into four main sections:

### 1. Configuration

```python
--8<-- "tutorials/actuators/dephy/commanding_voltage.py:1:12"
```

Key parameters:

- `TIME_TO_STEP`: Delay before applying voltage (1.0 second)
- `FREQUENCY`: Control loop rate (1000 Hz)
- `GEAR_RATIO`: Actuator gear ratio (1.0)

### 2. Initialization

```python
--8<-- "tutorials/actuators/dephy/commanding_voltage.py:12:34"
```

This section:

- Creates a data logger for recording measurements
- Initializes the Dephy actuator with specified parameters
- Sets up a real-time loop for consistent timing

### 3. Control Setup

```python
--8<-- "tutorials/actuators/dephy/commanding_voltage.py:36:38"
```

Before the main loop, we:

- Initialize command voltage to zero
- Set up logging variables for voltage and time
- Configure the real-time loop

### 4. Control Loop

```python
--8<-- "tutorials/actuators/dephy/commanding_voltage.py:39:50"
```

The main loop:

1. Starts with zero voltage
2. After `TIME_TO_STEP`, commands 1000 mV
3. Updates actuator state
4. Logs time, motor voltage, and motor current

## Running the Example

1. Navigate to the tutorial directory:

   ```bash
   cd tutorials/actuators/dephy
   ```

2. Run the script:

   ```bash
   python commanding_voltage.py
   ```

3. Expected behavior:

   - t < 1.0s: Motor maintains 0 mV
   - t ≥ 1.0s: Motor steps to 1000 mV
   - Data is continuously logged to `./logs/commanding_voltage.csv`

## Additional Notes

- Voltage control provides no feedback regulation
- Motor speed will vary with load under constant voltage
- This mode is useful for basic testing and characterization of the motor

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
