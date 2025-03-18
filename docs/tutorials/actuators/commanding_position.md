# Commanding Position

This tutorial demonstrates how to command position to a Dephy actuator using the `opensourceleg` library. You'll learn how to implement a basic step response test by commanding a position setpoint.

## Overview

Position control allows direct control of the actuator's angular position. This example shows how to:

- Initialize a Dephy actuator in position control mode
- Command a position step input
- Log and monitor the actuator's response

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/actuators/dephy/commanding_position.py) is organized into four main sections:

### 1. Configuration

```python
--8<-- "tutorials/actuators/dephy/commanding_position.py:1:14"
```

Key parameters:

- `TIME_TO_STEP`: Delay before position step (1.0 second)
- `FREQUENCY`: Control loop rate (1000 Hz)
- `GEAR_RATIO`: Actuator gear ratio (1.0)

### 2. Initialization

```python
--8<-- "tutorials/actuators/dephy/commanding_position.py:15:24"
```

This section:

- Creates a data logger for recording measurements
- Initializes the Dephy actuator with specified parameters
- Sets up a real-time loop for consistent timing

### 3. Control Setup

```python
--8<-- "tutorials/actuators/dephy/commanding_position.py:25:36"
```

Before the main loop, we:

- Configure the actuator for position control mode
- Initialize position control gains
- Get initial position and set up command position
- Configure logging variables for tracking positions

### 4. Control Loop

```python
--8<-- "tutorials/actuators/dephy/commanding_position.py:37:46"
```

The main loop:

1. Starts at current position
2. After `TIME_TO_STEP`, commands a π/2 radian (90 degree) position step
3. Updates actuator state
4. Logs time and position data

## Running the Example

1. Navigate to the tutorial directory:

   ```bash
   cd tutorials/actuators/dephy
   ```

2. Run the script:

   ```bash
   python commanding_position.py
   ```

3. Expected behavior:

   - t < 1.0s: Motor maintains initial position
   - t ≥ 1.0s: Motor moves to position + π/2 radians (90 degrees)
   - Data is continuously logged to `./logs/commanding_position.csv`

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
