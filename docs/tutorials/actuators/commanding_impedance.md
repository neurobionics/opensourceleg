# Commanding Impedance

This tutorial demonstrates how to implement impedance control with a Dephy actuator using the `opensourceleg` library. You'll learn how to command position setpoints while maintaining compliant behavior through impedance control.

## Overview

Impedance control allows for position control while maintaining a specified dynamic relationship between position and force. This example shows how to:

- Initialize a Dephy actuator in impedance control mode
- Command a position step input
- Monitor position tracking and motor current

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/actuators/dephy/commanding_impedance.py) is organized into four main sections:

### 1. Configuration

```python
--8<-- "tutorials/actuators/dephy/commanding_impedance.py:1:13"
```

Key parameters:

- `TIME_TO_STEP`: Delay before position step (1.0 second)
- `FREQUENCY`: Control loop rate (1000 Hz)
- `GEAR_RATIO`: Actuator gear ratio (1.0)

### 2. Initialization

```python
--8<-- "tutorials/actuators/dephy/commanding_impedance.py:16:26"
```

This section:

- Creates a data logger for recording measurements
- Initializes the Dephy actuator with specified parameters
- Sets up a real-time loop for consistent timing

### 3. Control Setup

```python
--8<-- "tutorials/actuators/dephy/commanding_impedance.py:28:39"
```

Before the main loop, we:

- Configure the actuator for impedance control mode
- Initialize impedance control gains
- Get initial position and set up command position
- Configure logging variables for position and current

### 4. Control Loop

```python
--8<-- "tutorials/actuators/dephy/commanding_impedance.py:41:56"
```

The main loop:

1. Starts at current position
2. After `TIME_TO_STEP`, commands a π/2 radian (90 degree) position step
3. Updates actuator state
4. Logs time, positions, and motor current

## Running the Example

1. Navigate to the tutorial directory:

      ```bash
      cd tutorials/actuators/dephy
      ```

2. Run the script:

      ```bash
      python commanding_impedance.py
      ```

3. Expected behavior:
      - t < 1.0s: Motor maintains initial position
      - t ≥ 1.0s: Motor moves to position + π/2 radians
      - Movement will be compliant due to impedance control
      - Data is continuously logged to `./logs/commanding_impedance.csv`

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.discourse.group).
