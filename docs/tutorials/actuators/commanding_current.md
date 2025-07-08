# Commanding Current

This tutorial demonstrates how to command current to a Dephy actuator using the `opensourceleg` library. You'll learn how to implement a basic step response test by commanding a current setpoint.

## Overview

Current control is fundamental for motor control applications. This example shows how to:

- Initialize a Dephy actuator in current control mode
- Command a current step input
- Log and monitor the actuator's response

## Code Structure

The [tutorial script](https://github.com/neurobionics/opensourceleg/blob/main/tutorials/actuators/dephy/commanding_current.py) is organized into four main sections:

### 1. Configuration

```python
--8<-- "tutorials/actuators/dephy/commanding_current.py:1:10"
```

Key parameters:

- `TIME_TO_STEP`: Delay before applying current (1.0 second)
- `FREQUENCY`: Control loop rate (1000 Hz)
- `CURRENT_SETPOINT`: Target current (600 mA)

### 2. Initialization

```python
--8<-- "tutorials/actuators/dephy/commanding_current.py:13:20"
```

This section:

- Creates a data logger for recording measurements
- Initializes the Dephy actuator with specified parameters
- Sets up a real-time loop for consistent timing

### 3. Control Setup

```python
--8<-- "tutorials/actuators/dephy/commanding_current.py:25:32"
```

Before the main loop, we:

- Configure the actuator for current control mode
- Initialize current control gains
- Set up variables for tracking commanded and measured current

### 4. Control Loop

```python
--8<-- "tutorials/actuators/dephy/commanding_current.py:34:45"
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
      python commanding_current.py
      ```

3. Expected behavior:
      - t < 1.0s: Motor maintains 0 mA
      - t ≥ 1.0s: Motor steps to 600 mA
      - Data is continuously logged to `./logs/commanding_current.csv`

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
