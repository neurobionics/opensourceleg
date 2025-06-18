# FSM Walking Ankle Controller

## Overview

This example demonstrates how to use a finite state machine (FSM) to control the OSL ankle joint with impedance control. The FSM uses load cell feedback and the motor encoder (or optionally the joint encoder) to transition between states, applying different impedance parameters for each phase of the gait cycle. This script is intended for testing and development with the OSL hardware.

The figure below shows the basic execution of the controller:

![A diagram of the finite state machine](./assets/FSM_Diagram.svg)

The implementation is entirely in Python and uses the `StateMachine` class from the control subpackage of this library.

---

## Python Implementation

### Setup and Configuration

First, perform the necessary imports and define the hardware and control parameters:

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py:1:38"
```

Next, define the tunable FSM parameters for each state and the transition thresholds:

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py:40:74"
```
> **Note**:
> These parameters were roughly tuned for a moderately paced walking gait. You may want to tune them to better suit your intended use case.

### FSM Definition

Next, we create a function that returns an instance of the `StateMachine` class. Create the FSM by defining the states, their impedance parameters, and the transition criteria functions. Each transition is based on sensor feedback or joint state:

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py:76:137"
```

> **Note**:
> If instantiating the OSL hardware and sensors is unfamiliar, check out the [the tutorials pages](../tutorials/actuators/getting_started.md).

### Hardware Initialization

Initialize the actuators, sensors, logger, and realtime loop:

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py:139:170"
```

### Main Loop

Home the OSL, set the ankle to impedance mode, and start the main control loop. In each iteration, update the hardware, FSM, and log the relevant data:

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py:172:end"
```

---

### Full Code

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py"
```

---

## Notes

- This example is designed for use with the OSL hardware and requires the appropriate sensors and actuators to be connected.
- The FSM transitions and impedance parameters may need to be tuned for your specific application or hardware setup.
- For more information on the OSL library and hardware setup, refer to the [tutorials](../tutorials/actuators/getting_started.md).
