# FSM Walking Ankle Controller

## Overview

This example demonstrates how to use a finite state machine (FSM) to control the OSL ankle joint with impedance control. The FSM uses load cell feedback and the motor encoder (or optionally the joint encoder) to transition between states, applying different impedance parameters for each phase of the gait cycle. This script is intended for testing and development with the OSL hardware.

The FSM implements four distinct gait phases:

1. **Early Stance**: Initial foot contact with moderate stiffness for shock absorption
2. **Late Stance**: High stiffness for push-off power generation during heel rise
3. **Early Swing**: Low stiffness allowing rapid dorsiflexion for ground clearance
4. **Late Swing**: Maintains dorsiflexion position preparing for next heel strike

The figure below shows the basic execution of the controller:

![A diagram of the finite state machine](./assets/FSM_Diagram.svg)

The implementation is entirely in Python and uses the `StateMachine` class from the control subpackage of this library.

---

## Python Implementation

### Setup and Configuration

First, perform the necessary imports and define the hardware and control parameters:

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py:1:34"
```

Next, define the tunable FSM parameters for each state and the transition thresholds:

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py:36:63"
```
> **Note**:
> These parameters were roughly tuned for a moderately paced walking gait. You may want to tune them to better suit your intended use case.

### FSM Definition

The FSM consists of four states representing the phases of the gait cycle, each with specific impedance parameters. We first define the transition criteria functions that determine when to switch between states based on sensor feedback:

#### Transition Functions

The transition criteria are implemented as functions that take the OSL object and return a boolean indicating whether the transition should occur:

- **Early Stance → Late Stance**: Triggered when load cell force exceeds threshold AND ankle position is dorsiflexed enough
- **Late Stance → Early Swing**: Triggered when load cell force drops below threshold (heel off)
- **Early Swing → Late Swing**: Triggered when ankle reaches target dorsiflexion AND velocity decreases
- **Late Swing → Early Stance**: Triggered when load cell force increases (heel strike) OR ankle plantarflexes sufficiently

#### State Machine Creation

Next, we create a function that returns an instance of the `StateMachine` class. Create the FSM by defining the states, their impedance parameters, and the transition criteria functions. Each transition is based on sensor feedback or joint state:

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py:66:182"
```

> **Note**:
> If instantiating the OSL hardware and sensors is unfamiliar, check out the [the tutorials pages](../tutorials/actuators/getting_started.md).

### Hardware Initialization

Initialize the required hardware components including the Dephy actuator, load cell sensor, data logger, and real-time control loop. The system uses a 200Hz control frequency for responsive control:

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py:184:218"
```

### Main Loop

The main control loop homes the OSL, sets the ankle to impedance mode, and runs the control algorithm. In each iteration, the system updates hardware sensors, evaluates FSM transitions, applies impedance control, and logs relevant data:

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py:220:end"
```

---

### Full Code

```python
--8<-- "examples/fsm_walking_ankle_python_controller.py"
```

---

## Notes

### Hardware Requirements
- This example is designed for use with the OSL hardware and requires the appropriate sensors and actuators to be connected.
- A 6-axis load cell (NBLoadcellDAQ) is required for ground reaction force measurements.
- The Dephy actuator must be properly configured and connected via USB.

### Parameter Tuning
- The FSM transitions and impedance parameters may need to be tuned for your specific application or hardware setup.
- Body weight scaling affects the load thresholds - adjust `BODY_WEIGHT` parameter accordingly

### Additional Resources
- For more information on the OSL library and hardware setup, refer to the [tutorials](../tutorials/actuators/getting_started.md).
