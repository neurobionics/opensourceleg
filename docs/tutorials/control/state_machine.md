# State Machine Tutorial - Robot Vaccum

The `opensourceleg.control` module provides functionality for creating and managing finite state machines (FSMs) via the `StateMachine` class. This tutorial walks you through the implementation of a simple state machine for a robot vaccum cleaner using the `finite_state_machine.py` example.

## Overview
This tutorial demonstrates how to define states, transitions, and criteria for switching between states using the `StateMachine` class.

## Define Transition Criteria

First, we define the transition criteria of the FSM after importing the pertinent classes. These are functions that return a boolean value based on some condition. You can define any arguments you like, so here we will use the `battery_level` to determine when to switch states.

```python
--8<-- "tutorials/control/finite_state_machine.py:1:18"
```

## Define States and State Machine
Next, we define three states for our system: cleaning, charging, and docking. For the docking state, we assume that it takes 10 seconds to go from anywhere in the room to the docking station. We add each of the states as a list to a new instance of the `StateMachine` module and set the docking state as the initial state.

```python
--8<-- "tutorials/control/finite_state_machine.py:29:35"
```

## Add Transitions Between States
Now that the state machine is built, we need to link the states with transitions. We do this using the `add_transition()` method. Transitions specify the conditions under which the FSM moves from one state to another. Each transition includes:

- A source state
- A destination state
- An event name
- A criteria (a function that returns `True` when the transition should occur)

```python
--8<-- "tutorials/control/finite_state_machine.py:37:53"
```

## Main Loop
We initialize the FSM inside a `with` context and create an instance of `SoftRealtimeLoop` to simulate our state machine. Each time through the loop, we call `fsm.update(battery_level=battery_level)`, which provides the appropriate inputs to the transition functions.

```python
--8<-- "tutorials/control/finite_state_machine.py:55:76"
```

## Example Output

When running the example, you can expect the following behavior:

1. The FSM starts in the `Charging` state.
2. Once the battery is fully charged, it transitions to the `Cleaning` state.
3. When the battery level drops below 20%, it transitions to the `Docking` state.
4. After docking, it transitions back to the `Charging` state.

This cycle repeats indefinitely, simulating the behavior of the robot vacuum cleaner.

## Full Script for This Tutorial
```python
--8<-- "tutorials/control/finite_state_machine.py"
```
