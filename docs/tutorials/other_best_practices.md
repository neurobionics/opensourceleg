# Best Practices

## Softstart

In robotic prostheses (as well as in general robotics), startup and initialization can often produce abrupt changes in motor torque that are undesirable. These sudden step changes in torque can be unsettling to the user and unececssarily hard on the hardware. To address this, we recommend implementing a **softstart** mechanism. This approach ensures a smooth ramp-up of torque at startup by scaling the control parameters, such as torque, impedance gains, or position gains, over a specified period.

Although the `opensourceleg` library does not include a built-in softstart feature, it is straightforward to implement using the `SaturatingRamp` class from the math submodule. Below, we outline the general procedure and provide an example script for a position control implementation. Similar effects for impedance, torque, and velocity control can be achieved following the same steps.

## Implementation Steps

1. **Define a `SaturatingRamp` Instance**
   Create an instance of the `SaturatingRamp` class from the `opensourceleg.math.math` module, specifying the desired softstart duration (e.g., `SOFT_START_TIME = 1.5` seconds).

2. **Scale Control Parameters**
   During each iteration of your control loop, use the output of the `SaturatingRamp` instance to scale your control parameters (e.g., torque commands, impedance gains, or position gains). After the ramp duration, the `SaturatingRamp` object outputs a value of `1`, leaving your parameters unchanged for the remainder of the execution.

## Example: Position Control with Softstart

Here, we walk through an example script demonstrating softstart in a position control paradigm.

### Key Code Snippets

1. **Initialize the `SaturatingRamp` Instance**
   The following snippet shows how to create a `SaturatingRamp` instance with the desired softstart time:
```python
--8<-- "tutorials/other/softstart_position.py:24:24"
```
2. **Scale Control Gains**
   During each loop iteration, call the ramp's update method and scale the control gains with the result:

```python
--8<-- "tutorials/other/softstart_position.py:33:40"
```

Here is the full script:
```python
--8<-- "tutorials/other/softstart_position.py:"
```
