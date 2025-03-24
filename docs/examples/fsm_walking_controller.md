# Finite State Machine Controller

## Overview

The library ships with three example implementations of the same finite state machine (FSM) walking controller. The figure below shows the basic execution of the controller: 

![A diagram of the finite state machine](./assets/FSM_Diagram.svg)


The first implementation is entirely in Python and uses the `StateMachine` class from the control subpackage of this library. If you plan to write your controllers exclusively in Python, this example is a good starting point.

The library also supports using compiled `C` and `C++` library functions via the `CompiledController` class. You can see a basic example of this module on the [tutorials page](/opensourceleg/tutorials/control/compiled_controller/), which may be helpful to review before starting with this example. We've duplicated the FSM behavior in both `C++` and `MATLAB`. The source code for these control implementations is available in [this repository](https://github.com/neurobionics/OSL_CompiledControllers_Source). Refer to the documentation in that repository for instructions on compiling both the `C++` and the `MATLAB` source code. 

---

## Python Implementation

Documentation coming soon!

---

## C++ and MATLAB Implementation

To get started, ensure you have compiled either the `C++` or `MATLAB` source code and have a `FSMController.so` library. If not, refer to the [source repository](https://github.com/neurobionics/OSL_CompiledControllers_Source) for compilation instructions. To run this example, make sure the generated library is in the same directory as your script, or modify the search path for the library when loading the controller.

---

### Load Compiled Library

First, perform standard imports, handle paths, and set up the hardware for the OSL joints and sensors

```python
--8<-- "examples/fsm_walking_compiled_controller.py:11:59"
```

> **Note**:  
> If instantiating the OSL hardware and sensors is unfamiliar, check out the [the tutorials pages](/opensourceleg/tutorials/actuators/getting_started/).

Next, instantiate a `CompiledController` wrapper object:

```python
--8<-- "examples/fsm_walking_compiled_controller.py:61:68"
```

---

### Define Custom Datatypes

Define the data structures used in the controller. These must match the size and order of the structures used to create the library.

```python
--8<-- "examples/fsm_walking_compiled_controller.py:70:121"
```

---

### Configure Impedance and Transition Parameters

Configure the impedance and transition parameters based on predefined tuning. Adjust these values to achieve the desired behavior:

```python
--8<-- "examples/fsm_walking_compiled_controller.py:124:159"
```

---

### Main Loop

After configuration, home the OSL joints, calibrate the loadcell, set the joints to impedance mode, and begin running the controller. During each loop iteration, update the inputs, call the `run()` method, and write outputs to the hardware:

```python
--8<-- "examples/fsm_walking_compiled_controller.py:161:209"
```

> **Note**:  
> Be careful with units when writing outputs to the hardware. Convert values to the appropriate units if necessary.

---

### Full Code for This Example

```python
--8<-- "examples/fsm_walking_compiled_controller.py"
```