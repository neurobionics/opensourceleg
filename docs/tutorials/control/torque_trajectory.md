# Torque Trajectory Control Tutorial

This `opensourceleg.control` module provides functiionality for doing torque control. This tutorial demonstrates how to control the Open Source Leg (OSL) using torque trajectories for both the knee and ankle joints.
## Warnings:

1. This example is not meant to be used as a walking controller. The goal of this example is to provide a reference for how a torque trajectory can be loaded and commanded.
2. While runnig this script make sure to have load on the actuators.
3. Please be cautious while changing mass parameters.

## Overview

The script implements a torque control system that:

1. Loads pre-defined torque trajectories for knee and ankle joints
2. Applies these trajectories in a cyclic manner
3. Logs the performance data
4. Generates visualization plots

## Prerequisites

- OpenSourceLeg hardware setup
- Python environment with required dependencies:

      - numpy
      - matplotlib

- Access to torque trajectory files:

      - `ankle.pkl`
      - `knee.pkl`

## Command Line Arguments

The script accepts the following command line arguments:

- `--mass`: User mass in kg (default: 1.0)
- `--stride-time`: Stride time in seconds (default: 1.0)
- `--frequency`: Control loop frequency in Hz (default: 200.0)

## Key Parameters

- `USER_MASS`: The mass of the user in kg (configurable via command line)
- `STRIDE_TIME`: The stride time in seconds (configurable via command line)
- `FREQUENCY`: The control loop frequency in Hz (configurable via command line)
- `TRAJECTORY_LEN`: Fixed at 150 points. This is the length of the torque trajectories in the ankle.pkl and knee.pkl files.
- `GEAR_RATIO`: Set to 9 * (83/18)

## Hardware Configuration

The script configures two DephyActuators and two AS5048B encoders:

1. **Actuators**:

      - Knee actuator (port=`/dev/ttyACM0`)
      - Ankle actuator (port=`/dev/ttyACM1`)

   Both configured with:

      - Specified gear ratio
      - User-defined frequency
      - Dephy logging disabled

2. **Encoders**:

      - Knee joint encoder (bus=1, A1=True, A2=False)
      - Ankle joint encoder (bus=1, A1=False, A2=True)

## Functions

### Get Torque

```python
--8<-- "tutorials/control/torque_trajectory/torque_trajectory.py:22:42"
```

Calculates the torque setpoint for a given time point:

- `t`: Current time in seconds
- `data`: List containing torque trajectory data points
- `user_mass`: Mass of the user in kg
- `stride_time`: Stride time in seconds
- `trajectory_len`: Length of the trajectory

Returns torque setpoint scaled by user mass

### Plot Data

```python
--8<-- "tutorials/control/torque_trajectory/torque_trajectory.py:45:88"
```

Generates three plots:

1. Ankle torque (setpoint vs. actual)
2. Knee torque (setpoint vs. actual)
3. Joint positions (knee and ankle in degrees)

Saves the plot as `plot.png`

## Operation Flow

1. **Initialization**:

      - Parses command line arguments
      - Sets up logging with specified frequency
      - Configures actuators and sensors
      - Loads trajectory data from pickle files

2. **Control Sequence**:

      - Homes the OSL
      - Sets control mode to CURRENT for both actuators
      - Sets current gains
      - Waits for user input
      - Executes torque trajectory
      - Logs performance data

3. **Visualization**:

      - Generates plots after completion
      - Saves plots to "plot.png"

## Usage

1. Ensure trajectory files (`ankle.pkl` and `knee.pkl`) are in the working directory.
   We have provided a sample trajectory file in the `tutorials/control/torque_trajectory/` directory for both the ankle and knee joints.
   These trajectories are output of an high-level controller (not provided here) for level walking.
   The torque trajectories present the joint torque setpoints for one gait cycle, and the units are in Nm/kg.

2. Run the script with optional arguments:
   ```bash
   python torque_trajectory.py --mass 10 --stride-time 1 --frequency 200
   ```
3. Press Enter when prompted to start the walking trajectory
4. The system will execute the trajectory and generate plots upon completion

## Safety Notes

- This example is not meant to be used as a walking controller
- Please be cautious while changing mass parameters
- Ensure all connections are secure before operation
- Keep emergency stop accessible during operation

## Output

The script generates:

1. Real-time logs in the `./logs` directory with filename "torque_trajectory"
2. A plot file (`plot.png`) showing:

      - Torque trajectories for both joints
      - Actual vs. commanded torques
      - Joint position data in degrees

## Full Script for this tutorial
```python
--8<-- "tutorials/control/torque_trajectory/torque_trajectory.py:1:212"
```
If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
