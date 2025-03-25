"""
This example is not meant to be used as a walking controller but
just to provide a reference for how a torque trajectory can be loaded and commanded.
"""

import argparse
import pickle

import numpy as np

from opensourceleg.actuators import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging import Logger
from opensourceleg.robots.osl import OpenSourceLeg
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.utilities import SoftRealtimeLoop

ANKLE_TRAJECTORY_PATH = "./ankle.pkl"
KNEE_TRAJECTORY_PATH = "./knee.pkl"


def get_torque(t: float, data: list, user_mass: float, stride_time: float, trajectory_len: int) -> int:
    """Calculate the torque setpoint for a given time point in the trajectory.

    Args:
        t (float): Current time in seconds
        data (list): List containing the torque trajectory data points
        user_mass (float): The mass of the user in kg
        stride_time (float): The stride time in seconds
        trajectory_len (int): The length of the trajectory

    Returns:
        int: Torque setpoint scaled by user mass

    The function:
    1. Calculates walking_time by taking modulo of current time with stride_time
    2. Converts walking_time to trajectory index based on trajectory_len
    3. Returns torque value at that index scaled by user_mass
    """
    walking_time = t % stride_time
    index = int(walking_time * trajectory_len)
    return user_mass * data[index]


def plot_data(plotting_data: list) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed, skipping plot")
        return

    # plot the data using matplotlib
    plt.figure(figsize=(12, 10))

    # Create three subplots for better visualization
    # Ankle Torque
    plt.subplot(3, 1, 1)
    plt.plot(plotting_data[0], plotting_data[1], label="Ankle Torque Set Point")
    plt.plot(plotting_data[0], plotting_data[3], label="Ankle Joint Torque")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title("Ankle Torque Trajectory")
    plt.legend()
    plt.grid(True)

    # Knee Torque
    plt.subplot(3, 1, 2)
    plt.plot(plotting_data[0], plotting_data[2], label="Knee Torque Set Point")
    plt.plot(plotting_data[0], plotting_data[4], label="Knee Joint Torque")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title("Knee Torque Trajectory")
    plt.legend()
    plt.grid(True)

    # Joint Positions
    plt.subplot(3, 1, 3)
    plt.plot(plotting_data[0], np.rad2deg(plotting_data[5]), label="Knee Position")
    plt.plot(plotting_data[0], np.rad2deg(plotting_data[6]), label="Ankle Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint Position (deg)")
    plt.title("Joint Positions")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig("plot.png")
    plt.show()


if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Torque trajectory control for Open Source Leg")
    parser.add_argument("--mass", type=float, default=1.0, help="User mass in kg (default: 1.0)", required=False)
    parser.add_argument(
        "--stride-time", type=float, default=1, help="Stride time in seconds (default: 1)", required=False
    )
    parser.add_argument(
        "--frequency", type=float, default=200.0, help="Control loop frequency in Hz (default: 200.0)", required=False
    )

    # Parse arguments
    args = parser.parse_args()

    # Set global parameters from arguments
    USER_MASS = args.mass
    STRIDE_TIME = args.stride_time
    FREQUENCY = args.frequency

    # Fixed parameters
    TRAJECTORY_LEN = 150
    GEAR_RATIO = 9 * (83 / 18)

    torque_logger = Logger(log_path="./logs", file_name="torque_trajectory")
    clock = SoftRealtimeLoop(dt=1 / FREQUENCY)

    actuators = {
        "knee": DephyActuator(
            tag="knee",
            port="/dev/ttyACM0",
            gear_ratio=GEAR_RATIO,
            frequency=FREQUENCY,
            dephy_log=False,
        ),
        "ankle": DephyActuator(
            tag="ankle",
            port="/dev/ttyACM1",
            gear_ratio=GEAR_RATIO,
            frequency=FREQUENCY,
            dephy_log=False,
        ),
    }

    sensors = {
        "joint_encoder_knee": AS5048B(
            tag="joint_encoder_knee",
            bus=1,
            A1_adr_pin=True,
            A2_adr_pin=False,
            zero_position=0,
            enable_diagnostics=False,
        ),
        "joint_encoder_ankle": AS5048B(
            tag="joint_encoder_ankle",
            bus=1,
            A1_adr_pin=False,
            A2_adr_pin=True,
            zero_position=0,
            enable_diagnostics=False,
        ),
    }

    osl = OpenSourceLeg(
        tag="osl",
        actuators=actuators,
        sensors=sensors,
    )

    plotting_data = [
        [],
        [],
        [],
        [],
        [],
        [],
        [],
    ]  # Time, ankle_sp, knee_sp, ankle_torque, knee_torque, knee_pos, ankle_pos

    with open(ANKLE_TRAJECTORY_PATH, "rb") as f:
        ankle_data = pickle.load(f)  # noqa: S301

    with open(KNEE_TRAJECTORY_PATH, "rb") as f:
        knee_data = pickle.load(f)  # noqa: S301

    with osl:
        osl.home()

        osl.knee.set_control_mode(CONTROL_MODES.CURRENT)
        osl.knee.set_current_gains()
        osl.knee.set_output_torque(0)

        osl.ankle.set_control_mode(CONTROL_MODES.CURRENT)
        osl.ankle.set_current_gains()
        osl.ankle.set_output_torque(0)

        input("Press Enter to start walking")

        for t in clock:
            osl.update()

            ankle_torque_sp = get_torque(t, ankle_data, USER_MASS, STRIDE_TIME, TRAJECTORY_LEN)
            knee_torque_sp = get_torque(t, knee_data, USER_MASS, STRIDE_TIME, TRAJECTORY_LEN)

            osl.ankle.set_output_torque(ankle_torque_sp)
            osl.knee.set_output_torque(knee_torque_sp)

            torque_logger.info(
                f"t: {t}, ankle_torque_sp: {ankle_torque_sp}, knee_torque_sp: {knee_torque_sp}, "
                f"ankle_output_torque: {osl.ankle.output_torque}, "
                f"knee_output_torque: {osl.knee.output_torque} "
                f"knee_position: {osl.knee.output_position}, ankle_position: {osl.ankle.output_position}"
            )
            plotting_data[0].append(t)
            plotting_data[1].append(ankle_torque_sp)
            plotting_data[2].append(knee_torque_sp)
            plotting_data[3].append(osl.ankle.output_torque)
            plotting_data[4].append(osl.knee.output_torque)
            plotting_data[5].append(osl.knee.output_position)
            plotting_data[6].append(osl.ankle.output_position)

    print("Plotting data...")
    plot_data(plotting_data)
