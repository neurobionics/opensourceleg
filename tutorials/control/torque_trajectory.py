import pickle

import numpy as np

from opensourceleg.actuators import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging import LOGGER, Logger
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.time import SoftRealtimeLoop

TRAJECTORY_PATH = "./smoothed_action_mean_6999.pkl"
TRAJECTORY_LEN = 150

USER_MASS = 60  # kg
GEAR_RATIO = 9 * (83 / 18)

FREQUENCY = 200
STRIDE_TIME = 1  # sec


def get_torque(t: float, data: list) -> int:
    walking_time = t % STRIDE_TIME
    index = int(walking_time * TRAJECTORY_LEN)
    return USER_MASS * data[index]


def plot_data(plotting_data: list) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        LOGGER.warning("matplotlib is not installed")
        return

    # plot the data using matplotlib
    plt.figure(figsize=(12, 8))

    # Create two subplots
    plt.subplot(2, 1, 1)
    plt.plot(plotting_data[0], plotting_data[1], label="Torque Set Point")
    plt.plot(plotting_data[0], plotting_data[3], label="Actual Joint Torque")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title("Torque Trajectory")
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(plotting_data[0], plotting_data[2])
    plt.xlabel("Time (s)")
    plt.ylabel("Joint Position (deg)")
    plt.title("Joint Position")
    plt.grid(True)

    plt.tight_layout()
    plt.savefig("plot.png")
    plt.show()


if __name__ == "__main__":
    torque_logger = Logger(log_path="./logs", file_name="torque_trajectory")
    clock = SoftRealtimeLoop(dt=1 / FREQUENCY)
    actuator = DephyActuator(
        tag="ankle", port="/dev/ttyACM0", gear_ratio=GEAR_RATIO, frequency=FREQUENCY, stop_motor_on_disconnect=True
    )
    encoder = AS5048B(
        bus=1,
        A1_adr_pin=False,
        A2_adr_pin=True,
        name="encoder1",
        zero_position=0,
        enable_diagnostics=False,
    )
    plotting_data = [[], [], [], []]

    with open(TRAJECTORY_PATH, "rb") as f:
        data = pickle.load(f)  # noqa: S301

    with actuator, encoder:
        actuator.set_control_mode(CONTROL_MODES.CURRENT)
        actuator.set_current_gains()
        actuator.set_output_torque(0)

        encoder.set_zero_position()

        input("Press Enter to start walking")

        for t in clock:
            actuator.update()
            encoder.update()
            torque_sp = get_torque(t, data)
            torque_logger.info(
                f"t: {t}, torque_sp: {torque_sp}, output_torque: {actuator.output_torque}, "
                f"winding_temperature: {actuator.winding_temperature}"
            )
            plotting_data[0].append(t)
            plotting_data[1].append(torque_sp)
            plotting_data[2].append(np.rad2deg(encoder.position))
            plotting_data[3].append(actuator.output_torque)

            actuator.set_output_torque(torque_sp)

    plot_data(plotting_data)
