import time

import matplotlib.pyplot as plt
import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.tmotor import TMotorServoActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 200
DT = 1 / FREQUENCY
MOTOR_ID_1 = 104  # First motor CAN ID
MOTOR_ID_2 = 105  # Second motor CAN ID


def torque_control():
    torque_logger = Logger(
        log_path="./logs",
        file_name="tmotor_dual_torque_control",
    )

    # Initialize TMotor actuators
    motor1 = TMotorServoActuator(
        motor_type="AK80-9",
        motor_id=MOTOR_ID_1,
        offline=False,
    )

    motor2 = TMotorServoActuator(
        motor_type="AK80-9",
        motor_id=MOTOR_ID_2,
        offline=False,
    )

    clock = SoftRealtimeLoop(dt=DT)

    # Data storage for plotting
    data = {
        "time": [],
        "motor1": {
            "motor_torque": [],
            "output_torque": [],
            "command_output_torque": [],
            "motor_current": [],
            "motor_position": [],
            "motor_velocity": [],
        },
        "motor2": {
            "motor_torque": [],
            "output_torque": [],
            "command_output_torque": [],
            "motor_current": [],
            "motor_position": [],
            "motor_velocity": [],
        },
    }

    with motor1, motor2:
        print(f"Connected to Motor 1: {motor1.device_info_string()}")
        print(f"Connected to Motor 2: {motor2.device_info_string()}")

        # Home both motors
        print("Homing motors...")
        motor1.home()
        motor2.home()

        # Set to current control mode for torque control
        motor1.set_control_mode(mode=CONTROL_MODES.CURRENT)
        motor2.set_control_mode(mode=CONTROL_MODES.CURRENT)

        motor1.update()
        motor2.update()

        # Safety limits
        MAX_OUTPUT_TORQUE = 2.0  # Maximum output torque in Nm

        # Initialize command torque variables
        command_output_torque_1 = 0.0
        command_output_torque_2 = 0.0

        # Track data for motor 1
        torque_logger.track_function(lambda: motor1.motor_torque, "Motor1 Motor Torque")
        torque_logger.track_function(lambda: motor1.output_torque, "Motor1 Output Torque")
        torque_logger.track_function(lambda: command_output_torque_1, "Motor1 Command Output Torque")
        torque_logger.track_function(lambda: motor1.motor_current, "Motor1 Motor Current")
        torque_logger.track_function(lambda: motor1.motor_position, "Motor1 Motor Position")
        torque_logger.track_function(lambda: motor1.motor_velocity, "Motor1 Motor Velocity")

        # Track data for motor 2
        torque_logger.track_function(lambda: motor2.motor_torque, "Motor2 Motor Torque")
        torque_logger.track_function(lambda: motor2.output_torque, "Motor2 Output Torque")
        torque_logger.track_function(lambda: command_output_torque_2, "Motor2 Command Output Torque")
        torque_logger.track_function(lambda: motor2.motor_current, "Motor2 Motor Current")
        torque_logger.track_function(lambda: motor2.motor_position, "Motor2 Motor Position")
        torque_logger.track_function(lambda: motor2.motor_velocity, "Motor2 Motor Velocity")

        torque_logger.track_function(lambda: time.time(), "Time")

        print("Starting dual motor torque control...")

        # Control loop
        for t in clock:
            # Sinusoidal output torque command (0.2 Hz, amplitude 0.5 Nm)
            # Motor 1 and Motor 2 can have different commands if needed
            command_output_torque_1 = 0.5 * np.sin(2 * np.pi * 0.2 * t)
            command_output_torque_2 = 0.5 * np.sin(2 * np.pi * 0.2 * t)  # Same command, modify as needed

            # Limit output torque for safety
            command_output_torque_1 = np.clip(command_output_torque_1, -MAX_OUTPUT_TORQUE, MAX_OUTPUT_TORQUE)
            command_output_torque_2 = np.clip(command_output_torque_2, -MAX_OUTPUT_TORQUE, MAX_OUTPUT_TORQUE)

            # Set output torque for both motors
            motor1.set_output_torque(command_output_torque_1)
            motor2.set_output_torque(command_output_torque_2)

            motor1.update()
            motor2.update()

            # Store data for plotting
            data["time"].append(t)
            data["motor1"]["motor_torque"].append(motor1.motor_torque)
            data["motor1"]["output_torque"].append(motor1.output_torque)
            data["motor1"]["command_output_torque"].append(command_output_torque_1)
            data["motor1"]["motor_current"].append(motor1.motor_current)
            data["motor1"]["motor_position"].append(motor1.motor_position)
            data["motor1"]["motor_velocity"].append(motor1.motor_velocity)

            data["motor2"]["motor_torque"].append(motor2.motor_torque)
            data["motor2"]["output_torque"].append(motor2.output_torque)
            data["motor2"]["command_output_torque"].append(command_output_torque_2)
            data["motor2"]["motor_current"].append(motor2.motor_current)
            data["motor2"]["motor_position"].append(motor2.motor_position)
            data["motor2"]["motor_velocity"].append(motor2.motor_velocity)

            torque_logger.info(
                f"Time: {t:.3f}; "
                f"M1 Cmd: {command_output_torque_1:.3f} Nm; M1 Out: {motor1.output_torque:.3f} Nm; "
                f"M2 Cmd: {command_output_torque_2:.3f} Nm; M2 Out: {motor2.output_torque:.3f} Nm"
            )
            torque_logger.update()

            # Run for 10 seconds
            if t > 10.0:
                break

        print("Torque control complete")

        # Stop both motors
        print("Stopping motors...")
        motor1.set_output_torque(0.0)
        motor2.set_output_torque(0.0)
        motor1.update()
        motor2.update()
        time.sleep(1.0)

    # Plot the data
    plot_dual_motor_data(data)


def plot_dual_motor_data(data):
    """Plot data from both motors using subplots."""
    time_arr = np.array(data["time"])

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("Dual Motor Torque Control Comparison", fontsize=14)

    # Row 1: Torque
    # Motor 1 Torque
    axes[0, 0].plot(time_arr, data["motor1"]["command_output_torque"], "b--", label="Command", linewidth=1.5)
    axes[0, 0].plot(time_arr, data["motor1"]["output_torque"], "r-", label="Output", linewidth=1.5)
    axes[0, 0].plot(time_arr, data["motor1"]["motor_torque"], "g-", label="Motor", linewidth=1.0, alpha=0.7)
    axes[0, 0].set_ylabel("Torque (Nm)")
    axes[0, 0].set_title(f"Motor 1 (ID {MOTOR_ID_1}) - Torque")
    axes[0, 0].legend(loc="upper right")
    axes[0, 0].grid(True, alpha=0.3)

    # Motor 2 Torque
    axes[0, 1].plot(time_arr, data["motor2"]["command_output_torque"], "b--", label="Command", linewidth=1.5)
    axes[0, 1].plot(time_arr, data["motor2"]["output_torque"], "r-", label="Output", linewidth=1.5)
    axes[0, 1].plot(time_arr, data["motor2"]["motor_torque"], "g-", label="Motor", linewidth=1.0, alpha=0.7)
    axes[0, 1].set_ylabel("Torque (Nm)")
    axes[0, 1].set_title(f"Motor 2 (ID {MOTOR_ID_2}) - Torque")
    axes[0, 1].legend(loc="upper right")
    axes[0, 1].grid(True, alpha=0.3)

    # Row 2: Current
    # Motor 1 Current
    axes[1, 0].plot(time_arr, data["motor1"]["motor_current"], "m-", linewidth=1.5)
    axes[1, 0].set_ylabel("Current (A)")
    axes[1, 0].set_title(f"Motor 1 (ID {MOTOR_ID_1}) - Current")
    axes[1, 0].grid(True, alpha=0.3)

    # Motor 2 Current
    axes[1, 1].plot(time_arr, data["motor2"]["motor_current"], "m-", linewidth=1.5)
    axes[1, 1].set_ylabel("Current (A)")
    axes[1, 1].set_title(f"Motor 2 (ID {MOTOR_ID_2}) - Current")
    axes[1, 1].grid(True, alpha=0.3)

    # Row 3: Position and Velocity
    # Motor 1 Position & Velocity
    ax1_pos = axes[2, 0]
    ax1_vel = ax1_pos.twinx()
    ax1_pos.plot(time_arr, data["motor1"]["motor_position"], "c-", label="Position", linewidth=1.5)
    ax1_vel.plot(time_arr, data["motor1"]["motor_velocity"], "orange", label="Velocity", linewidth=1.5)
    ax1_pos.set_xlabel("Time (s)")
    ax1_pos.set_ylabel("Position (rad)", color="c")
    ax1_vel.set_ylabel("Velocity (rad/s)", color="orange")
    ax1_pos.set_title(f"Motor 1 (ID {MOTOR_ID_1}) - Position & Velocity")
    ax1_pos.grid(True, alpha=0.3)

    # Motor 2 Position & Velocity
    ax2_pos = axes[2, 1]
    ax2_vel = ax2_pos.twinx()
    ax2_pos.plot(time_arr, data["motor2"]["motor_position"], "c-", label="Position", linewidth=1.5)
    ax2_vel.plot(time_arr, data["motor2"]["motor_velocity"], "orange", label="Velocity", linewidth=1.5)
    ax2_pos.set_xlabel("Time (s)")
    ax2_pos.set_ylabel("Position (rad)", color="c")
    ax2_vel.set_ylabel("Velocity (rad/s)", color="orange")
    ax2_pos.set_title(f"Motor 2 (ID {MOTOR_ID_2}) - Position & Velocity")
    ax2_pos.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("./logs/dual_motor_torque_control.png", dpi=150)
    plt.show()
    print("Plot saved to ./logs/dual_motor_torque_control.png")


if __name__ == "__main__":
    torque_control()
