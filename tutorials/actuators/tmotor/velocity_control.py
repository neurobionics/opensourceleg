import time

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.tmotor import TMotorServoActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 200
DT = 1 / FREQUENCY
MOTOR_ID = 104  # Change this to match your motor's CAN ID


def velocity_control():
    velocity_logger = Logger(
        log_path="./logs",
        file_name="tmotor_velocity_control",
    )

    # Initialize TMotor actuator
    motor = TMotorServoActuator(
        motor_type="AK80-9",  # Change to your motor model
        motor_id=MOTOR_ID,
        offline=False,
    )

    clock = SoftRealtimeLoop(dt=DT)

    with motor:
        print(f"Connected to TMotor: {motor.device_info_string()}")

        # Home the motor first
        print("Homing motor...")
        motor.home()

        # Set to velocity control mode (PID parameters are built-in)
        motor.set_control_mode(mode=CONTROL_MODES.VELOCITY)

        motor.update()

        # Initialize command velocity variable before tracking (needed for lambda)
        command_velocity = 0.0

        # Track velocity data
        velocity_logger.track_function(lambda: motor.motor_velocity, "Motor Velocity")
        velocity_logger.track_function(lambda: command_velocity, "Command Velocity")
        velocity_logger.track_function(lambda: motor.motor_position, "Motor Position")
        velocity_logger.track_function(lambda: time.time(), "Time")

        print("Starting velocity control...")

        # Create a sinusoidal velocity profile
        for t in clock:
            # Sinusoidal velocity command (0.5 Hz, amplitude 0.5 rad/s)
            command_velocity = 0.5 * np.sin(2 * np.pi * 0.5 * t)

            motor.set_motor_velocity(value=command_velocity)
            motor.update()

            velocity_logger.info(
                f"Time: {t:.3f}; "
                f"Command Velocity: {command_velocity:.3f} rad/s; "
                f"Motor Velocity: {motor.motor_velocity:.3f} rad/s; "
                f"Position: {motor.motor_position:.3f} rad"
            )
            velocity_logger.update()

            # Run for 10 seconds to see multiple cycles
            if t > 10.0:
                break

        print("Velocity control complete")

        # Stop the motor
        print("Stopping motor...")
        motor.set_motor_velocity(value=0.0)
        motor.update()
        time.sleep(1.0)


if __name__ == "__main__":
    velocity_control()
