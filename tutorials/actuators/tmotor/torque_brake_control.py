import time

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.tmotor import TMotorServoActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 200
DT = 1 / FREQUENCY
MOTOR_ID = 104  # Change this to match your motor's CAN ID


def torque_control():
    torque_logger = Logger(
        log_path="./logs",
        file_name="tmotor_torque_control",
    )

    # Initialize TMotor actuator
    motor = TMotorServoActuator(
        motor_type="AK80-9",  # Change to your motor model
        motor_id=MOTOR_ID,
        gear_ratio=9.0,
        offline=False,
    )

    clock = SoftRealtimeLoop(dt=DT)

    with motor:
        motor.update()

        # Set the encoder origin first (optional)
        print("Setting encoder origin...")
        motor.set_origin()

        # Set to current control mode for torque control
        motor.set_control_mode(mode=CONTROL_MODES.CURRENT_BRAKE)

        # Motor torque constant for AK80-9
        MAX_OUTPUT_TORQUE = 10.0  # Maximum output torque in Nm (safety limit)

        # Track torque data
        torque_logger.track_function(lambda: motor.motor_torque, "Motor Torque")
        torque_logger.track_function(lambda: motor.output_torque, "Output Torque")
        torque_logger.track_function(lambda: command_torque_brake, "Command Output Torque")
        torque_logger.track_function(lambda: motor.motor_current, "Motor Current")
        torque_logger.track_function(lambda: motor.output_position, "Motor Position")
        torque_logger.track_function(lambda: motor.output_velocity, "Motor Velocity")
        torque_logger.track_function(lambda: time.monotonic(), "Time")

        print("Starting torque control...")

        # Create a torque profile
        for t in clock:
            # Current brake command
            command_torque_brake = 5.0

            # Limit output torque for safety
            command_torque_brake = np.clip(command_torque_brake, 0, MAX_OUTPUT_TORQUE)

            # Set output torque
            motor.set_output_brake_torque(command_torque_brake)
            motor.update()

            torque_logger.info(
                f"Time: {t:.3f}; "
                f"Command Output Torque: {command_torque_brake:.3f} Nm; "
                f"Output Torque: {motor.output_torque:.3f} Nm; "
                f"Motor Torque: {motor.motor_torque:.3f} Nm; "
                f"Current: {motor.motor_current:.2f} mA; "
                f"Velocity: {motor.output_velocity:.2f} rad/s"
            )
            torque_logger.update()

            # Run for 10 seconds
            if t > 10.0:
                break

        print("Torque control complete")

        # Stop the motor
        print("Stopping motor...")
        motor.set_output_brake_torque(0.0)
        motor.update()
        time.sleep(1.0)


if __name__ == "__main__":
    torque_control()
