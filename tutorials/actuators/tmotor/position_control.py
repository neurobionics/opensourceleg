import time

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.tmotor import TMotorServoActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.utilities import SoftRealtimeLoop

TIME_TO_STEP = 2.0
FREQUENCY = 200
DT = 1 / FREQUENCY
MOTOR_ID = 104  # Change this to match your motor's CAN ID


def position_control():
    position_logger = Logger(
        log_path="./logs",
        file_name="tmotor_position_control",
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

        # Set to position control mode (PID parameters are built-in)
        motor.set_control_mode(mode=CONTROL_MODES.POSITION)

        motor.update()
        initial_position = motor.motor_position
        command_position = initial_position

        position_logger.track_function(lambda: motor.motor_position, "Motor Position")
        position_logger.track_function(lambda: command_position, "Command Position")
        position_logger.track_function(lambda: time.time(), "Time")

        print("Starting position control...")
        for t in clock:
            if t > TIME_TO_STEP:
                # Step to a new position
                command_position = initial_position + (np.pi / 8)  # 22.5 degrees
                motor.set_motor_position(value=command_position)
            else:
                motor.set_motor_position(value=command_position)

            motor.update()

            position_logger.info(
                f"Time: {t:.3f}; "
                f"Command Position: {command_position:.3f} rad; "
                f"Motor Position: {motor.motor_position:.3f} rad; "
                f"Error: {command_position - motor.motor_position:.4f} rad"
            )
            position_logger.update()

            # Run for 5 seconds total
            if t > 5.0:
                break

        print("Position control complete")

        # Return to home position
        print("Returning to home...")
        motor.set_motor_position(value=initial_position)
        time.sleep(1.0)
        motor.update()


if __name__ == "__main__":
    position_control()
