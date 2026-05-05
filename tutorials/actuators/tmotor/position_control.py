import time

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.tmotor import TMotorServoActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.utilities import SoftRealtimeLoop

STEP_AMPLITUDE = 0.3  # Radians
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
        gear_ratio=9.0,
        offline=False,
    )

    clock = SoftRealtimeLoop(dt=DT)

    with motor:
        motor.update()

        print("Setting motor origin...")
        motor.set_origin()
        print(f"Current motor position {motor.output_position:.3f}")

        # Set to position control mode (PID parameters are built-in and
        # can only be modified via R-link)
        motor.set_control_mode(mode=CONTROL_MODES.POSITION)

        initial_position = motor.output_position
        command_position = initial_position

        position_logger.track_function(lambda: motor.output_position, "Motor Position")
        position_logger.track_function(lambda: command_position, "Command Position")
        position_logger.track_function(lambda: time.monotonic(), "Time")

        print(f"Starting staircase step response (Step: {STEP_AMPLITUDE})...")

        for t in clock:
            # Calculate how many steps have passed
            step_count = int(t)

            # Continuously increment position based on step count
            command_position = initial_position + (step_count * STEP_AMPLITUDE)

            # Send command
            motor.set_output_position(value=command_position)
            motor.update()

            # Log data
            position_logger.info(
                f"Time: {t:.3f}s | "
                f"Step: {step_count} | "
                f"Cmd: {command_position:.3f} | "
                f"Meas: {motor.output_position:.3f}"
            )
            position_logger.update()

            # Run for 5 seconds total
            if t > 5.0:
                break

        print("Position control complete")

        # Return to home position
        print("Returning to home...")
        motor.set_output_position(value=initial_position)
        time.sleep(1.0)
        motor.update()


if __name__ == "__main__":
    position_control()
