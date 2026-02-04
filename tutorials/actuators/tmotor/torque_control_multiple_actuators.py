import time

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
        gear_ratio=9.0,
        offline=False,
    )

    motor2 = TMotorServoActuator(
        motor_type="AK80-9",
        motor_id=MOTOR_ID_2,
        gear_ratio=9.0,
        offline=False,
    )

    clock = SoftRealtimeLoop(dt=DT)

    with motor1, motor2:
        print(f"Connected to Motor 1: {motor1.__str__()}")
        print(f"Connected to Motor 2: {motor2.__str__()}")

        motor1.update()
        motor2.update()

        # Home both motors
        print("Setting origin...")
        motor1.set_origin()
        motor2.set_origin()

        # Set to current control mode for torque control
        motor1.set_control_mode(mode=CONTROL_MODES.CURRENT)
        motor2.set_control_mode(mode=CONTROL_MODES.CURRENT)

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
        torque_logger.track_function(lambda: motor1.output_position, "Motor1 Motor Position")
        torque_logger.track_function(lambda: motor1.output_velocity, "Motor1 Motor Velocity")

        # Track data for motor 2
        torque_logger.track_function(lambda: motor2.motor_torque, "Motor2 Motor Torque")
        torque_logger.track_function(lambda: motor2.output_torque, "Motor2 Output Torque")
        torque_logger.track_function(lambda: command_output_torque_2, "Motor2 Command Output Torque")
        torque_logger.track_function(lambda: motor2.motor_current, "Motor2 Motor Current")
        torque_logger.track_function(lambda: motor2.output_position, "Motor2 Motor Position")
        torque_logger.track_function(lambda: motor2.output_velocity, "Motor2 Motor Velocity")

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


if __name__ == "__main__":
    torque_control()
