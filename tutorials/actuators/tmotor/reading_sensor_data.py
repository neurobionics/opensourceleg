from opensourceleg.actuators.tmotor import TMotorServoActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 200
DT = 1 / FREQUENCY
MOTOR_ID = 104  # Change this to match your motor's CAN ID

if __name__ == "__main__":
    sensor_logger = Logger(
        log_path="./logs",
        file_name="tmotor_sensor_data",
    )
    clock = SoftRealtimeLoop(dt=DT)

    # Initialize TMotor actuator
    motor = TMotorServoActuator(
        motor_type="AK80-9",  # Change to your motor model
        motor_id=MOTOR_ID,
        offline=False,
    )

    # Track sensor data
    sensor_logger.track_function(lambda: motor.motor_position, "Motor Position")
    sensor_logger.track_function(lambda: motor.motor_velocity, "Motor Velocity")
    sensor_logger.track_function(lambda: motor.motor_current, "Motor Current")
    sensor_logger.track_function(lambda: motor.motor_torque, "Motor Torque")
    sensor_logger.track_function(lambda: motor.case_temperature, "Case Temperature")
    sensor_logger.track_function(lambda: motor.winding_temperature, "Winding Temperature")

    with motor:
        print(f"Connected to TMotor: {motor.device_info_string()}")
        print("Reading sensor data...")

        for t in clock:
            motor.update()

            sensor_logger.info(
                f"Time: {t:.3f}; "
                f"Position: {motor.motor_position:.3f} rad; "
                f"Velocity: {motor.motor_velocity:.2f} rad/s; "
                f"Current: {motor.motor_current:.2f} A; "
                f"Torque: {motor.motor_torque:.3f} Nm"
            )
            sensor_logger.update()

            # Stop after 5 seconds
            if t > 5.0:
                break

        print("Sensor data logging complete")
