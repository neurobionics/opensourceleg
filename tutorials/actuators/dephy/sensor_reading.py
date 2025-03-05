from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.time import SoftRealtimeLoop

FREQUENCY = 1000

if __name__ == "__main__":
    sensor_logger = Logger(
        log_path="./",
        file_name="sensor_reading.log",
    )
    clock = SoftRealtimeLoop(dt=1 / FREQUENCY)
    actpack = DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=1.0,
    )
    sensor_logger.track_variable(lambda: actpack.motor_position, "Motor Position")
    sensor_logger.track_variable(lambda: actpack.motor_current, "Motor Current")

    with actpack:
        for t in clock:
            actpack.update()
            sensor_logger.info(f"Time: {t}; Motor Position: {actpack.motor_position};")
            sensor_logger.update()
