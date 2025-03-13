from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.timing import SoftRealtimeLoop

FREQUENCY = 1000
DT = 1 / FREQUENCY
GEAR_RATIO = 1.0

if __name__ == "__main__":
    sensor_logger = Logger(
        log_path="./",
        file_name="sensor_reading.log",
    )
    clock = SoftRealtimeLoop(dt=DT)
    actpack = DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
        debug_level=0,
        dephy_log=False,
    )
    sensor_logger.track_variable(lambda: actpack.motor_position, "Motor Position")
    sensor_logger.track_variable(lambda: actpack.motor_current, "Motor Current")

    with actpack:
        for t in clock:
            actpack.update()
            sensor_logger.info(f"Time: {t}; Motor Position: {actpack.motor_position};")
            sensor_logger.update()
