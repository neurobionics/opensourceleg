from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 500
DT = 1 / FREQUENCY
GEAR_RATIO = 9.0

if __name__ == "__main__":
    sensor_logger = Logger(
        log_path="./logs",
        file_name="reading_sensor_data",
    )
    clock = SoftRealtimeLoop(dt=DT)
    actpack_1 = DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
        debug_level=4,
        dephy_log=True,
    )
    actpack_2 = DephyActuator(
        port="/dev/ttyACM1",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
        debug_level=4,
        dephy_log=True,
    )

    sensor_logger.track_function(clock.__next__, "Time")
    sensor_logger.track_function(lambda: actpack_1.motor_current, "Motor Current")
    sensor_logger.track_function(lambda: actpack_1.winding_temperature, "Winding Temperature")
    sensor_logger.track_function(lambda: actpack_1.case_temperature, "Case Temperature")

    sensor_logger.track_function(lambda: actpack_2.motor_current, "Motor Current")
    sensor_logger.track_function(lambda: actpack_2.winding_temperature, "Winding Temperature")
    sensor_logger.track_function(lambda: actpack_2.case_temperature, "Case Temperature")

    sensor_logger.set_stream_terminator("\r")

    with actpack_1, actpack_2:
        for t in clock:
            actpack_1.update()
            actpack_2.update()

            sensor_logger.info(
                f"Time: {t:.2f};"
                f" Motor Current 01: {actpack_1.motor_current:.2f};"
                f" Motor Current 02: {actpack_2.motor_current:.2f};"
            )
            sensor_logger.update()
