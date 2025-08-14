from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.rust import Logger
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 1000
DT = 1 / FREQUENCY
GEAR_RATIO = 1.0

if __name__ == "__main__":
    Logger.update_log_file_configuration(
        log_directory="./logs",
        log_name="reading_sensor_data.log",
    )
    clock = SoftRealtimeLoop(dt=DT)
    actpack = DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
        debug_level=0,
        dephy_log=False,
    )
    Logger.track_functions({
        "Motor_Position": lambda: actpack.motor_position,
        "Motor_Current": lambda: actpack.motor_current,
    })

    with actpack:
        for t in clock:
            actpack.update()
            Logger.info(f"Time: {t}; Motor Position: {actpack.motor_position};")
            Logger.record()
