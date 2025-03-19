from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.imu import LordMicrostrainIMU
from opensourceleg.time import SoftRealtimeLoop

FREQUENCY = 200
DT = 1 / FREQUENCY

if __name__ == "__main__":
    imu_logger = Logger(
        log_path="./logs",
        file_name="reading_imu_data",
    )
    clock = SoftRealtimeLoop(dt=DT)
    imu = LordMicrostrainIMU(
        port="/dev/ttyUSB0",
        frequency=FREQUENCY,
    )
    imu_logger.track_variable(lambda: imu.roll, "Roll")
    imu_logger.track_variable(lambda: imu.pitch, "Pitch")
    imu_logger.track_variable(lambda: imu.yaw, "Yaw")

    with imu:
        for t in clock:
            imu.update()
            imu_logger.info(f"Time: {t}; Roll: {imu.roll}; Pitch: {imu.pitch}; Yaw: {imu.yaw};")
            imu_logger.update()
