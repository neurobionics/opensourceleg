from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.imu import LordMicrostrainIMU
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 200
DT = 1 / FREQUENCY

if __name__ == "__main__":
    imu_logger = Logger(
        log_path="./logs",
        file_name="reading_imu_data",
    )
    clock = SoftRealtimeLoop(dt=DT)
    imu = LordMicrostrainIMU(
        tag="LordMicrostrainIMU",
        port=r"/dev/ttyS0",
        baud_rate=921600,
        frequency=FREQUENCY,
        update_timeout=500,
        max_packets=1,
        return_packets=False,
        offline=False,
    )
    imu_logger.track_function(lambda: imu.roll, "Roll")
    imu_logger.track_function(lambda: imu.pitch, "Pitch")
    imu_logger.track_function(lambda: imu.yaw, "Yaw")

    with imu:
        for t in clock:
            imu.update()
            imu_logger.info(f"Time: {t}; Roll: {imu.roll}; Pitch: {imu.pitch}; Yaw: {imu.yaw};")
            imu_logger.update()
