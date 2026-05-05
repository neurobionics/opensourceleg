from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.imu import BHI260AP
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 200
DT = 1 / FREQUENCY

if __name__ == "__main__":
    imu_logger = Logger(
        log_path="./logs",
        file_name="reading_bhi260ap_data",
    )
    clock = SoftRealtimeLoop(dt=DT)
    imu = BHI260AP(data_rate=FREQUENCY, firmware_path="./firmware/BHI260AP.fw")
    imu.start()

    imu_logger.track_function(lambda: imu.gyro_x, "GyroX")
    imu_logger.track_function(lambda: imu.gyro_y, "GyroY")
    imu_logger.track_function(lambda: imu.gyro_z, "GyroZ")
    imu_logger.track_function(lambda: imu.acc_x, "AccelX")
    imu_logger.track_function(lambda: imu.acc_y, "AccelY")
    imu_logger.track_function(lambda: imu.acc_z, "AccelZ")

    # Enable sensors
    imu.enable_gyroscope()
    imu.enable_accelerometer()

    # Flush buffer before starting "control" loop
    imu.flush_buffer()

    for t in clock:
        imu.update()
        imu_logger.info(
            f"Time: {t:.4f}; GyroX: {imu.gyro_x:.4f}; GyroY: {imu.gyro_y:.4f}; GyroZ: {imu.gyro_z:.4f};"
            f"AccelX: {imu.acc_x:.4f}; AccelY: {imu.acc_y:.4f}; AccelZ: {imu.acc_z:.4f};"
        )
        imu_logger.update()

    # Stop IMU
    imu.stop()
