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
    imu = BHI260AP(
        data_rate = FREQUENCY, 
        firmware_path = "/home/kwalte/firmware/BHI260AP.fw"
        )    
    imu.start()

    imu_logger.track_function(lambda: imu.gyro[0], "GyroX")
    imu_logger.track_function(lambda: imu.gyro[1], "GyroY")
    imu_logger.track_function(lambda: imu.gyro[2], "GyroZ")
    imu_logger.track_function(lambda: imu.gravity[0], "GravityX")
    imu_logger.track_function(lambda: imu.gravity[1], "GravityY")
    imu_logger.track_function(lambda: imu.gravity[2], "GravityZ")

     # Enable sensors
    imu.enable_gyroscope()
    imu.enable_gravity()

    # Flush buffer before starting "control" loop
    imu.flush_buffer()

    for t in clock:
        imu.update()
        imu_logger.info(f"Time: {t}; GyroX: {imu.gyro[0]}; GyroY: {imu.gyro[1]}; GyroZ: {imu.gyro[2]};")
        imu_logger.update()

    # Stop IMU
    imu.stop()

