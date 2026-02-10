from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.imu import BHI260AP, AxisTransform
from opensourceleg.utilities import SoftRealtimeLoop
from opensourceleg.utilities.filters import KalmanFilter2D

FREQUENCY = 200
DT = 1 / FREQUENCY

if __name__ == "__main__":
    imu_logger = Logger(
        log_path="./logs",
        file_name="kalman_filter",
    )
    clock = SoftRealtimeLoop(dt=DT)
    imu = BHI260AP(data_rate=FREQUENCY, firmware_path="./firmware/BHI260AP.fw")
    imu.start()

    imu_logger.track_function(lambda: roll, "Roll")
    imu_logger.track_function(lambda: pitch, "Pitch")
    imu_logger.track_function(lambda: roll_rate, "Roll Rate")
    imu_logger.track_function(lambda: pitch_rate, "Pitch Rate")
    imu_logger.track_function(lambda: yaw, "Yaw")

    # Define axis transformation (modify as needed for your IMU mounting)
    imu_transform = AxisTransform(roll="z", pitch="y", yaw="-x")

    # Kalman filter
    kalman_filter = KalmanFilter2D(
        tag="KalmanFilter2D", Q_bias=1e-13, Q_angle=1e-4, Q_rate=1e-2, R_accel=3e-6, R_gyro=1e-3
    )

    # Enable sensors
    imu.enable_gyroscope()
    imu.enable_gravity()

    # Flush buffer before starting "control" loop
    imu.flush_buffer()

    for t in clock:
        imu.update()

        acc_x, acc_y, acc_z = imu.gravity
        gyro_x, gyro_y, gyro_z = imu.gyro

        # Transform to orientation frame
        ax_t, ay_t, az_t = imu_transform.transform_accel(acc_x, acc_y, acc_z)
        gx_t, gy_t, gz_t = imu_transform.transform_gyro(gyro_x, gyro_y, gyro_z)

        # Update filter
        roll, pitch, roll_rate, pitch_rate, yaw = kalman_filter.update(ax_t, ay_t, az_t, gx_t, gy_t, gz_t)

        imu_logger.info(
            f"Time: {t:.4f}; Roll: {roll:+7.3f}, Pitch: {pitch:+7.3f}, "
            f"Roll Rate: {roll_rate:+7.3f}, Pitch Rate: {pitch_rate:+7.3f}, Yaw: {yaw:+7.3f}"
        )
        imu_logger.update()

    # Stop IMU
    imu.stop()
