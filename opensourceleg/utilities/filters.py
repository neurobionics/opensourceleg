import math
import time

import numpy as np


class KalmanFilter2D:
    """
    2D Kalman filter to estimate global roll and pitch angles and angular rates
    from IMU gyroscope (rad/s) and acceleration (m/s^2) signals.

    Roll, pitch, and yaw are defined about the x,y,z axes, respectively.

    The state tracks 2D angles, angular rates, and gyro bias:
        x = [roll, pitch, roll_rate, pitch_rate, roll_bias, pitch_bias]

    The process model assumes linear kinematics:
        roll_{t} = roll_{t-1} + roll_rate_{t-1} * dt
        roll_rate_{t} = roll_rate_{t-1} (assumes constant velocity)
        bias_{t} = bias_{t-1} (random walk)

    Measurement model:
        z = [accel_roll, accel_pitch, gyro_x, gyro_y]

    Important note: when using the BHI260AP IMU, the acceleration filter input
    must be the "gravity" IMU output.

    Author:
        Katharine Walters

    Date:
        01/26/2026
    """

    def __init__(
        self,
        tag: str = "KalmanFilter2D",
        Q_bias: float = 1e-13,
        Q_angle: float = 1e-4,
        Q_rate: float = 1e-2,
        R_accel: float = 3e-6,
        R_gyro: float = 1e-3,
    ) -> None:
        """
        Initialize 2D Kalman filter

        Params:
            tag: identifier of KalmanFilter2D instance
            Q_bias: Variance in gyro drift rate
            Q_angle: Variance in angle prediction
            Q_rate: Variance in velocity prediction
            R_accel: Accelerometer angle measurement noise
            R_gyro: Gyroscope measurement noise
        """
        # Initialize state: [roll, pitch, roll_rate, pitch_rate, b_roll, b_pitch]
        self.x = np.zeros(6)
        self.yaw = 0.0

        # Initialize prediction covariance
        self.P = 0.1 * np.eye(6)

        # Process noise covariance (6x6)
        self.Q = np.diag([Q_angle, Q_angle, Q_rate, Q_rate, Q_bias, Q_bias])

        # Measurement noise covariance (4x4)
        # z vector order: [roll_a, pitch_a, gyro_x, gyro_y]
        self.R = np.diag([R_accel, R_accel, R_gyro, R_gyro])

        # Initialize previous time
        self.prev_time = time.monotonic()

    def _normalize_angle(self, a: float) -> float:
        """Normalize angle to [-pi, pi]."""
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _predict(self, dt: float) -> None:
        """
        Prediction step based on internal physics model.
        We assume constant angular velocity between steps.
        """
        # State-transition matrix
        F = np.eye(6)
        F[0, 2] = dt  # roll += roll_rate * dt
        F[1, 3] = dt  # pitch += pitch_rate * dt

        # Prediction
        self.x = F @ self.x
        self.x[0] = self._normalize_angle(self.x[0])
        self.x[1] = self._normalize_angle(self.x[1])

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float) -> tuple:
        """
        Update global angle estimations

        Params:
            ax, ay, az: Gravity measurement (m/s^2)
            gx, gy, gz: Gyroscope measurement (rad/s)

        Returns: (roll, pitch, roll_rate, pitch_rate, yaw)
        """
        # Calculate dt
        curr_time = time.monotonic()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time

        # Guard against invalid dt
        if dt <= 0:
            return self.x[0], self.x[1], self.x[2], self.x[3], self.yaw

        # Prediction: assumes constant angular velocity
        self._predict(dt)

        # Dead reckoning yaw (simple integration, no fusion)
        self.yaw += gz * dt

        # Accelerometer global angle calculation
        roll_a = math.atan2(ay, az)
        pitch_a = math.atan2(-ax, math.hypot(ay, az))

        # Wrap to smoothly transition across 0 from (-) to (+)
        if az < 0:
            if roll_a > 0:
                roll_a -= np.pi
            elif roll_a < 0:
                roll_a += np.pi

        z = np.array([roll_a, pitch_a, gx, gy])

        # Measurement model
        # roll_meas = roll
        # pitch_meas = pitch
        # gx = roll_rate + bias_roll
        # gy = pitch_rate + bias_pitch
        H = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 1, 0], [0, 0, 0, 1, 0, 1]])

        # Innovation step
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        y = z - H @ self.x
        y[0] = self._normalize_angle(y[0])
        y[1] = self._normalize_angle(y[1])

        # Update
        self.x = self.x + K @ y
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        # Return roll, pitch, roll_rate, pitch_rate, yaw
        return self.x[0], self.x[1], self.x[2], self.x[3], self.yaw
