import math
import numpy as np
import time

class KalmanFilter2D:
    """2D Kalman filter for roll and pitch."""
    
    def __init__(self, Q_bias=1e-13, Q_angle=1e-4, R_var=3e-6):
        self.x = np.zeros(4)  # [roll, pitch, bias_roll, bias_pitch]
        self.yaw = 0.0
        self.P = np.eye(4)
        self.Q = np.diag([Q_angle, Q_angle, Q_bias, Q_bias])
        self.R = R_var * np.eye(2)
        self.prev_time = time.monotonic()

    def _predict(self, gx, gy, gz, dt):
        gx_unbiased = gx - self.x[2]
        gy_unbiased = gy - self.x[3]

        self.x[0] += gx_unbiased * dt
        self.x[1] += gy_unbiased * dt
        self.yaw += gz * dt
        self.P = self.P + self.Q

    def update(self, ax, ay, az, gx, gy, gz):
        curr_time = time.monotonic()
        dt = curr_time - self.prev_time

        self._predict(gx, gy, gz, dt)
        self.prev_time = curr_time

        roll_a = math.atan2(ay, az)
        if roll_a > 0:
            roll_a -= np.pi
        elif roll_a < 0:
            roll_a += np.pi
        pitch_a = math.atan2(-ax, math.hypot(ay, az))
        z = np.array([roll_a, pitch_a])

        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        y = z - H @ self.x
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

        return self.x[0].copy(), self.x[1].copy(), self.yaw
