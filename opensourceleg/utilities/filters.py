import math
import numpy as np
import time

def normalize_angle(a):
    """Normalize angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi

class KalmanFilter2D:
    """2D Kalman filter for roll and pitch."""
    
    def __init__(self, Q_bias=1e-13, Q_angle=1e-4, R_var=3e-6):
        """
        Initialize 2D Kalman filter to estimate global roll and pitch 
        from IMU gyroscope (rad/s) and gravity (m/s^2) signals
        
        Params:
            Q_bias: variance in gyroscope drift rate
            Q_angle: uncertainty in gyroscope angle prediction
            R_var: accelerometer angle measurement noise
        """
        # Initialize state
        self.x = np.zeros(4)  # [roll, pitch, bias_roll, bias_pitch]
        self.yaw = 0.0

        # Initialize prediction covariance
        self.P = 0.1*np.eye(4)

        # Process noise covariance 
        self.Q = np.diag([Q_angle, Q_angle, Q_bias, Q_bias])

        # Measurement noise covaraince (accel noise)
        self.R = R_var* np.eye(2)

        # Initialize previous time
        self.prev_time = time.monotonic()

    def _predict(self, gx, gy, gz, dt):
        """
        Prediction step
        
        Params:
            gx: x-axis gyroscope measurement (rad/s)
            gy: y-axis gyroscope measurement (rad/s)
            gz: z-axis gyroscope measurement (rad/s)
            dt: discrete time step (seconds)
        """
        # State-transition matrix
        F = np.array([
            [1,0,-dt,0], 
            [0,1,0,-dt], 
            [0,0,1,0], 
            [0,0,0,1]
        ])
        
        # Control input matrix
        B = np.array([
            [dt, 0], 
            [0, dt], 
            [0,0], 
            [0,0]
        ])
        u = np.array([gx, gy])

        # Prediction x_new = x + (gyro - bias)*dt
        # x = F*x + B*u
        self.x = F@self.x + B@u
        self.x[0] = normalize_angle(self.x[0])
        self.x[1] = normalize_angle(self.x[1])

        # Covariance prediction
        self.P = F@self.P@F.T + self.Q*dt

    def update(self, ax, ay, az, gx, gy, gz):
        """
        Update global angle estimations
        
        Params:
            ax: x-axis gravity measurement (m/s^2)
            ay: y-axis gravity measurement (m/s^2)
            az: z-axis gravity measurement (m/s^2)
            gx: x-axis gyroscope measurement (rad/s)
            gy: y-axis gyroscope measurement (rad/s)
            gz: z-axis gyroscope measurement (rad/s)
        """
        # Calculate change in time since last update
        curr_time = time.monotonic()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time

        # Predict
        self._predict(gx, gy, gz, dt)
        
        # Accelerometer global angle calculation
        roll_a = math.atan2(ay, az)
        pitch_a = math.atan2(-ax, math.hypot(ay, az))

        # Wrap to smoothly transition across 0 from (-) to (+)
        if roll_a > 0:
            roll_a -= np.pi
        elif roll_a < 0:
            roll_a += np.pi
        
        z = np.array([roll_a, pitch_a])

        # Measurement model
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Innovation step
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        y = z - H @ self.x
        y[0] = normalize_angle(y[0])
        y[1] = normalize_angle(y[1])

        # Update
        self.x = self.x + K @ y
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T # Joseph form for numeric stability

        return self.x[0].copy(), self.x[1].copy(), self.yaw
