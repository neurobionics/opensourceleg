import math
import numpy as np
import time

class KalmanFilter2D:
    """
    2D Kalman filter to estimate global roll and pitch angles
    from IMU gyroscope (rad/s) and acceleration (m/s^2) signals.

    Roll, pitch, and yaw are defined about the x,y,z axes, respectively.

    The state tracks 2D angles and gyro bias:
        x = [roll, pitch, roll_bias, pitch_bias]

    The process model assumes linear kinematics: 
        x_{t} = x_{t-1} + (gyro_{t} - bias{t-1})*dt

    Important note: When using the BHI260AP IMU, the acceleration 
    filter input must be the "gravity" IMU output.  
    
    This implementation was tested with the BHI260AP IMU against 
    the LordMicrostrain GX5 IMU, with maximum error <0.02 rad. 

    Author:
        Katharine Walters

    Date:
        01/13/2026
    """
    
    def __init__(self, Q_bias=1e-13, Q_angle=1e-4, R_var=3e-6) -> None:
        """
        Initialize 2D Kalman filter
        
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

    def _normalize_angle(self, a: float) -> float:
        """Normalize angle to [-pi, pi]."""
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _predict(self, gx: float, gy: float, gz: float, dt: float) -> None:
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
        self.x[0] = self._normalize_angle(self.x[0])
        self.x[1] = self._normalize_angle(self.x[1])
        self.yaw += gz*dt  # Dead reckoning yaw (simple integration, no fusion)

        # Covariance prediction
        self.P = F@self.P@F.T + self.Q*dt

    def update(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float) -> tuple:
        """
        Update global angle estimations
        
        Params:
            ax: x-axis gravity measurement (m/s^2)
            ay: y-axis gravity measurement (m/s^2)
            az: z-axis gravity measurement (m/s^2)
            gx: x-axis gyroscope measurement (rad/s)
            gy: y-axis gyroscope measurement (rad/s)
            gz: z-axis gyroscope measurement (rad/s)

        Returns: (roll, pitch, yaw)
        """
        # Calculate change in time since last update
        curr_time = time.monotonic()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time

        # Guard against invalid dt
        if dt <= 0: return self.x[0].copy(), self.x[1].copy(), self.yaw

        # Predict
        self._predict(gx, gy, gz, dt)
        
        # Accelerometer global angle calculation
        roll_a = math.atan2(ay, az)
        pitch_a = math.atan2(-ax, math.hypot(ay, az))

        # Wrap to smoothly transition across 0 from (-) to (+)
        if az < 0:
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
        y[0] = self._normalize_angle(y[0])
        y[1] = self._normalize_angle(y[1])

        # Update
        self.x = self.x + K @ y
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T # Joseph form for numeric stability

        return self.x[0].copy(), self.x[1].copy(), self.yaw
