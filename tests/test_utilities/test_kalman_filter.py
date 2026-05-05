import math
import time

import numpy as np
import pytest

# Assuming the class is saved in this location, adjust as needed
from opensourceleg.utilities.filters import KalmanFilter2D


@pytest.fixture
def sample_filter():
    """Create a basic KalmanFilter2D instance"""
    return KalmanFilter2D()


@pytest.fixture
def sample_filter_custom():
    """Create a KalmanFilter2D instance with custom parameters"""
    return KalmanFilter2D(tag="CustomFilter", Q_bias=1e-12, Q_angle=1e-3, Q_rate=1e-2, R_accel=0.1, R_gyro=0.01)


# --- Initialization Tests ---


def test_init_default(sample_filter: KalmanFilter2D):
    """Test default initialization for 6-state filter"""
    assert all([
        sample_filter.x.shape == (6,),
        np.allclose(sample_filter.x, np.zeros(6)),
        sample_filter.yaw == 0.0,
        sample_filter.P.shape == (6, 6),
        sample_filter.Q.shape == (6, 6),
        sample_filter.R.shape == (4, 4),  # 2 accel + 2 gyro measurements
    ])


def test_init_custom():
    """Test custom initialization"""
    tag = "TestFilter"
    q_bias = 1e-12
    q_angle = 1e-3
    q_rate = 1e-2
    r_accel = 0.5
    r_gyro = 0.01

    f = KalmanFilter2D(tag=tag, Q_bias=q_bias, Q_angle=q_angle, Q_rate=q_rate, R_accel=r_accel, R_gyro=r_gyro)

    # Check Q diagonal: [angle, angle, rate, rate, bias, bias]
    assert np.allclose(f.Q[0, 0], q_angle)
    assert np.allclose(f.Q[2, 2], q_rate)
    assert np.allclose(f.Q[4, 4], q_bias)

    # Check R diagonal: [accel, accel, gyro, gyro]
    assert np.allclose(f.R[0, 0], r_accel)
    assert np.allclose(f.R[2, 2], r_gyro)


def test_init_covariance_matrices(sample_filter: KalmanFilter2D):
    """Test that covariance matrices are symmetric"""
    assert np.allclose(sample_filter.P, sample_filter.P.T)
    assert np.allclose(sample_filter.Q, sample_filter.Q.T)
    assert np.allclose(sample_filter.R, sample_filter.R.T)


def test_init_covariance_positive_definite(sample_filter: KalmanFilter2D):
    """Test that covariance matrices are positive definite"""
    # Check eigenvalues are positive
    p_eigenvalues = np.linalg.eigvals(sample_filter.P)
    q_eigenvalues = np.linalg.eigvals(sample_filter.Q)
    r_eigenvalues = np.linalg.eigvals(sample_filter.R)

    assert np.all(p_eigenvalues > 0)
    assert np.all(q_eigenvalues > 0)
    assert np.all(r_eigenvalues > 0)


# --- Angle Normalization Tests (Unchanged) ---
# These logic tests remain valid regardless of state size
def test_normalize_angle_zero(sample_filter: KalmanFilter2D):
    result = sample_filter._normalize_angle(0.0)
    assert result == 0.0


def test_normalize_angle_positive(sample_filter: KalmanFilter2D):
    angle = math.pi / 4
    result = sample_filter._normalize_angle(angle)
    assert np.isclose(result, angle)


def test_normalize_angle_wrapping(sample_filter: KalmanFilter2D):
    angle = 3 * math.pi + 0.1
    result = sample_filter._normalize_angle(angle)
    assert -math.pi <= result <= math.pi


# --- Predict Method Tests ---


def test_predict_stationary(sample_filter: KalmanFilter2D):
    """Test predict with zero state rates"""
    initial_state = sample_filter.x.copy()
    # In new filter, predict only takes dt.
    # It relies on internal state x[2] (roll_rate) and x[3] (pitch_rate).
    sample_filter._predict(0.01)

    # State should remain unchanged as initialized rates are 0
    assert np.allclose(sample_filter.x[:2], initial_state[:2])


def test_predict_gyro_rotation(sample_filter: KalmanFilter2D):
    """Test predict integrates internal rate state"""
    dt = 0.01
    fake_rate = 0.5  # rad/s

    # Manually set the state rate (simulate a converged filter moving)
    sample_filter.x[2] = fake_rate  # Roll Rate index
    initial_roll = sample_filter.x[0]

    sample_filter._predict(dt)

    # Roll should increase by rate * dt
    expected_roll = initial_roll + fake_rate * dt
    assert np.isclose(sample_filter.x[0], expected_roll, atol=1e-6)


def test_predict_covariance_increases(sample_filter: KalmanFilter2D):
    """Test that covariance increases during prediction"""
    initial_P = sample_filter.P.copy()
    sample_filter._predict(0.01)

    # Trace should increase (uncertainty grows due to Q)
    assert np.trace(sample_filter.P) > np.trace(initial_P)


# --- Update Method Tests ---


def test_update_basic(sample_filter: KalmanFilter2D):
    """Test basic update with level IMU"""
    ax, ay, az = 0.0, 0.0, 9.81
    gx, gy, gz = 0.0, 0.0, 0.0

    # New returns: roll, pitch, roll_rate, pitch_rate, yaw
    roll, pitch, r_rate, p_rate, yaw = sample_filter.update(ax, ay, az, gx, gy, gz)

    assert abs(roll) < 0.1
    assert abs(pitch) < 0.1
    assert abs(r_rate) < 0.1


def test_update_returns_tuple_5(sample_filter: KalmanFilter2D):
    """Test that update returns a tuple of FIVE values"""
    result = sample_filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
    assert isinstance(result, tuple)
    assert len(result) == 5


def test_update_tilted_roll(sample_filter: KalmanFilter2D):
    """Test update with roll tilt"""
    angle = math.pi / 4
    # Tilted around X: ay becomes negative components of gravity
    ax, ay, az = 0.0, -9.81 * math.sin(angle), 9.81 * math.cos(angle)
    gx, gy, gz = 0.0, 0.0, 0.0

    for _ in range(50):  # Needs more steps or higher P to converge quickly
        roll, pitch, _, _, _ = sample_filter.update(ax, ay, az, gx, gy, gz)

    # Allow for angle wrapping and convergence
    assert abs(roll + angle) < 0.3 or abs(roll - angle) < 0.3


def test_update_with_gyro_drift(sample_filter: KalmanFilter2D):
    """Test that bias states capture constant gyro error"""
    # Stationary Accel
    ax, ay, az = 0.0, 0.0, 9.81
    # Gyro reports rotation, but accel says we are still
    gx, gy, gz = 0.05, 0.0, 0.0

    for _ in range(50):
        sample_filter.update(ax, ay, az, gx, gy, gz)

    # Bias indices are now 4 (roll_bias) and 5 (pitch_bias)
    # The filter should learn that the 0.05 is just bias
    assert abs(sample_filter.x[4]) > 0.01


def test_rate_estimation():
    """
    Test that the filter estimates angular rate correctly.

    NOTE: We effectively disable the accelerometer (High R_accel) to prevent
    gravity conflicts. We also lock the Bias variance to zero to prevent
    the filter from splitting the signal between 'Rate' and 'Bias'.
    """
    f = KalmanFilter2D(
        R_accel=1000.0,
        R_gyro=0.01,
        Q_bias=0.0,  # Tell filter bias never changes
    )

    # Manually lock the Bias Uncertainty (P matrix) to zero
    # Indices 4 and 5 are roll_bias and pitch_bias
    f.P[4, 4] = 0.0
    f.P[5, 5] = 0.0

    ax, ay, az = 0.0, 0.0, 9.81
    gx_input = 0.5  # rad/s

    # Run loop
    for _ in range(50):
        f.update(ax, ay, az, gx_input, 0.0, 0.0)
        time.sleep(0.001)

    # Now it should be exactly 0.5 (or very close)
    assert np.isclose(f.x[2], gx_input, atol=0.05)


def test_update_prev_time_updated(sample_filter: KalmanFilter2D):
    """Test that previous time is updated"""
    prev_time_before = sample_filter.prev_time
    time.sleep(0.001)
    sample_filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
    assert sample_filter.prev_time > prev_time_before


def test_update_with_zero_dt(sample_filter: KalmanFilter2D):
    """Test update with zero time difference"""
    # Force prev_time to be now
    sample_filter.prev_time = time.monotonic()
    result = sample_filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
    assert result is not None


# --- Stability Tests ---


def test_numerical_stability_high_gyro(sample_filter: KalmanFilter2D):
    """Test stability with high gyroscope values"""
    ax, ay, az = 0.0, 0.0, 9.81
    gx, gy, gz = 10.0, 10.0, 10.0

    results = sample_filter.update(ax, ay, az, gx, gy, gz)
    assert all(np.isfinite(val) for val in results)


def test_yaw_independent_of_accel(sample_filter: KalmanFilter2D):
    """Test that yaw is simple integration"""
    gz = 0.5
    # Update once to set prev_time
    sample_filter.update(0, 0, 9.81, 0, 0, 0)
    time.sleep(0.01)

    # Update with rotation
    _, _, _, _, yaw = sample_filter.update(0, 0, 9.81, 0.0, 0.0, gz)
    assert yaw != 0.0


def test_standing_still(sample_filter: KalmanFilter2D):
    """Test filter with standing still (no motion)"""
    ax, ay, az = 0.0, 0.0, 9.81
    gx, gy, gz = 0.0, 0.0, 0.0

    for _ in range(50):
        roll, pitch, rr, pr, yaw = sample_filter.update(ax, ay, az, gx, gy, gz)

    assert abs(roll) < 0.1
    assert abs(pitch) < 0.1
    assert abs(rr) < 0.1  # Rate should be near 0
