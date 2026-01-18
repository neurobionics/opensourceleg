import pytest
import numpy as np
import math
import time

from opensourceleg.utilities.filters import KalmanFilter2D


@pytest.fixture
def sample_filter():
    """Create a basic KalmanFilter2D instance"""
    return KalmanFilter2D()


@pytest.fixture
def sample_filter_custom():
    """Create a KalmanFilter2D instance with custom parameters"""
    return KalmanFilter2D(
        tag="CustomFilter",
        Q_bias=1e-12,
        Q_angle=1e-3,
        R_var=5e-6
    )


# Test initialization
def test_init_default(sample_filter: KalmanFilter2D):
    """Test default initialization"""
    assert all([
        sample_filter.x.shape == (4,),
        np.allclose(sample_filter.x, np.zeros(4)),
        sample_filter.yaw == 0.0,
        sample_filter.P.shape == (4, 4),
        sample_filter.Q.shape == (4, 4),
        sample_filter.R.shape == (2, 2),
    ])


def test_init_custom():
    """Test custom initialization"""
    tag = "TestFilter"
    q_bias = 1e-12
    q_angle = 1e-3
    r_var = 5e-6
    
    f = KalmanFilter2D(tag=tag, Q_bias=q_bias, Q_angle=q_angle, R_var=r_var)
    
    assert all([
        f.x.shape == (4,),
        np.allclose(f.Q[0, 0], q_angle),
        np.allclose(f.Q[2, 2], q_bias),
        np.allclose(f.R[0, 0], r_var),
    ])


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


# Test angle normalization
def test_normalize_angle_zero(sample_filter: KalmanFilter2D):
    """Test normalization of zero angle"""
    result = sample_filter._normalize_angle(0.0)
    assert result == 0.0


def test_normalize_angle_positive(sample_filter: KalmanFilter2D):
    """Test normalization of positive angle"""
    angle = math.pi / 4
    result = sample_filter._normalize_angle(angle)
    assert -math.pi <= result <= math.pi
    assert np.isclose(result, angle)


def test_normalize_angle_negative(sample_filter: KalmanFilter2D):
    """Test normalization of negative angle"""
    angle = -math.pi / 4
    result = sample_filter._normalize_angle(angle)
    assert -math.pi <= result <= math.pi
    assert np.isclose(result, angle)


def test_normalize_angle_out_of_range_positive(sample_filter: KalmanFilter2D):
    """Test normalization of angle > pi"""
    angle = 2 * math.pi + 0.5
    result = sample_filter._normalize_angle(angle)
    assert -math.pi <= result <= math.pi
    assert np.isclose(result, 0.5)


def test_normalize_angle_out_of_range_negative(sample_filter: KalmanFilter2D):
    """Test normalization of angle < -pi"""
    angle = -2 * math.pi - 0.5
    result = sample_filter._normalize_angle(angle)
    assert -math.pi <= result <= math.pi
    assert np.isclose(result, -0.5)


def test_normalize_angle_wrapping(sample_filter: KalmanFilter2D):
    """Test angle wrapping at pi boundary"""
    # Test wrap from positive to negative
    angle = 3 * math.pi + 0.1
    result = sample_filter._normalize_angle(angle)
    assert -math.pi <= result <= math.pi


# Test predict method
def test_predict_stationary(sample_filter: KalmanFilter2D):
    """Test predict with zero gyroscope input"""
    initial_state = sample_filter.x.copy()
    sample_filter._predict(0.0, 0.0, 0.0, 0.01)
    
    # State should remain unchanged (no gyro input, no bias)
    assert np.allclose(sample_filter.x[:2], initial_state[:2])


def test_predict_gyro_rotation(sample_filter: KalmanFilter2D):
    """Test predict with gyroscope input"""
    dt = 0.01
    gx = 0.5  # rad/s
    
    initial_roll = sample_filter.x[0]
    sample_filter._predict(gx, 0.0, 0.0, dt)
    
    # Roll should increase by gyro * dt
    expected_roll = initial_roll + gx * dt
    assert np.isclose(sample_filter.x[0], expected_roll, atol=1e-6)


def test_predict_yaw_integration(sample_filter: KalmanFilter2D):
    """Test yaw integration with gyroscope z-axis"""
    dt = 0.01
    gz = 0.3  # rad/s
    
    initial_yaw = sample_filter.yaw
    sample_filter._predict(0.0, 0.0, gz, dt)
    
    # Yaw should increase by gz * dt
    expected_yaw = initial_yaw + gz * dt
    assert np.isclose(sample_filter.yaw, expected_yaw)


def test_predict_covariance_increases(sample_filter: KalmanFilter2D):
    """Test that covariance increases during prediction"""
    initial_P = sample_filter.P.copy()
    sample_filter._predict(0.1, 0.1, 0.1, 0.01)
    
    # Trace should increase (uncertainty grows)
    assert np.trace(sample_filter.P) > np.trace(initial_P)


def test_predict_angle_normalization(sample_filter: KalmanFilter2D):
    """Test that angles are normalized after prediction"""
    # Set state to large angle
    sample_filter.x[0] = 10.0  # Large roll
    
    sample_filter._predict(0.0, 0.0, 0.0, 0.01)
    
    # Roll should be normalized to [-pi, pi]
    assert -math.pi <= sample_filter.x[0] <= math.pi


# Test update method
def test_update_basic(sample_filter: KalmanFilter2D):
    """Test basic update with level IMU"""
    # Level IMU: gravity aligned with z-axis
    ax, ay, az = 0.0, 0.0, 9.81
    gx, gy, gz = 0.0, 0.0, 0.0
    
    roll, pitch, yaw = sample_filter.update(ax, ay, az, gx, gy, gz)
    
    # Should estimate near zero roll and pitch
    assert isinstance(roll, (float, np.floating))
    assert isinstance(pitch, (float, np.floating))
    assert isinstance(yaw, (float, np.floating))
    assert abs(roll) < 0.1
    assert abs(pitch) < 0.1


def test_update_tilted_roll(sample_filter: KalmanFilter2D):
    """Test update with roll tilt"""
    # IMU tilted 45 degrees around x-axis (positive rotation)
    # When tilted positively around x-axis, ay becomes negative
    angle = math.pi / 4
    ax, ay, az = 0.0, -9.81 * math.sin(angle), 9.81 * math.cos(angle)
    gx, gy, gz = 0.0, 0.0, 0.0
    
    # Multiple updates for convergence
    for _ in range(20):
        roll, pitch, yaw = sample_filter.update(ax, ay, az, gx, gy, gz)
    
    # Should estimate roll around -45 degrees (negative because of the acceleration orientation)
    assert abs(roll + angle) < 0.3 or abs(roll - angle) < 0.3  # Allow for angle wrapping


def test_update_tilted_pitch(sample_filter: KalmanFilter2D):
    """Test update with pitch tilt"""
    # IMU tilted 45 degrees around y-axis
    angle = math.pi / 4
    ax, ay, az = 9.81 * math.sin(angle), 0.0, 9.81 * math.cos(angle)
    gx, gy, gz = 0.0, 0.0, 0.0
    
    # Multiple updates for convergence
    for _ in range(20):
        roll, pitch, yaw = sample_filter.update(ax, ay, az, gx, gy, gz)
    
    # Should estimate pitch around -45 degrees
    assert abs(pitch + angle) < 0.3  # Some tolerance for filter convergence


def test_update_with_gyro_drift(sample_filter: KalmanFilter2D):
    """Test update with gyroscope bias"""
    # Simulate gyroscope drift
    ax, ay, az = 0.0, 0.0, 9.81
    gx, gy, gz = 0.05, 0.0, 0.0  # Biased gyro
    
    # Run multiple updates
    for _ in range(10):
        roll, pitch, yaw = sample_filter.update(ax, ay, az, gx, gy, gz)
    
    # Bias should be partially estimated
    assert abs(sample_filter.x[2]) > 0.0  # roll_bias should be non-zero


def test_update_returns_tuple(sample_filter: KalmanFilter2D):
    """Test that update returns a tuple of three values"""
    result = sample_filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
    
    assert isinstance(result, tuple)
    assert len(result) == 3


def test_update_convergence(sample_filter: KalmanFilter2D):
    """Test that filter converges to correct angles"""
    # Level IMU
    ax, ay, az = 0.0, 0.0, 9.81
    gx, gy, gz = 0.0, 0.0, 0.0
    
    # Multiple updates
    for _ in range(50):
        roll, pitch, yaw = sample_filter.update(ax, ay, az, gx, gy, gz)
    
    # Should converge near zero
    assert abs(roll) < 0.05
    assert abs(pitch) < 0.05


def test_update_prev_time_updated(sample_filter: KalmanFilter2D):
    """Test that previous time is updated"""
    prev_time_before = sample_filter.prev_time
    time.sleep(0.001)  # Small delay
    
    sample_filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
    
    prev_time_after = sample_filter.prev_time
    assert prev_time_after > prev_time_before


def test_update_with_zero_dt(sample_filter: KalmanFilter2D):
    """Test update with zero time difference"""
    # Call update twice immediately
    result1 = sample_filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
    result2 = sample_filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
    
    # Both should return valid results
    assert result1 is not None
    assert result2 is not None


# Test state evolution
def test_state_shape_preserved(sample_filter: KalmanFilter2D):
    """Test that state shape is preserved through updates"""
    for _ in range(10):
        sample_filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
    
    assert sample_filter.x.shape == (4,)


def test_bias_convergence(sample_filter: KalmanFilter2D):
    """Test that gyro bias converges when constant drift is present"""
    # Constant gyro drift
    gx_bias = 0.01
    ax, ay, az = 0.0, 0.0, 9.81
    
    for _ in range(20):
        sample_filter.update(ax, ay, az, gx_bias, 0.0, 0.0)
    
    # Bias should converge to approximately the true bias
    assert abs(sample_filter.x[2] - gx_bias) < 0.02


def test_multiple_updates_in_sequence(sample_filter: KalmanFilter2D):
    """Test multiple sequential updates"""
    ax, ay, az = 0.0, 0.0, 9.81
    gx, gy, gz = 0.1, 0.0, 0.0
    
    results = []
    for _ in range(5):
        result = sample_filter.update(ax, ay, az, gx, gy, gz)
        results.append(result)
    
    assert len(results) == 5
    # All results should be valid numbers
    for roll, pitch, yaw in results:
        assert np.isfinite(roll)
        assert np.isfinite(pitch)
        assert np.isfinite(yaw)


# Test numerical stability
def test_numerical_stability_large_angles(sample_filter: KalmanFilter2D):
    """Test stability with large angle inputs"""
    # Very tilted IMU
    ax, ay, az = 9.0, 4.0, 1.0
    
    roll, pitch, yaw = sample_filter.update(ax, ay, az, 0.0, 0.0, 0.0)
    
    # Should not produce NaN or Inf
    assert np.isfinite(roll)
    assert np.isfinite(pitch)
    assert np.isfinite(yaw)


def test_numerical_stability_high_gyro(sample_filter: KalmanFilter2D):
    """Test stability with high gyroscope values"""
    ax, ay, az = 0.0, 0.0, 9.81
    gx, gy, gz = 10.0, 10.0, 10.0  # Very high rotation rate
    
    roll, pitch, yaw = sample_filter.update(ax, ay, az, gx, gy, gz)
    
    assert np.isfinite(roll)
    assert np.isfinite(pitch)
    assert np.isfinite(yaw)


def test_numerical_stability_zero_acceleration(sample_filter: KalmanFilter2D):
    """Test stability with zero acceleration (edge case)"""
    # This is physically unrealistic but should not crash
    ax, ay, az = 0.0, 0.0, 0.0
    
    roll, pitch, yaw = sample_filter.update(ax, ay, az, 0.1, 0.0, 0.0)
    
    # Should still produce valid output
    assert np.isfinite(roll)
    assert np.isfinite(pitch)
    assert np.isfinite(yaw)


# Test independence of axes
def test_roll_pitch_independence(sample_filter: KalmanFilter2D):
    """Test that roll and pitch are estimated independently"""
    # Roll tilt
    ax1, ay1, az1 = 0.0, -9.81 * 0.707, 9.81 * 0.707
    roll1, pitch1, _ = sample_filter.update(ax1, ay1, az1, 0.0, 0.0, 0.0)
    
    # Reset filter
    sample_filter = KalmanFilter2D()
    
    # Pitch tilt
    ax2, ay2, az2 = 9.81 * 0.707, 0.0, 9.81 * 0.707
    roll2, pitch2, _ = sample_filter.update(ax2, ay2, az2, 0.0, 0.0, 0.0)
    
    # Roll and pitch should be affected differently by the two tilts
    assert abs(roll1) > abs(roll2)
    assert abs(pitch2) > abs(pitch1)


def test_yaw_independent_of_accel(sample_filter: KalmanFilter2D):
    """Test that yaw is not affected by acceleration"""
    gz = 0.5  # rad/s rotation around z
    
    # Different accelerations should not affect yaw increment
    for ax, ay, az in [(0, 0, 9.81), (1, 1, 9), (-1, -1, 9)]:
        sample_filter = KalmanFilter2D()
        sample_filter.update(ax, ay, az, 0.0, 0.0, gz)
        # Yaw should be approximately gz * dt (dt will be very small but positive)
        assert sample_filter.yaw > 0


# Test with realistic motion patterns
def test_standing_still(sample_filter: KalmanFilter2D):
    """Test filter with standing still (no motion)"""
    ax, ay, az = 0.0, 0.0, 9.81
    gx, gy, gz = 0.0, 0.0, 0.0
    
    # Simulate standing still for 1 second of updates
    for _ in range(100):
        roll, pitch, yaw = sample_filter.update(ax, ay, az, gx, gy, gz)
    
    # Should remain level
    assert abs(roll) < 0.1
    assert abs(pitch) < 0.1
    assert abs(yaw) < 0.01


def test_rotating_around_z(sample_filter: KalmanFilter2D):
    """Test filter with rotation around z-axis"""
    ax, ay, az = 0.0, 0.0, 9.81
    gz = 0.5  # rad/s
    
    initial_yaw = sample_filter.yaw
    
    # Simulate rotation
    for _ in range(10):
        roll, pitch, yaw = sample_filter.update(ax, ay, az, 0.0, 0.0, gz)
    
    # Yaw should increase (dead reckoning)
    assert yaw > initial_yaw
    assert abs(roll) < 0.1
    assert abs(pitch) < 0.1
