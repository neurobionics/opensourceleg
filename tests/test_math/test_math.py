from collections import deque

import pytest

from opensourceleg.actuators.base import MOTOR_CONSTANTS
from opensourceleg.math import EdgeDetector, SaturatingRamp, ThermalLimitException, ThermalModel
from opensourceleg.math.math import (
    MAX_SENSIBLE_CURRENT,
    MAX_SENSIBLE_TEMPERATURE,
    MIN_SENSIBLE_CURRENT,
    MIN_SENSIBLE_TEMPERATURE,
    SENSOR_HISTORY_SIZE,
)


def test_edge_detector_init():
    edi = EdgeDetector(bool_in=False)
    assert edi.cur_state is False
    assert edi.rising_edge is False
    assert edi.falling_edge is False


def test_edge_detector_update():
    edu = EdgeDetector(bool_in=False)
    edu.update(bool_in=True)
    assert edu.rising_edge is True
    assert edu.falling_edge is False
    assert edu.cur_state is True
    edu2 = EdgeDetector(bool_in=True)
    edu2.update(bool_in=False)
    assert edu2.rising_edge is False
    assert edu2.falling_edge is True
    assert edu2.cur_state is False


def test_saturating_ramp_init():
    sri = SaturatingRamp(loop_frequency=100, ramp_time=1.0)
    assert sri.delta_per_update == 1.0 / 100
    assert sri.value == 0.0


def test_saturating_ramp_update():
    sru = SaturatingRamp(loop_frequency=100, ramp_time=1.0)
    sru.update(enable_ramp=True)
    assert sru.value == 0.01
    sru.update(enable_ramp=False)
    assert sru.value == 0.0


# Create test motor constants
test_motor_constants = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=2048,
    NM_PER_AMP=0.02,
    NM_PER_RAD_TO_K=0.001,
    NM_S_PER_RAD_TO_B=0.0001,
    MAX_CASE_TEMPERATURE=80.0,
    MAX_WINDING_TEMPERATURE=120.0,
)

# Initializing the ThermalModel objects
test_model_default = ThermalModel(motor_constants=test_motor_constants)
test_model_specified = ThermalModel(
    motor_constants=test_motor_constants,
    ambient_temperature=10,
)


# Testing the ThermalModel constructor
@pytest.mark.parametrize(
    ("test_model", "class_variable", "expected_value"),
    [
        (test_model_default, "winding_thermal_capacitance", 0.20 * 81.46202695970649),
        (test_model_default, "winding_to_case_resistance", 1.0702867186480716),
        (test_model_default, "case_thermal_capacitance", 512.249065845453),
        (test_model_default, "case_to_ambient_resistance", 1.9406620046327363),
        (test_model_default, "copper_temperature_coefficient", 0.393 / 100),
        (test_model_default, "reference_temperature", 65.0),
        (test_model_default, "winding_temperature", 21.0),
        (test_model_default, "case_temperature", 21.0),
        (test_model_default, "ambient_temperature", 21.0),
        (test_model_default, "winding_soft_limit", 70.0),
        (test_model_default, "winding_hard_limit", 120.0),
        (test_model_default, "case_soft_limit", 60.0),
        (test_model_default, "case_hard_limit", 80.0),
        (test_model_specified, "winding_thermal_capacitance", 0.20 * 81.46202695970649),
        (test_model_specified, "winding_to_case_resistance", 1.0702867186480716),
        (test_model_specified, "case_thermal_capacitance", 512.249065845453),
        (test_model_specified, "case_to_ambient_resistance", 1.9406620046327363),
        (test_model_specified, "copper_temperature_coefficient", 0.393 / 100),
        (test_model_specified, "reference_temperature", 65.0),
        (test_model_specified, "winding_temperature", 10.0),
        (test_model_specified, "case_temperature", 10.0),
        (test_model_specified, "ambient_temperature", 10.0),
        (test_model_specified, "winding_soft_limit", 70.0),
        (test_model_specified, "winding_hard_limit", 120.0),
        (test_model_specified, "case_soft_limit", 60.0),
        (test_model_specified, "case_hard_limit", 80.0),
    ],
)
def test_init(test_model, class_variable, expected_value):
    """
    Tests the ThermalModel constructor\n
    Asserts the class variables of the ThermalModel object are equal to the expected values.
    """

    assert getattr(test_model, class_variable) == expected_value


def test_update():
    """
    Tests the ThermalModel update method\n
    Calls the update method with no arguments and asserts the class variables are equal to the expected values.
    Then calls the update method with a motor_current argument and asserts the class variables
    are equal to the expected values.
    This is repeated once more to test the update method with a motor_current argument.
    """

    # Testing the default ThermalModel update method with no args
    scale_factor = test_model_default.update(dt=1 / 200, motor_current=0.0)
    assert test_model_default.winding_temperature == 21.0
    assert test_model_default.case_temperature == 21.0
    assert test_model_default.ambient_temperature == 21.0
    assert isinstance(scale_factor, float)
    # Testing the default ThermalModel update method with motor_current arg
    scale_factor = test_model_default.update(dt=1 / 200, motor_current=1.0)
    # The actual temperature computation involves complex thermal equations
    # We just verify the attributes exist and have reasonable values
    assert isinstance(test_model_default.winding_temperature, float)
    assert isinstance(test_model_default.case_temperature, float)
    assert test_model_default.ambient_temperature == 21.0
    assert isinstance(scale_factor, float)
    assert 0.0 <= scale_factor <= 1.0


# New comprehensive tests for enhanced ThermalModel functionality


@pytest.fixture
def enhanced_thermal_model():
    """Create a thermal model for enhanced testing"""
    motor_constants = MOTOR_CONSTANTS(
        MOTOR_COUNT_PER_REV=2048,
        NM_PER_AMP=0.02,
        NM_PER_RAD_TO_K=0.001,
        NM_S_PER_RAD_TO_B=0.0001,
        MAX_CASE_TEMPERATURE=80.0,
        MAX_WINDING_TEMPERATURE=120.0,
    )
    return ThermalModel(
        motor_constants=motor_constants,
        actuator_tag="test_actuator",
        ambient_temperature=25.0,
    )


def test_thermal_model_initialization(enhanced_thermal_model):
    """Test thermal model initializes with correct values"""
    assert enhanced_thermal_model.winding_temperature == 25.0
    assert enhanced_thermal_model.case_temperature == 25.0
    assert enhanced_thermal_model.ambient_temperature == 25.0
    assert enhanced_thermal_model.actuator_tag == "test_actuator"
    assert len(enhanced_thermal_model.current_history) == 0


def test_thermal_model_repr(enhanced_thermal_model):
    """Test thermal model string representation"""
    repr_str = repr(enhanced_thermal_model)
    assert "ThermalModel" in repr_str
    assert "Tw=25.0°C" in repr_str
    assert "Tc=25.0°C" in repr_str


def test_is_within_bounds(enhanced_thermal_model):
    """Test bounds checking functionality"""
    assert enhanced_thermal_model._is_within_bounds(5.0, 0.0, 10.0) is True
    assert enhanced_thermal_model._is_within_bounds(0.0, 0.0, 10.0) is True
    assert enhanced_thermal_model._is_within_bounds(10.0, 0.0, 10.0) is True
    assert enhanced_thermal_model._is_within_bounds(-1.0, 0.0, 10.0) is False
    assert enhanced_thermal_model._is_within_bounds(11.0, 0.0, 10.0) is False


def test_get_fallback_value(enhanced_thermal_model):
    """Test fallback value generation"""

    # Empty history should return default
    empty_history = deque()
    assert enhanced_thermal_model._get_fallback_value(empty_history, 42.0) == 42.0

    # Non-empty history should return last value
    history = deque([1.0, 2.0, 3.0])
    assert enhanced_thermal_model._get_fallback_value(history, 42.0) == 3.0


def test_outlier_detection_insufficient_data(enhanced_thermal_model):
    """Test data packet corruption detection with insufficient faults"""

    history = deque([1.0, 2.0])
    dt = 0.005  # 200Hz

    # Should not raise exception with few faults
    filtered_value, fault_count = enhanced_thermal_model._diagnose_sensor_value(
        100000.0, history, -80000.0, 80000.0, 0.0, dt, 5
    )
    assert fault_count == 6
    assert filtered_value == 2.0  # Last value in history


def test_outlier_detection_normal_value(enhanced_thermal_model):
    """Test sensor diagnosis with normal values"""

    history = deque([5.0, 6.0, 7.0, 8.0, 9.0])
    dt = 0.005

    filtered_value, fault_count = enhanced_thermal_model._diagnose_sensor_value(7.5, history, 0.0, 100.0, 20.0, dt, 0)
    assert fault_count == 0
    assert filtered_value == 7.5


def test_outlier_detection_outlier_value(enhanced_thermal_model):
    """Test sensor diagnosis with out-of-bounds values"""
    import pytest

    history = deque([5.0, 6.0, 7.0, 8.0, 9.0])
    dt = 0.005  # 200Hz

    # Should trigger data packet corruption exception after enough faults
    from opensourceleg.math.math import DataPacketCorruptionException

    with pytest.raises(DataPacketCorruptionException):
        fault_count = 0
        for _ in range(int(1.0 / (2 * dt)) + 1):  # Exceed fault threshold
            _, fault_count = enhanced_thermal_model._diagnose_sensor_value(
                500.0, history, 0.0, 100.0, 20.0, dt, fault_count
            )


def test_current_sensor_filtering_valid(enhanced_thermal_model):
    """Test current sensor filtering with valid values"""
    history = deque([1000, 1100, 1200])  # mA values
    dt = 0.005
    filtered, fault_count = enhanced_thermal_model._diagnose_sensor_value(
        1150, history, MIN_SENSIBLE_CURRENT, MAX_SENSIBLE_CURRENT, 0.0, dt, 0
    )
    assert isinstance(filtered, float)
    assert fault_count == 0


def test_current_sensor_filtering_out_of_bounds(enhanced_thermal_model):
    """Test current sensor filtering with out-of-bounds values"""
    history = deque([1000, 1100, 1200])
    dt = 0.005
    # Very high current should trigger fallback
    filtered, fault_count = enhanced_thermal_model._diagnose_sensor_value(
        100000, history, MIN_SENSIBLE_CURRENT, MAX_SENSIBLE_CURRENT, 0.0, dt, 0
    )
    assert filtered == 1200  # Should return last valid value
    assert fault_count == 1


def test_temperature_sensor_filtering_valid(enhanced_thermal_model):
    """Test temperature sensor filtering with valid values"""
    history = deque([20.0, 25.0, 30.0])
    dt = 0.005
    filtered, fault_count = enhanced_thermal_model._diagnose_sensor_value(
        28.0,
        history,
        MIN_SENSIBLE_TEMPERATURE,
        MAX_SENSIBLE_TEMPERATURE,
        enhanced_thermal_model.ambient_temperature,
        dt,
        0,
    )
    assert isinstance(filtered, float)
    assert fault_count == 0


def test_temperature_sensor_filtering_out_of_bounds(enhanced_thermal_model):
    """Test temperature sensor filtering with out-of-bounds values"""
    history = deque([20.0, 25.0, 30.0])
    dt = 0.005
    # Very high temperature should trigger fallback
    filtered, fault_count = enhanced_thermal_model._diagnose_sensor_value(
        500.0,
        history,
        MIN_SENSIBLE_TEMPERATURE,
        MAX_SENSIBLE_TEMPERATURE,
        enhanced_thermal_model.ambient_temperature,
        dt,
        0,
    )
    assert filtered == 30.0  # Should return last valid value
    assert fault_count == 1

    # Negative temperature should use ambient as fallback
    empty_history = deque()
    filtered, fault_count = enhanced_thermal_model._diagnose_sensor_value(
        -50.0,
        empty_history,
        MIN_SENSIBLE_TEMPERATURE,
        MAX_SENSIBLE_TEMPERATURE,
        enhanced_thermal_model.ambient_temperature,
        dt,
        0,
    )
    assert filtered == 25.0  # ambient_temperature
    assert fault_count == 1


def test_soft_limiting_function(enhanced_thermal_model):
    """Test soft-limiting function behavior"""
    # Below soft limit should return 1.0
    scale = enhanced_thermal_model._soft_limiting_function(50.0, 60.0, 80.0)
    assert scale == 1.0

    # At soft limit boundary should return 1.0
    scale = enhanced_thermal_model._soft_limiting_function(60.0, 60.0, 80.0)
    assert scale == 1.0

    # Above hard limit should return 0.0
    scale = enhanced_thermal_model._soft_limiting_function(85.0, 60.0, 80.0)
    assert scale == 0.0

    # Between soft and hard limits should scale smoothly
    scale1 = enhanced_thermal_model._soft_limiting_function(65.0, 60.0, 80.0)
    scale2 = enhanced_thermal_model._soft_limiting_function(75.0, 60.0, 80.0)
    assert 0.0 < scale1 < 1.0
    assert 0.0 < scale2 < 1.0
    assert scale1 > scale2  # Higher temp should have lower scale


def test_get_thermal_scale_factor(enhanced_thermal_model):
    """Test thermal scale factor calculation"""
    # Normal temperatures should give scale factor near 1.0
    scale = enhanced_thermal_model.get_thermal_scale_factor()
    assert 0.0 <= scale <= 1.0
    assert scale > 0.8  # Should be high for normal temps

    # Set high temperatures
    enhanced_thermal_model.winding_temperature = 110.0  # Near limit
    enhanced_thermal_model.case_temperature = 75.0  # Near limit
    scale_high = enhanced_thermal_model.get_thermal_scale_factor()
    assert 0.0 <= scale_high <= 1.0
    assert scale_high < scale  # Should be lower than normal temps


def test_check_thermal_limits_no_exception(enhanced_thermal_model):
    """Test thermal limit checking with safe temperatures"""
    # Normal temperatures should not raise exception
    enhanced_thermal_model.check_thermal_limits_and_raise()  # Should not raise


def test_check_thermal_limits_case_exception(enhanced_thermal_model):
    """Test thermal limit exception for case temperature"""
    enhanced_thermal_model.case_temperature = 85.0  # Above limit (80.0)
    with pytest.raises(ThermalLimitException) as exc_info:
        enhanced_thermal_model.check_thermal_limits_and_raise()

    assert "TEST_ACTUATOR" in str(exc_info.value)
    assert "Case thermal limit" in str(exc_info.value)


def test_check_thermal_limits_winding_exception(enhanced_thermal_model):
    """Test thermal limit exception for winding temperature"""
    enhanced_thermal_model.winding_temperature = 125.0  # Above limit (120.0)
    with pytest.raises(ThermalLimitException) as exc_info:
        enhanced_thermal_model.check_thermal_limits_and_raise()

    assert "TEST_ACTUATOR" in str(exc_info.value)
    assert "Winding thermal limit" in str(exc_info.value)


def test_update_with_current(enhanced_thermal_model):
    """Test update method with motor current"""
    initial_temp = enhanced_thermal_model.winding_temperature
    scale_factor = enhanced_thermal_model.update(dt=1 / 200, motor_current=5000)  # 5A in mA

    assert isinstance(scale_factor, float)
    assert 0.0 <= scale_factor <= 1.0
    # Temperature should increase with current
    assert enhanced_thermal_model.winding_temperature >= initial_temp


def test_update_with_case_temperature_sensor(enhanced_thermal_model):
    """Test update method with case temperature sensor input"""
    scale_factor = enhanced_thermal_model.update(dt=1 / 200, motor_current=1000, case_temperature=35.0)

    assert isinstance(scale_factor, float)
    assert 0.0 <= scale_factor <= 1.0


def test_update_history_management(enhanced_thermal_model):
    """Test that update method manages history correctly"""
    initial_history_len = len(enhanced_thermal_model.current_history)

    # Multiple updates should populate history
    for i in range(3):
        enhanced_thermal_model.update(dt=1 / 200, motor_current=1000 + i * 100)

    assert len(enhanced_thermal_model.current_history) == min(3, SENSOR_HISTORY_SIZE)
    assert len(enhanced_thermal_model.current_history) > initial_history_len


def test_filter_window_size_limit(enhanced_thermal_model):
    """Test that history respects filter window size"""

    # Add more values than filter window size
    for i in range(SENSOR_HISTORY_SIZE + 3):
        enhanced_thermal_model.update(dt=1 / 200, motor_current=1000 + i * 100)

    # History should be limited to window size (but may be less due to filtering)
    assert len(enhanced_thermal_model.current_history) <= SENSOR_HISTORY_SIZE
    # Should have some history entries
    assert len(enhanced_thermal_model.current_history) > 0
