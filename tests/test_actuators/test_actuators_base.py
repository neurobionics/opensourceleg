from typing import ClassVar
from unittest.mock import Mock, patch

import numpy as np
import pytest

from opensourceleg.actuators.base import (
    CONTROL_MODE_CONFIGS,
    CONTROL_MODE_METHODS,
    CONTROL_MODES,
    HARDWARE_REQUIRED_METHODS,
    HARDWARE_REQUIRED_PROPERTIES,
    MOTOR_CONSTANTS,
    ActuatorBase,
    ControlGains,
    ControlModeConfig,
    MethodWithRequiredModes,
    T,
    requires,
)
from opensourceleg.logging.exceptions import ControlModeException

DEFAULT_VALUES = [0, 1, 1000, -1000]


@pytest.fixture
def non_zero_positive_values(request):
    max_len = request.param[0] if isinstance(request.param, tuple) else request.param
    values = np.random.randint(1, 1000, max_len)

    # Apply thermal limit constraints to avoid validation errors
    if max_len > 5:
        # MAX_CASE_TEMPERATURE is at index 4, MAX_WINDING_TEMPERATURE is at index 5
        values[5] = values[4] + np.random.randint(1, 100)  # winding temp > case temp

        # WINDING_SOFT_LIMIT is at index 13, must be < MAX_WINDING_TEMPERATURE
        if max_len > 13:
            # Ensure soft limit is positive and less than hard limit
            values[13] = max(1, values[5] - np.random.randint(1, min(50, values[5])))

        # CASE_SOFT_LIMIT is at index 14, must be < MAX_CASE_TEMPERATURE
        if max_len > 14:
            # Ensure soft limit is positive and less than hard limit
            values[14] = max(1, values[4] - np.random.randint(1, min(50, values[4])))

    return values


@pytest.fixture
def zero_values(request):
    max_len = request.param
    return np.zeros(max_len)


@pytest.fixture
def non_zero_negative_values(request):
    max_len = request.param
    return np.random.randint(-1000, -1, max_len)


def create_motor_constants(values):
    return MOTOR_CONSTANTS(*values)


@pytest.mark.parametrize(
    "non_zero_positive_values",
    [(len(MOTOR_CONSTANTS.__annotations__),)],
    indirect=True,
)
def test_motor_constants_init(non_zero_positive_values):
    motor_constants = create_motor_constants(non_zero_positive_values)
    assert list(motor_constants.__dict__.values()) == list(non_zero_positive_values)
    assert 2 * np.pi / non_zero_positive_values[0] == motor_constants.RAD_PER_COUNT
    assert non_zero_positive_values[1] / 1000 == motor_constants.NM_PER_MILLIAMP


@pytest.mark.parametrize(
    "zero_values, non_zero_negative_values",
    [
        (
            len(MOTOR_CONSTANTS.__annotations__),
            len(MOTOR_CONSTANTS.__annotations__),
        )
    ],
    indirect=True,
)
def test_motor_constants_init_values(zero_values, non_zero_negative_values):
    with pytest.raises(ValueError):
        create_motor_constants(zero_values)

    with pytest.raises(ValueError):
        create_motor_constants(non_zero_negative_values)


def test_motor_constants_init_types():
    with pytest.raises(TypeError):
        MOTOR_CONSTANTS(1, 2)


@pytest.mark.parametrize(
    "non_zero_positive_values",
    [(len(MOTOR_CONSTANTS.__annotations__))],
    indirect=True,
)
def test_motor_constants_properties(non_zero_positive_values):
    motor_constants = create_motor_constants(non_zero_positive_values)
    assert 2 * np.pi / non_zero_positive_values[0] == motor_constants.RAD_PER_COUNT
    assert non_zero_positive_values[1] / 1000 == motor_constants.NM_PER_MILLIAMP


def test_control_modes_default_four():
    assert {"POSITION", "CURRENT", "VOLTAGE", "IMPEDANCE"} <= {e.name for e in CONTROL_MODES}


def test_control_modes_dephy_order():
    assert CONTROL_MODES.POSITION.value == 0
    assert CONTROL_MODES.VOLTAGE.value == 1
    assert CONTROL_MODES.CURRENT.value == 2
    assert CONTROL_MODES.IMPEDANCE.value == 3


def test_control_modes_len():
    assert len(CONTROL_MODES) >= 4


@pytest.mark.parametrize("default_value", DEFAULT_VALUES)
def test_control_gains_init(default_value):
    control_gains = ControlGains(
        kp=default_value,
        ki=default_value,
        kd=default_value,
        k=default_value,
        b=default_value,
        ff=default_value,
    )
    assert control_gains.kp == default_value
    assert control_gains.ki == default_value
    assert control_gains.kd == default_value
    assert control_gains.k == default_value
    assert control_gains.b == default_value
    assert control_gains.ff == default_value


def test_control_gains_init_default():
    control_gains = ControlGains()
    assert control_gains.kp == 0
    assert control_gains.ki == 0
    assert control_gains.kd == 0
    assert control_gains.k == 0
    assert control_gains.b == 0
    assert control_gains.ff == 0


@pytest.mark.parametrize("default_value", DEFAULT_VALUES)
def test_control_gains_init_partial(default_value):
    control_gains = ControlGains(
        kp=default_value,
        ki=default_value,
    )

    assert control_gains.kp == default_value
    assert control_gains.ki == default_value
    assert control_gains.kd == 0
    assert control_gains.k == 0
    assert control_gains.b == 0
    assert control_gains.ff == 0


@pytest.mark.parametrize("default_value", DEFAULT_VALUES)
def test_control_gains_init_invalid(default_value):
    with pytest.raises(TypeError):
        ControlGains(kt=default_value)

    # Python3.9 does not support @dataclass(kw_only=True)
    # with pytest.raises(TypeError):
    #     control_gains = ControlGains(
    #         default_value,
    #         default_value,
    #         default_value,
    #         default_value,
    #         default_value,
    #         default_value,
    #     )


def test_control_gains_len():
    control_gains = ControlGains()
    assert len(control_gains.__dict__) == 6


def test_control_mode_config_init_default():
    control_mode_config = ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=lambda _: None,
    )
    assert control_mode_config.has_gains is False
    assert control_mode_config.max_gains is None


def test_control_mode_config_init_invalid():
    with pytest.raises(TypeError):
        ControlModeConfig()

    with pytest.raises(TypeError):
        ControlModeConfig(
            control_mode=CONTROL_MODES.POSITION,
            gains=ControlGains(),
        )

    with pytest.raises(TypeError):
        ControlModeConfig(
            has_gains=True,
        )

    # Python3.9 does not support @dataclass(kw_only=True)
    # with pytest.raises(TypeError):
    #     control_mode_config = ControlModeConfig(
    #         lambda x: x,
    #         lambda x: x**2,
    #     )


def test_control_mode_config_init():
    entry_callback = lambda x: x
    exit_callback = lambda x: x**2
    control_mode_config = ControlModeConfig(
        entry_callback=entry_callback,
        exit_callback=exit_callback,
    )

    assert control_mode_config.entry_callback == entry_callback
    assert control_mode_config.exit_callback == exit_callback
    assert control_mode_config.entry_callback(2) == 2
    assert control_mode_config.exit_callback(2) == 4


def test_control_mode_configs_init_default():
    control_mode_configs = CONTROL_MODE_CONFIGS()

    assert control_mode_configs.POSITION is None
    assert control_mode_configs.CURRENT is None
    assert control_mode_configs.VOLTAGE is None
    assert control_mode_configs.IMPEDANCE is None
    assert control_mode_configs.VELOCITY is None
    assert control_mode_configs.TORQUE is None
    assert control_mode_configs.IDLE is None


def test_control_mode_configs_init():
    entry_callback = lambda x: x
    exit_callback = lambda x: x**2

    default_control_mode_config = ControlModeConfig(
        entry_callback=entry_callback,
        exit_callback=exit_callback,
    )

    control_mode_configs = CONTROL_MODE_CONFIGS(
        POSITION=default_control_mode_config,
        CURRENT=default_control_mode_config,
        VOLTAGE=default_control_mode_config,
        IMPEDANCE=default_control_mode_config,
    )

    assert default_control_mode_config == control_mode_configs.POSITION
    assert default_control_mode_config == control_mode_configs.CURRENT
    assert default_control_mode_config == control_mode_configs.VOLTAGE
    assert default_control_mode_config == control_mode_configs.IMPEDANCE
    assert control_mode_configs.VELOCITY is None
    assert control_mode_configs.TORQUE is None
    assert control_mode_configs.IDLE is None


def test_control_mode_methods():
    assert type(CONTROL_MODE_METHODS) is list
    assert all(type(x) is str for x in CONTROL_MODE_METHODS)
    assert len(CONTROL_MODE_METHODS) >= 11


def test_typevar_usage():
    def example_func(x: int) -> int:
        return x

    typed_func: T = example_func
    assert callable(typed_func)
    assert typed_func(2) == 2


def test_typevar_usage_invalid():
    example_func = 2

    typed_func: T = example_func
    assert not callable(typed_func)


def test_method_with_required_modes():
    class TestClass(MethodWithRequiredModes):
        _required_modes: ClassVar = {CONTROL_MODES.POSITION, CONTROL_MODES.CURRENT}

    test_class_instance = TestClass()

    assert hasattr(test_class_instance, "_required_modes")
    assert isinstance(test_class_instance._required_modes, set)
    assert CONTROL_MODES.POSITION in test_class_instance._required_modes
    assert CONTROL_MODES.CURRENT in test_class_instance._required_modes


def test_method_with_required_modes_invalid():
    class TestClass:
        pass

    test_class_instance = TestClass()
    assert not isinstance(test_class_instance, MethodWithRequiredModes)

    class TestClassWrongType(MethodWithRequiredModes):
        _required_modes = CONTROL_MODES.POSITION

    test_class_instance = TestClassWrongType()
    assert not isinstance(test_class_instance._required_modes, set)


def test_requries_decorator():
    @requires(CONTROL_MODES.POSITION)
    def test_func():
        return True

    assert isinstance(test_func._required_modes, set)
    assert test_func._required_modes == {CONTROL_MODES.POSITION}

    @requires(CONTROL_MODES.POSITION, CONTROL_MODES.CURRENT)
    def test_func_multi():
        return True

    assert test_func_multi._required_modes == {
        CONTROL_MODES.POSITION,
        CONTROL_MODES.CURRENT,
    }

    @requires(CONTROL_MODES.POSITION)
    @requires(CONTROL_MODES.CURRENT)
    def test_func_multi_decorators():
        return True

    assert test_func_multi_decorators._required_modes == {
        CONTROL_MODES.POSITION,
        CONTROL_MODES.CURRENT,
    }


def test_requires_decorator_invalid():
    with pytest.raises(TypeError):

        @requires(2)
        def test_func():
            return True

    with pytest.raises(TypeError):

        @requires(CONTROL_MODES.POSITION, 2)
        def test_func():
            return True

    with pytest.raises(TypeError):

        @requires(CONTROL_MODES.POSITION)
        @requires(2)
        def test_func():
            return True

    with pytest.raises(TypeError):

        @requires(CONTROL_MODES.POSITION)
        @requires(CONTROL_MODES.CURRENT, 2)
        def test_func():
            return True


MOCK_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    POSITION=ControlModeConfig(
        entry_callback=lambda _: "Position entry callback",
        exit_callback=lambda _: "Position exit callback",
    ),
    CURRENT=ControlModeConfig(
        entry_callback=lambda _: "Current entry callback",
        exit_callback=lambda _: "Current exit callback",
    ),
    VOLTAGE=ControlModeConfig(
        entry_callback=lambda _: "Voltage entry callback",
        exit_callback=lambda _: "Voltage exit callback",
    ),
    IMPEDANCE=ControlModeConfig(
        entry_callback=lambda _: "Impedance entry callback",
        exit_callback=lambda _: "Impedance exit callback",
    ),
    VELOCITY=ControlModeConfig(
        entry_callback=lambda _: "Velocity entry callback",
        exit_callback=lambda _: "Velocity exit callback",
    ),
    TORQUE=ControlModeConfig(
        entry_callback=lambda _: "Torque entry callback",
        exit_callback=lambda _: "Torque exit callback",
    ),
    IDLE=ControlModeConfig(
        entry_callback=lambda _: "Idle entry callback",
        exit_callback=lambda _: "Idle exit callback",
    ),
)


class MockActuator(ActuatorBase):
    @property
    def _CONTROL_MODE_CONFIGS(self):
        return MOCK_CONTROL_MODE_CONFIGS

    def start(self):
        pass

    def stop(self):
        pass

    def update(self):
        pass

    def set_motor_voltage(self, value):
        pass

    def set_motor_current(self, value):
        pass

    def set_motor_position(self, value):
        pass

    def set_motor_torque(self, value):
        pass

    def set_output_torque(self, value):
        pass

    def set_current(self, value):
        pass

    def set_voltage(self, value):
        pass

    def set_motor_impedance(self, value):
        pass

    def set_output_impedance(self, value):
        pass

    def set_impedance_cc_pidf_gains(self, kp, ki, kd, ff):
        pass

    def set_current_gains(self, kp, ki, kd, ff):
        pass

    def set_position_gains(self, kp, ki, kd, ff):
        pass

    def _set_impedance_gains(self, k, p):
        pass

    def home(self):
        pass

    @property
    def motor_position(self):
        return 0.0

    @property
    def motor_velocity(self):
        return 0.0

    @property
    def motor_voltage(self):
        return 0.0

    @property
    def motor_current(self):
        return 0.0

    @property
    def motor_torque(self):
        return 0.0

    @property
    def case_temperature(self):
        return 0.0

    @property
    def winding_temperature(self):
        return 0.0


@pytest.fixture
def mock_actuator():
    return MockActuator(
        "test_actuator",
        10.0,
        MOTOR_CONSTANTS(
            MOTOR_COUNT_PER_REV=1000,
            NM_PER_AMP=0.1,
            NM_PER_RAD_TO_K=1.0,
            NM_S_PER_RAD_TO_B=0.1,
            MAX_CASE_TEMPERATURE=100.0,
            MAX_WINDING_TEMPERATURE=150.0,
        ),
    )


def test_actuator_initialization(mock_actuator: MockActuator):
    assert mock_actuator.tag == "test_actuator"
    assert mock_actuator.gear_ratio == 10.0
    assert mock_actuator.mode == CONTROL_MODES.IDLE
    assert not mock_actuator.is_homed
    assert not mock_actuator.is_offline
    assert mock_actuator.frequency == 1000


def test_set_control_mode(mock_actuator: MockActuator):
    mock_actuator.set_control_mode(CONTROL_MODES.VOLTAGE)
    assert mock_actuator.mode == CONTROL_MODES.VOLTAGE


def test_set_and_get_positions(mock_actuator: MockActuator):
    mock_actuator.set_motor_zero_position(1.0)
    assert mock_actuator.motor_zero_position == 1.0


def test_output_position_and_velocity(mock_actuator: MockActuator):
    with patch.object(MockActuator, "motor_position", new_callable=Mock(return_value=10.0)):
        assert mock_actuator.output_position == 1.0  # 10.0 / 10.0 (gear_ratio)

    with patch.object(MockActuator, "motor_velocity", new_callable=Mock(return_value=5.0)):
        assert mock_actuator.output_velocity == 0.5  # 5.0 / 10.0 (gear_ratio)


def test_temperature_limits(mock_actuator: MockActuator):
    assert mock_actuator.max_case_temperature == 100.0
    assert mock_actuator.max_winding_temperature == 150.0


def test_method_restriction(mock_actuator: MockActuator):
    mock_actuator.set_control_mode(CONTROL_MODES.IDLE)

    with pytest.raises(ControlModeException):
        mock_actuator.set_motor_voltage(5.0)

    mock_actuator.set_control_mode(CONTROL_MODES.VOLTAGE)
    with patch.object(mock_actuator, "set_motor_voltage") as mock_method:
        mock_actuator.set_motor_voltage(5.0)
        mock_method.assert_called_once_with(5.0)


# def test_context_manager():
#     with (
#         patch("opensourceleg.actuators.base.ActuatorBase.start") as mock_start,
#         patch("opensourceleg.actuators.base.ActuatorBase.stop") as mock_stop,
#     ):
#         with MockActuator("test", 1.0, MOTOR_CONSTANTS(1, 1, 1, 1, 1, 1)):
#             pass

#         mock_start.assert_called_once()
#         mock_stop.assert_called_once()


def test_set_output_position(mock_actuator: MockActuator):
    mock_actuator.set_control_mode(CONTROL_MODES.POSITION)
    with patch.object(mock_actuator, "set_motor_position") as mock_set_motor_position:
        mock_actuator.set_output_position(1.0)
        mock_set_motor_position.assert_called_once_with(value=10.0)  # 1.0 * gear_ratio


def test_motor_constants(mock_actuator: MockActuator):
    assert mock_actuator.MOTOR_CONSTANTS.MOTOR_COUNT_PER_REV == 1000
    assert mock_actuator.MOTOR_CONSTANTS.NM_PER_AMP == 0.1
    assert mock_actuator.MOTOR_CONSTANTS.NM_PER_RAD_TO_K == 1.0
    assert mock_actuator.MOTOR_CONSTANTS.NM_S_PER_RAD_TO_B == 0.1
    assert mock_actuator.MOTOR_CONSTANTS.MAX_CASE_TEMPERATURE == 100.0
    assert mock_actuator.MOTOR_CONSTANTS.MAX_WINDING_TEMPERATURE == 150.0


@pytest.fixture
def mock_actuator_offline():
    return MockActuator(
        "test_actuator_offline",
        10.0,
        MOTOR_CONSTANTS(
            MOTOR_COUNT_PER_REV=1000,
            NM_PER_AMP=0.1,
            NM_PER_RAD_TO_K=1.0,
            NM_S_PER_RAD_TO_B=0.1,
            MAX_CASE_TEMPERATURE=100.0,
            MAX_WINDING_TEMPERATURE=150.0,
        ),
        offline=True,
    )


def test_hardware_required_constants():
    """Test that hardware requirement constants are defined."""
    assert isinstance(HARDWARE_REQUIRED_METHODS, list)
    assert isinstance(HARDWARE_REQUIRED_PROPERTIES, list)
    assert len(HARDWARE_REQUIRED_METHODS) > 0
    assert len(HARDWARE_REQUIRED_PROPERTIES) > 0

    # Check that essential methods are included
    essential_methods = ["start", "stop", "update", "home"]
    for method in essential_methods:
        assert method in HARDWARE_REQUIRED_METHODS

    # Check that essential properties are included
    essential_properties = ["motor_position", "motor_velocity", "motor_current"]
    for prop in essential_properties:
        assert prop in HARDWARE_REQUIRED_PROPERTIES


def test_offline_mode_initialization(mock_actuator_offline: MockActuator):
    """Test that offline mode is properly initialized."""
    assert mock_actuator_offline.is_offline is True
    assert mock_actuator_offline.is_open is True
    assert mock_actuator_offline.is_streaming is True
    assert mock_actuator_offline.tag == "test_actuator_offline"


def test_offline_mode_method_calls(mock_actuator_offline: MockActuator):
    """Test that hardware-required methods work in offline mode without errors."""
    # These should not raise exceptions in offline mode
    mock_actuator_offline.start()
    mock_actuator_offline.stop()
    mock_actuator_offline.update()
    mock_actuator_offline.home()
    mock_actuator_offline.set_motor_voltage(12.0)
    mock_actuator_offline.set_motor_current(1.5)
    mock_actuator_offline.set_motor_position(0.5)
    mock_actuator_offline.set_motor_torque(10.0)
    mock_actuator_offline.set_output_torque(5.0)
    mock_actuator_offline.set_current_gains(1.0, 0.1, 0.01, 0.0)
    mock_actuator_offline.set_position_gains(1.0, 0.1, 0.01, 0.0)


def test_offline_mode_property_access(mock_actuator_offline: MockActuator):
    """Test that hardware-required properties return default values in offline mode."""
    # These should return default values in offline mode
    assert mock_actuator_offline.motor_position == 0.0
    assert mock_actuator_offline.motor_velocity == 0.0
    assert mock_actuator_offline.motor_voltage == 0.0
    assert mock_actuator_offline.motor_current == 0.0
    assert mock_actuator_offline.motor_torque == 0.0
    assert mock_actuator_offline.case_temperature == 25.0  # Room temperature default
    assert mock_actuator_offline.winding_temperature == 25.0  # Room temperature default


def test_offline_mode_control_modes(mock_actuator_offline: MockActuator):
    """Test that control mode switching works in offline mode."""
    # Control mode switching should work normally
    mock_actuator_offline.set_control_mode(CONTROL_MODES.VOLTAGE)
    assert mock_actuator_offline.mode == CONTROL_MODES.VOLTAGE

    mock_actuator_offline.set_control_mode(CONTROL_MODES.CURRENT)
    assert mock_actuator_offline.mode == CONTROL_MODES.CURRENT

    mock_actuator_offline.set_control_mode(CONTROL_MODES.POSITION)
    assert mock_actuator_offline.mode == CONTROL_MODES.POSITION


def test_offline_vs_online_mode():
    """Test differences between offline and online mode initialization."""
    online_actuator = MockActuator(
        "online",
        10.0,
        MOTOR_CONSTANTS(1000, 0.1, 1.0, 0.1, 100.0, 150.0),
        offline=False,
    )

    offline_actuator = MockActuator(
        "offline",
        10.0,
        MOTOR_CONSTANTS(1000, 0.1, 1.0, 0.1, 100.0, 150.0),
        offline=True,
    )

    # Online actuator should not be open/streaming by default
    assert online_actuator.is_offline is False
    assert online_actuator.is_open is False
    assert online_actuator.is_streaming is False

    # Offline actuator should be open/streaming by default
    assert offline_actuator.is_offline is True
    assert offline_actuator.is_open is True
    assert offline_actuator.is_streaming is True


def test_offline_mode_with_context_manager(mock_actuator_offline: MockActuator):
    """Test that offline mode works properly with context manager."""
    # This should not raise any exceptions
    with mock_actuator_offline as actuator:
        assert actuator.is_offline is True
        actuator.set_motor_voltage(12.0)
        actuator.update()
        pos = actuator.motor_position  # Should return 0.0
        assert pos == 0.0
