import numpy as np
import pytest

from opensourceleg.actuators.base import *

DEFAULT_VALUES = [0, 1, 1000, -1000]


def create_motor_constants(default_value):
    return MOTOR_CONSTANTS(
        MOTOR_COUNT_PER_REV=default_value,
        NM_PER_AMP=default_value,
        NM_PER_RAD_TO_K=default_value,
        NM_S_PER_RAD_TO_B=default_value,
        MAX_CASE_TEMPERATURE=default_value,
        MAX_WINDING_TEMPERATURE=default_value,
    )


@pytest.mark.parametrize("default_value", DEFAULT_VALUES)
def test_motor_constants_init(default_value):
    motor_constants = create_motor_constants(default_value)
    assert motor_constants.MOTOR_COUNT_PER_REV == default_value
    assert motor_constants.NM_PER_AMP == default_value
    assert motor_constants.NM_PER_RAD_TO_K == default_value
    assert motor_constants.NM_S_PER_RAD_TO_B == default_value
    assert motor_constants.MAX_CASE_TEMPERATURE == default_value
    assert motor_constants.MAX_WINDING_TEMPERATURE == default_value


@pytest.mark.parametrize("default_value", DEFAULT_VALUES)
def test_motor_constants_init_invalid(default_value):
    with pytest.raises(TypeError):
        MOTOR_CONSTANTS(
            MOTOR_COUNT_PER_REV=default_value,
            NM_PER_RAD_TO_K=default_value,
            NM_S_PER_RAD_TO_B=default_value,
            MAX_CASE_TEMPERATURE=default_value,
            MAX_WINDING_TEMPERATURE=default_value,
        )

    with pytest.raises(TypeError):
        MOTOR_CONSTANTS(
            MAX_CASE_TEMPERATURE=default_value,
            MAX_WINDING_TEMPERATURE=default_value,
        )

    with pytest.raises(TypeError):
        MOTOR_CONSTANTS(
            MOTOR_COUNT=default_value,
            SOMETHING=default_value,
        )


@pytest.mark.parametrize("default_value", DEFAULT_VALUES)
def test_motor_constants_properties(default_value):
    motor_constants = create_motor_constants(default_value)

    if default_value == 0:
        assert motor_constants.RAD_PER_COUNT == 0
    else:
        assert motor_constants.RAD_PER_COUNT == 2 * np.pi / default_value

    assert motor_constants.NM_PER_MILLIAMP == default_value / 1000


def test_motor_constants_len():
    motor_constants = create_motor_constants(0)
    assert len(motor_constants) == 6


def test_control_modes_value():
    assert CONTROL_MODES.POSITION.value == 0
    assert CONTROL_MODES.CURRENT.value == 1
    assert CONTROL_MODES.VOLTAGE.value == 2
    assert CONTROL_MODES.IMPEDANCE.value == 3
    assert CONTROL_MODES.VELOCITY.value == 4
    assert CONTROL_MODES.TORQUE.value == 5
    assert CONTROL_MODES.IDLE.value == 6


def test_control_modes_name():
    assert CONTROL_MODES(0).name == "POSITION"
    assert CONTROL_MODES(1).name == "CURRENT"
    assert CONTROL_MODES(2).name == "VOLTAGE"
    assert CONTROL_MODES(3).name == "IMPEDANCE"
    assert CONTROL_MODES(4).name == "VELOCITY"
    assert CONTROL_MODES(5).name == "TORQUE"
    assert CONTROL_MODES(6).name == "IDLE"


def test_control_modes_len():
    assert len(CONTROL_MODES) == 7


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
        control_gains = ControlGains(kt=default_value)

    with pytest.raises(TypeError):
        control_gains = ControlGains(
            default_value,
            default_value,
            default_value,
            default_value,
            default_value,
            default_value,
        )


def test_control_gains_len():
    control_gains = ControlGains()
    assert len(control_gains.__dict__) == 6


def test_control_mode_config_init_default():
    control_mode_config = ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=lambda _: None,
    )
    assert control_mode_config.has_gains == False
    assert control_mode_config.max_gains == None


def test_control_mode_config_init_invalid():
    with pytest.raises(TypeError):
        control_mode_config = ControlModeConfig()

    with pytest.raises(TypeError):
        control_mode_config = ControlModeConfig(
            control_mode=CONTROL_MODES.POSITION,
            gains=ControlGains(),
        )

    with pytest.raises(TypeError):
        control_mode_config = ControlModeConfig(
            has_gains=True,
        )

    with pytest.raises(TypeError):
        control_mode_config = ControlModeConfig(
            lambda x: x,
            lambda x: x**2,
        )


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

    assert control_mode_configs.POSITION == None
    assert control_mode_configs.CURRENT == None
    assert control_mode_configs.VOLTAGE == None
    assert control_mode_configs.IMPEDANCE == None
    assert control_mode_configs.VELOCITY == None
    assert control_mode_configs.TORQUE == None
    assert control_mode_configs.IDLE == None


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

    assert control_mode_configs.POSITION == default_control_mode_config
    assert control_mode_configs.CURRENT == default_control_mode_config
    assert control_mode_configs.VOLTAGE == default_control_mode_config
    assert control_mode_configs.IMPEDANCE == default_control_mode_config
    assert control_mode_configs.VELOCITY == None
    assert control_mode_configs.TORQUE == None
    assert control_mode_configs.IDLE == None


def test_control_mode_methods():
    assert type(CONTROL_MODE_METHODS) == list
    assert all([type(x) == str for x in CONTROL_MODE_METHODS])
    assert len(CONTROL_MODE_METHODS) >= 12


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
        _required_modes = {CONTROL_MODES.POSITION, CONTROL_MODES.CURRENT}

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
