from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from opensourceleg.actuators.base import (
    CONTROL_MODES,
    MOTOR_CONSTANTS,
    UnsupportedControlModeError,
)
from opensourceleg.actuators.brushed import (
    MaxonActuator,
    _maxon_position_mode_exit,
    degrees_to_radians,
    radians_to_degrees,
)

# Valid constants (winding > case) so construction passes MOTOR_CONSTANTS validation.
VALID_MOTOR_CONSTANTS = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=1024,
    NM_PER_AMP=0.00652,
    MAX_CASE_TEMPERATURE=100.0,
    MAX_WINDING_TEMPERATURE=125.0,
)


# actuator fixture
@pytest.fixture
def mock_gpio():
    """
    Patch the gpiozero devices so MaxonActuator can be built online without hardware.
    OutputDevice is called for inb first, then ina (per __init__ order), so side_effect
    hands back distinct mocks and the two direction pins stay distinguishable.
    """
    with (
        patch("opensourceleg.actuators.brushed.LGPIOFactory"),
        patch("opensourceleg.actuators.brushed.PWMOutputDevice") as mock_pwm,
        patch("opensourceleg.actuators.brushed.OutputDevice") as mock_out,
    ):
        speed_control = MagicMock(name="speed_control")
        inb = MagicMock(name="inb")
        ina = MagicMock(name="ina")
        mock_pwm.return_value = speed_control
        mock_out.side_effect = [inb, ina]
        yield {"speed_control": speed_control, "ina": ina, "inb": inb}


@pytest.fixture
def maxon(mock_gpio):
    """A default online MaxonActuator backed by mocked gpiozero devices."""
    return MaxonActuator(motor_constants=VALID_MOTOR_CONSTANTS)


# Module-level unit conversion helpers
def test_degrees_to_radians():
    assert degrees_to_radians(180.0) == pytest.approx(np.pi)
    assert degrees_to_radians(0.0) == 0.0


def test_radians_to_degrees():
    assert radians_to_degrees(np.pi) == pytest.approx(180.0)
    assert radians_to_degrees(0.0) == 0.0


def test_degrees_radians_roundtrip():
    assert radians_to_degrees(degrees_to_radians(45.0)) == pytest.approx(45.0)


# Construction / configuration
def test_default_constants_no_raise(mock_gpio):
    actuator = MaxonActuator(motor_constants=VALID_MOTOR_CONSTANTS)
    assert actuator is not None


def test_maxon_init(maxon: MaxonActuator):
    assert maxon.tag == "maxon_actuator"
    assert maxon.gear_ratio == 6.6
    assert maxon.frequency == 6000
    assert maxon.enable_pin == 12
    assert maxon.ina_pin == 24
    assert maxon.inb_pin == 25
    assert maxon.pwm_maximum_command == 0.3
    assert maxon.pwm_minimum_command == 0.07
    assert maxon.pwm_lower_limit == 0.02


def test_maxon_init_uses_default_motor_constants(mock_gpio):
    """When motor_constants is omitted, MaxonActuator builds its own defaults.

    The base class may store the constants under a private name, so we verify
    the defaults via their observable effect: motor_position uses
    MOTOR_COUNT_PER_REV=1024, so 1024 encoder counts must equal exactly 2π rad.
    """
    encoder = MagicMock()
    encoder.count = 1024
    actuator = MaxonActuator()  # no motor_constants kwarg
    actuator.set_motor_encoder(encoder)
    actuator.position_control_config()
    assert actuator.motor_position == pytest.approx(2 * np.pi)


def test_maxon_init_custom_motor_constants(mock_gpio):
    """Explicitly supplied motor constants are stored as-is."""
    custom = MOTOR_CONSTANTS(
        MOTOR_COUNT_PER_REV=512,
        NM_PER_AMP=0.01,
        MAX_CASE_TEMPERATURE=90.0,
        MAX_WINDING_TEMPERATURE=130.0,
    )
    actuator = MaxonActuator(motor_constants=custom)
    mc = actuator.MOTOR_CONSTANTS  # base class exposes _MOTOR_CONSTANTS via MOTOR_CONSTANTS property
    assert mc.MOTOR_COUNT_PER_REV == 512
    assert pytest.approx(0.01) == mc.NM_PER_AMP


def test_maxon_control_mode_configs(maxon: MaxonActuator):
    configs = maxon._CONTROL_MODE_CONFIGS
    assert configs.POSITION is not None
    assert configs.CURRENT is None
    assert configs.VOLTAGE is None
    assert configs.IMPEDANCE is None
    assert configs.VELOCITY is None


# Control-mode callbacks
def test_maxon_position_mode_exit_calls_stop(maxon: MaxonActuator):
    """_maxon_position_mode_exit must call stop(), zeroing PWM and pins."""
    _maxon_position_mode_exit(maxon)
    assert maxon.speed_control.value == 0
    maxon.ina.off.assert_called()
    maxon.inb.off.assert_called()


def test_maxon_position_mode_exit_via_mode_switch(maxon: MaxonActuator):
    """Switching away from POSITION mode should trigger the exit callback."""
    maxon.set_control_mode(CONTROL_MODES.POSITION)
    maxon.set_motor_pwm(0.15)  # non-zero PWM to confirm stop zeroes it
    maxon.set_control_mode(CONTROL_MODES.VOLTAGE)  # exit callback fires here
    assert maxon.speed_control.value == 0


# Unsupported control modes
def test_maxon_voltage_unsupported(maxon: MaxonActuator):
    maxon.set_control_mode(CONTROL_MODES.VOLTAGE)
    with pytest.raises(UnsupportedControlModeError):
        maxon.set_motor_voltage(1.0)


def test_maxon_current_unsupported(maxon: MaxonActuator):
    maxon.set_control_mode(CONTROL_MODES.CURRENT)
    with pytest.raises(UnsupportedControlModeError):
        maxon.set_motor_current(1.0)


def test_maxon_set_motor_position_not_implemented(maxon: MaxonActuator):
    maxon.set_control_mode(CONTROL_MODES.POSITION)
    with pytest.raises(NotImplementedError):
        maxon.set_motor_position(1.0)


def test_maxon_set_output_impedance_not_implemented(maxon: MaxonActuator):
    maxon.set_control_mode(CONTROL_MODES.IMPEDANCE)
    with pytest.raises(NotImplementedError):
        maxon.set_output_impedance(1.0)


def test_maxon_impedance_gains_not_implemented(maxon: MaxonActuator):
    # _set_impedance_gains is not mode-gated.
    with pytest.raises(NotImplementedError):
        maxon._set_impedance_gains(1.0, 1.0)
    # set_impedance_gains IS mode-gated (IMPEDANCE); enter the mode first.
    maxon.set_control_mode(CONTROL_MODES.IMPEDANCE)
    with pytest.raises(NotImplementedError):
        maxon.set_impedance_gains(1.0, 1.0)


# start()
def test_maxon_start_is_noop(maxon: MaxonActuator):
    """start() is defined as a no-op; calling it must not raise."""
    maxon.start()  # should return None without touching any hardware


# Gains / PWM / direction
def test_maxon_set_position_gains(maxon: MaxonActuator):
    maxon.set_control_mode(CONTROL_MODES.POSITION)
    maxon.set_position_gains(k_p=0.02, k_i=1.0, k_d=0.001)
    assert maxon.k_p == 0.02
    assert maxon.k_i == 1.0
    assert maxon.k_d == 0.001


def test_maxon_pwm_within_range(maxon: MaxonActuator):
    maxon.set_motor_pwm(0.15)
    assert maxon.speed_control.value == 0.15


def test_maxon_pwm_clamps_to_max(maxon: MaxonActuator):
    maxon.set_motor_pwm(0.5)  # above the 0.3 maximum
    assert maxon.speed_control.value == 0.3


def test_maxon_pwm_bumped_to_min(maxon: MaxonActuator):
    maxon.set_motor_pwm(0.05)  # in [lower_limit, minimum) -> minimum
    assert maxon.speed_control.value == 0.07


def test_maxon_pwm_below_lower_limit_zeroed(maxon: MaxonActuator):
    maxon.set_motor_pwm(0.01)  # below the 0.02 lower limit -> 0
    assert maxon.speed_control.value == 0.0


def test_maxon_direction_forward(maxon: MaxonActuator):
    maxon.set_motor_direction_forward()
    maxon.ina.on.assert_called_once()
    maxon.inb.off.assert_called_once()


def test_maxon_direction_backward(maxon: MaxonActuator):
    maxon.set_motor_direction_backward()
    maxon.ina.off.assert_called_once()
    maxon.inb.on.assert_called_once()


def test_maxon_stop(maxon: MaxonActuator):
    maxon.stop()
    assert maxon.speed_control.value == 0
    maxon.ina.off.assert_called_once()
    maxon.inb.off.assert_called_once()


# Encoder
def test_maxon_update_with_encoder(maxon: MaxonActuator):
    encoder = MagicMock()
    encoder.count = 5000
    maxon.set_motor_encoder(encoder)
    maxon.position_control_config()  # sets scale / scale_perc used by cts_to_* helpers
    maxon.update()
    assert maxon.motor_position_cts == 5000
    assert maxon.motor_position_mm == maxon.cts_to_mm(5000)
    assert maxon.motor_position_perc == maxon.cts_to_perc(5000)


def test_maxon_update_without_encoder_defaults_to_zero(maxon: MaxonActuator):
    """update() with no encoder attached must set motor_position_cts to 0."""
    maxon.encoder_counter = None
    maxon.position_control_config()
    maxon.update()
    assert maxon.motor_position_cts == 0.0


def test_maxon_update_reflects_changing_encoder_count(maxon: MaxonActuator):
    """Each update() call should pick up the latest encoder count."""
    encoder = MagicMock()
    maxon.set_motor_encoder(encoder)
    maxon.position_control_config()

    encoder.count = 100
    maxon.update()
    assert maxon.motor_position_cts == 100

    encoder.count = 2048
    maxon.update()
    assert maxon.motor_position_cts == 2048


def test_maxon_unit_conversions(maxon: MaxonActuator):
    maxon.position_control_config(scale_perc=100.0, scale=200.0)
    assert maxon.perc_to_cts(2.0) == 200.0
    assert maxon.cts_to_perc(200.0) == 2.0
    assert maxon.mm_to_cts(3.0) == 600.0
    assert maxon.cts_to_mm(600.0) == 3.0


def test_maxon_position_control_config_derived(maxon: MaxonActuator):
    maxon.position_control_config(
        scale_perc=100.0,
        scale=200.0,
        slider_min_perc=0.5,
        slider_max_perc=99.5,
    )
    assert maxon.slider_min_counts == 0.5 * 100.0
    assert maxon.slider_max_counts == 99.5 * 100.0
    assert maxon.slider_min_mm == (0.5 * 100.0) / 200.0
    assert maxon.slider_max_mm == (99.5 * 100.0) / 200.0


def test_maxon_motor_position_property(maxon: MaxonActuator):
    encoder = MagicMock()
    encoder.count = 1024
    maxon.set_motor_encoder(encoder)
    maxon.position_control_config()
    # motor_position = counts * 2*pi / 1024, so 1024 counts -> 2*pi rad.
    assert maxon.motor_position == pytest.approx(2 * np.pi)


def test_maxon_motor_encoder_position_perc_calls_update(maxon: MaxonActuator):
    """motor_encoder_position_perc must refresh from the encoder on each access."""
    encoder = MagicMock()
    maxon.set_motor_encoder(encoder)
    maxon.position_control_config(scale_perc=1000.0, scale=2000.0)

    encoder.count = 500
    perc = maxon.motor_encoder_position_perc
    assert perc == pytest.approx(maxon.cts_to_perc(500))

    # Simulate the encoder advancing; the property must reflect the new reading.
    encoder.count = 1000
    perc2 = maxon.motor_encoder_position_perc
    assert perc2 == pytest.approx(maxon.cts_to_perc(1000))
    assert perc2 != pytest.approx(perc)


def test_maxon_motor_encoder_position_perc_at_zero(maxon: MaxonActuator):
    """At count 0, the property should return 0.0%."""
    encoder = MagicMock()
    encoder.count = 0
    maxon.set_motor_encoder(encoder)
    maxon.position_control_config(scale_perc=1000.0, scale=2000.0)
    assert maxon.motor_encoder_position_perc == pytest.approx(0.0)


def test_maxon_unsupported_readings(maxon: MaxonActuator):
    assert maxon.motor_velocity == 0.0
    assert maxon.case_temperature == 0.0
    assert maxon.winding_temperature == 0.0


# PID controller
def test_maxon_position_control_init_resets_state(maxon: MaxonActuator):
    # position_control_init calls set_position_gains internally, which is POSITION-gated.
    maxon.set_control_mode(CONTROL_MODES.POSITION)
    maxon.position_control_init(k_p=0.02, k_i=1.0, k_d=0.001)
    assert maxon.k_p == 0.02
    assert maxon.error_encoder_last == 0.0
    assert maxon.d_term_last == 0.0
    assert maxon.d_term_filtered_last == 0.0
    assert maxon.i_term == 0.0
    assert maxon.last_pwm == 0.0


def test_maxon_pid_returns_float_and_updates_last_pwm(maxon: MaxonActuator):
    maxon.set_control_mode(CONTROL_MODES.POSITION)
    maxon.position_control_init()
    pwm = maxon.pid_ctrl_position(error_encoder=1000.0, dt=0.01)
    assert isinstance(pwm, float)
    assert maxon.last_pwm == pwm


def test_maxon_pid_i_term_integrates_when_not_saturated(maxon: MaxonActuator):
    # the integrator now updates when last_pwm sits inside the unsaturated window.
    maxon.set_control_mode(CONTROL_MODES.POSITION)
    maxon.position_control_init()

    # last_pwm starts at 0.0, which should fall inside the window for any reasonable pwm_maximum_command.
    assert -(maxon.pwm_maximum_command - 0.05) < maxon.last_pwm < (maxon.pwm_maximum_command - 0.05)

    maxon.pid_ctrl_position(error_encoder=1000.0, dt=0.01)

    expected_i_term = maxon.k_i * 1000.0 * 0.01
    assert maxon.i_term == pytest.approx(expected_i_term)


def test_maxon_pid_i_term_holds_when_saturated(maxon: MaxonActuator):
    # When last_pwm is outside the guard window, the integrator must not update.
    maxon.set_control_mode(CONTROL_MODES.POSITION)
    maxon.position_control_init()

    maxon.last_pwm = maxon.pwm_maximum_command  # force saturation
    maxon.pid_ctrl_position(error_encoder=1000.0, dt=0.01)

    assert maxon.i_term == 0.0


# check_coupler_drift
def _setup_motor_position_mm(maxon: MaxonActuator, mm: float) -> None:
    """Helper: configure scale and plant an encoder count so motor_position_mm == mm."""
    scale = 1000.0
    maxon.position_control_config(scale_perc=1000.0, scale=scale)
    encoder = MagicMock()
    encoder.count = mm * scale  # cts_to_mm(count) == count / scale == mm
    maxon.set_motor_encoder(encoder)
    maxon.update()


def test_check_coupler_drift_no_warning_when_ok(maxon: MaxonActuator):
    """Positive mm (normal operating range) must not log any warning."""
    _setup_motor_position_mm(maxon, 5.0)
    with patch("opensourceleg.actuators.brushed.LOGGER") as mock_logger:
        maxon.check_coupler_drift()
        mock_logger.warning.assert_not_called()


def test_check_coupler_drift_slight_drift_warning(maxon: MaxonActuator):
    """
    When allowable_coupler_drift < position_mm < 0, a 'drifted a little' warning
    must be issued (first branch only).
    """
    # Default allowable_coupler_drift is -0.2; place position at -0.1.
    _setup_motor_position_mm(maxon, -0.1)
    with patch("opensourceleg.actuators.brushed.LOGGER") as mock_logger:
        maxon.check_coupler_drift()
        warning_msgs = [str(c.args[0]) for c in mock_logger.warning.call_args_list]
        assert any("drifted a little" in m for m in warning_msgs)
        # The severe 'reassembled' warning must NOT appear.
        assert not any("reassembled" in m for m in warning_msgs)


def test_check_coupler_drift_severe_drift_warning(maxon: MaxonActuator):
    """
    When position_mm <= allowable_coupler_drift, the 'should be reassembled' warning
    must be issued (second branch).
    """
    # Default allowable_coupler_drift is -0.2; place position at exactly -0.2.
    _setup_motor_position_mm(maxon, -0.2)
    with patch("opensourceleg.actuators.brushed.LOGGER") as mock_logger:
        maxon.check_coupler_drift()
        warning_msgs = [str(c.args[0]) for c in mock_logger.warning.call_args_list]
        assert any("reassembled" in m for m in warning_msgs)


def test_check_coupler_drift_severe_drift_well_below(maxon: MaxonActuator):
    """Values well below the threshold also trigger the severe warning."""
    _setup_motor_position_mm(maxon, -5.0)
    with patch("opensourceleg.actuators.brushed.LOGGER") as mock_logger:
        maxon.check_coupler_drift()
        warning_msgs = [str(c.args[0]) for c in mock_logger.warning.call_args_list]
        assert any("reassembled" in m for m in warning_msgs)


def test_check_coupler_drift_boundary_just_above_allowable(maxon: MaxonActuator):
    """
    A value just above allowable_coupler_drift but still negative should trigger
    the 'drifted a little' warning only.
    """
    _setup_motor_position_mm(maxon, -0.15)  # -0.2 < -0.15 < 0
    with patch("opensourceleg.actuators.brushed.LOGGER") as mock_logger:
        maxon.check_coupler_drift()
        warning_msgs = [str(c.args[0]) for c in mock_logger.warning.call_args_list]
        assert any("drifted a little" in m for m in warning_msgs)
        assert not any("reassembled" in m for m in warning_msgs)


# Homing
def test_maxon_home_stops_when_stable(maxon: MaxonActuator):
    encoder = MagicMock()
    encoder.count = 100  # constant -> position immediately "stable" -> one loop pass
    maxon.set_motor_encoder(encoder)
    maxon.position_control_config()
    with patch("opensourceleg.actuators.brushed.time.sleep"):
        maxon.home()
    # home() ends by calling stop(), which zeros PWM.
    assert maxon.speed_control.value == 0


def test_maxon_home_invokes_callback(maxon: MaxonActuator):
    encoder = MagicMock()
    encoder.count = 0
    maxon.set_motor_encoder(encoder)
    maxon.position_control_config()
    callback = MagicMock()
    with patch("opensourceleg.actuators.brushed.time.sleep"):
        maxon.home(callback=callback)
    callback.assert_called_once()


def test_maxon_home_zero_sets_direction_backward(maxon: MaxonActuator):
    """home(home_zero=True) must drive the motor backward."""
    encoder = MagicMock()
    encoder.count = 0
    maxon.set_motor_encoder(encoder)
    maxon.position_control_config()
    with patch("opensourceleg.actuators.brushed.time.sleep"):
        maxon.home(home_zero=True)
    # Backward: ina off, inb on (then stop sets both off — check inb.on was called)
    maxon.inb.on.assert_called()


def test_maxon_home_hardstop_sets_direction_forward(maxon: MaxonActuator):
    """home(home_zero=False) must drive the motor forward."""
    encoder = MagicMock()
    encoder.count = 0
    maxon.set_motor_encoder(encoder)
    maxon.position_control_config()
    with patch("opensourceleg.actuators.brushed.time.sleep"):
        maxon.home(home_zero=False)
    maxon.ina.on.assert_called()


def test_maxon_home_no_callback_does_not_raise(maxon: MaxonActuator):
    """home() with no callback should complete cleanly."""
    encoder = MagicMock()
    encoder.count = 0
    maxon.set_motor_encoder(encoder)
    maxon.position_control_config()
    with patch("opensourceleg.actuators.brushed.time.sleep"):
        maxon.home(callback=None)  # must not raise


def test_maxon_home_exits_after_position_stabilises(maxon: MaxonActuator):
    """
    Simulate a motor that moves for two samples then stops.
    home() exits when two consecutive encoder readings differ by less than
    position_threshold, and stop() must be called (speed_control.value == 0).

    The encoder count sequence is driven via a closure attached to a
    unittest.mock.PropertyMock so that attribute access (not a call) returns
    the right value at each update() call.
    """
    from unittest.mock import PropertyMock

    encoder = MagicMock()
    type(encoder).count = PropertyMock(side_effect=[0, 500, 505, 508, 508])

    maxon.set_motor_encoder(encoder)
    maxon.position_control_config()
    with patch("opensourceleg.actuators.brushed.time.sleep"):
        maxon.home(position_threshold=10)

    assert maxon.speed_control.value == 0


# Offline integration
@pytest.fixture
def maxon_offline():
    # No gpiozero mocking needed: offline mode skips the hardware branch entirely.
    return MaxonActuator(offline=True, motor_constants=VALID_MOTOR_CONSTANTS)


def test_maxon_offline_flags(maxon_offline: MaxonActuator):
    assert maxon_offline.is_offline is True
    assert maxon_offline.is_open is True
    assert maxon_offline.is_streaming is True
    assert maxon_offline.tag == "maxon_actuator"


def test_maxon_offline_hardware_methods_are_noops(maxon_offline: MaxonActuator):
    # OfflineMixin stubs these, so they run without any gpiozero devices present.
    maxon_offline.start()
    maxon_offline.stop()
    maxon_offline.update()
    maxon_offline.home()
