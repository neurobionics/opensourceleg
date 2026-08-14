from unittest.mock import Mock, patch

import numpy as np
import pytest

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.robots.vso import VSO
from tests.test_actuators.test_actuators_base import MOTOR_CONSTANTS, MockActuator
from tests.test_sensors.test_sensors_base import MockSensor


def _motor_constants():
    return MOTOR_CONSTANTS(
        MOTOR_COUNT_PER_REV=1000,
        NM_PER_AMP=0.1,
        MAX_CASE_TEMPERATURE=100.0,
        MAX_WINDING_TEMPERATURE=150.0,
    )


class MockJointEncoder(MockSensor):
    """
    Sensor mock with the two members _create_linear_joint_mapping actually uses:
    a `position` that advances on each read (so the least-squares fit gets varied
    data) and a `set_encoder_map` to capture the fitted polynomial.
    """

    def __init__(self, tag: str = "joint_encoder_ankle"):
        super().__init__(tag=tag)
        self._position = 0.0
        self.encoder_map = None

    @property
    def position(self):
        self._position += 1.0
        return self._position

    def set_encoder_map(self, encoder_map):
        self.encoder_map = encoder_map


class RecordingActuator(MockActuator):
    """
    MockActuator that records the mode-gated calls _create_linear_joint_mapping
    makes. Defined as a real subclass so the methods exist at __init__ time and
    survive ActuatorBase's method-mutation machinery.
    """

    def __init__(self, *args, **kwargs):
        self.calls = []
        super().__init__(*args, **kwargs)

    def set_position_gains(self, kp, ki, kd, ff):
        self.calls.append(("set_position_gains", kp, ki, kd, ff))

    def set_motor_voltage(self, value):
        self.calls.append(("set_motor_voltage", value))


# Fixtures
@pytest.fixture
def mock_actuator():
    return MockActuator("ankle", 10, _motor_constants())


@pytest.fixture
def mock_sensor():
    return MockSensor(tag="MockSensor")


@pytest.fixture
def vso(mock_actuator, mock_sensor):
    return VSO(
        tag="test_vso",
        actuators={"ankle": mock_actuator},
        sensors={"sensor1": mock_sensor},
    )


# Construction and RobotBase plumbing
def test_vso_init(vso: VSO, mock_actuator: MockActuator, mock_sensor: MockSensor):
    assert vso.tag == "test_vso"
    assert vso.actuators == {"ankle": mock_actuator}
    assert vso.sensors == {"sensor1": mock_sensor}


def test_vso_enter_calls_start(vso: VSO):
    vso.start = Mock()
    assert vso.__enter__() == vso
    vso.start.assert_called_once()


def test_vso_exit_calls_stop(vso: VSO):
    vso.stop = Mock()
    vso.__exit__(None, None, None)
    vso.stop.assert_called_once()


def test_vso_start(vso: VSO, mock_actuator: MockActuator, mock_sensor: MockSensor):
    mock_actuator.start = Mock()
    mock_sensor.start = Mock()
    vso.start()
    mock_actuator.start.assert_called_once()
    mock_sensor.start.assert_called_once()


def test_vso_stop(vso: VSO, mock_actuator: MockActuator, mock_sensor: MockSensor):
    mock_actuator.stop = Mock()
    mock_sensor.stop = Mock()
    vso.stop()
    mock_actuator.stop.assert_called_once()
    mock_sensor.stop.assert_called_once()


def test_vso_update(vso: VSO, mock_actuator: MockActuator, mock_sensor: MockSensor):
    mock_actuator.update = Mock()
    mock_sensor.update = Mock()
    vso.update()
    mock_actuator.update.assert_called_once()
    mock_sensor.update.assert_called_once()


# home()
def test_vso_home_defaults(vso: VSO, mock_actuator: MockActuator):
    mock_actuator.home = Mock()
    vso.home()
    mock_actuator.home.assert_called_once_with(
        homing_pwm=0.25,
        sample_rate=0.05,
        position_threshold=200,
        home_zero=True,
        callback=None,
        timeout_s=8.0,
    )


def test_vso_home_custom_params(vso: VSO, mock_actuator: MockActuator):
    mock_actuator.home = Mock()
    vso.home(
        homing_pwm=0.1,
        sample_rate=0.01,
        position_threshold=50,
        home_zero=False,
        timeout_s=2.0,
    )
    mock_actuator.home.assert_called_once_with(
        homing_pwm=0.1,
        sample_rate=0.01,
        position_threshold=50,
        home_zero=False,
        callback=None,
        timeout_s=2.0,
    )


def test_vso_home_passes_matching_callback(vso: VSO, mock_actuator: MockActuator):
    mock_actuator.home = Mock()
    callback = Mock()
    # Callbacks are looked up by actuator.tag, which here matches the dict key.
    vso.home(callbacks={"ankle": callback})
    assert mock_actuator.home.call_args.kwargs["callback"] is callback


def test_vso_home_callback_keyed_by_tag_not_dict_key():
    # Footgun: callbacks.get(actuator.tag) uses the TAG, but the actuators dict is
    # keyed independently. When they diverge the callback silently never fires.
    actuator = MockActuator("different_tag", 10, _motor_constants())
    actuator.home = Mock()
    robot = VSO(tag="test_vso", actuators={"ankle": actuator}, sensors={})
    robot.home(callbacks={"ankle": Mock()})
    assert actuator.home.call_args.kwargs["callback"] is None


def test_vso_home_all_actuators():
    left = MockActuator("left", 10, _motor_constants())
    right = MockActuator("right", 10, _motor_constants())
    left.home = Mock()
    right.home = Mock()
    robot = VSO(tag="test_vso", actuators={"left": left, "right": right}, sensors={})
    robot.home()
    left.home.assert_called_once()
    right.home.assert_called_once()


def test_vso_home_no_actuators():
    robot = VSO(tag="test_vso", actuators={}, sensors={})
    robot.home()  # should not raise


# Accessor properties
def test_vso_ankle_property(vso: VSO, mock_actuator: MockActuator):
    assert vso.ankle is mock_actuator


def test_vso_ankle_missing_exits():
    robot = VSO(tag="test_vso", actuators={}, sensors={})
    # The property logs and calls exit(1), which raises SystemExit.
    with pytest.raises(SystemExit):
        _ = robot.ankle


def test_vso_joint_encoder_ankle_property():
    encoder = MockJointEncoder()
    robot = VSO(tag="test_vso", actuators={}, sensors={"joint_encoder_ankle": encoder})
    assert robot.joint_encoder_ankle is encoder


def test_vso_joint_encoder_ankle_missing_exits(vso: VSO):
    with pytest.raises(SystemExit):
        _ = vso.joint_encoder_ankle


# make_encoder_linearization_map() dispatch
def test_map_skips_actuator_without_encoder(vso: VSO):
    # vso's only sensor is "sensor1", so there's no "joint_encoder_ankle".
    with patch.object(VSO, "_create_linear_joint_mapping") as mapper:
        vso.make_encoder_linearization_map()
    mapper.assert_not_called()


def test_map_dispatches_when_encoder_present(mock_actuator: MockActuator):
    robot = VSO(
        tag="test_vso",
        actuators={"ankle": mock_actuator},
        sensors={"joint_encoder_ankle": MockJointEncoder()},
    )
    with patch.object(VSO, "_create_linear_joint_mapping") as mapper:
        robot.make_encoder_linearization_map(overwrite=True)
    mapper.assert_called_once_with(
        actuator_key="ankle",
        encoder_key="joint_encoder_ankle",
        overwrite=True,
    )


def test_map_encoder_key_built_from_dict_key_not_tag():
    # The encoder lookup uses the actuators dict key, so a mismatched tag is fine
    # here (unlike callbacks in home()).
    actuator = MockActuator("some_other_tag", 10, _motor_constants())
    robot = VSO(
        tag="test_vso",
        actuators={"ankle": actuator},
        sensors={"joint_encoder_ankle": MockJointEncoder()},
    )
    with patch.object(VSO, "_create_linear_joint_mapping") as mapper:
        robot.make_encoder_linearization_map()
    mapper.assert_called_once()


# _create_linear_joint_mapping()
def test_mapping_aborts_when_not_homed(mock_actuator: MockActuator):
    encoder = MockJointEncoder()
    robot = VSO(
        tag="test_vso",
        actuators={"ankle": mock_actuator},
        sensors={"joint_encoder_ankle": encoder},
    )
    assert mock_actuator.is_homed is False
    robot._create_linear_joint_mapping("ankle", "joint_encoder_ankle")
    # Bails before touching the encoder or changing modes.
    assert encoder.encoder_map is None
    assert mock_actuator.mode == CONTROL_MODES.IDLE


def test_mapping_loads_existing_file(mock_actuator: MockActuator, monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    encoder = MockJointEncoder()
    coeffs = np.array([1.0, 2.0, 3.0, 4.0])
    np.save(f"./{encoder.tag}_linearization_map.npy", coeffs)

    mock_actuator._is_homed = True
    robot = VSO(
        tag="test_vso",
        actuators={"ankle": mock_actuator},
        sensors={"joint_encoder_ankle": encoder},
    )
    robot._create_linear_joint_mapping("ankle", "joint_encoder_ankle", overwrite=False)

    # Loaded the saved polynomial and returned early without entering POSITION mode.
    assert encoder.encoder_map is not None
    assert np.array_equal(encoder.encoder_map.coef, coeffs)
    assert mock_actuator.mode == CONTROL_MODES.IDLE


def test_mapping_full_run(monkeypatch, tmp_path):
    """
    Exercise the full fit path. time is mocked out (the real loop runs for 10 s of
    wall clock) and input() is stubbed. The clock advances 0.5 s per call, so the
    loop terminates after roughly 10 iterations.
    """
    monkeypatch.chdir(tmp_path)
    actuator = RecordingActuator("ankle", 10, _motor_constants())
    actuator._is_homed = True
    encoder = MockJointEncoder()
    robot = VSO(
        tag="test_vso",
        actuators={"ankle": actuator},
        sensors={"joint_encoder_ankle": encoder},
    )

    clock = iter([i * 0.5 for i in range(200)])
    with patch("opensourceleg.robots.vso.time") as mock_time, patch("builtins.input"):
        mock_time.time.side_effect = lambda: next(clock)
        robot._create_linear_joint_mapping("ankle", "joint_encoder_ankle", overwrite=True)

    # Fitted a polynomial, handed it to the encoder, and saved it to disk.
    assert encoder.encoder_map is not None
    assert (tmp_path / f"{encoder.tag}_linearization_map.npy").exists()

    # Gains are set with the hardcoded position defaults.
    assert ("set_position_gains", 0.015, 2, 0.001, 0.0) in actuator.calls


def test_mapping_ends_with_unsupported_voltage_command(monkeypatch, tmp_path):
    """
    _create_linear_joint_mapping sets position gains then runs the data-collection
    loop and saves the map. It does not issue any voltage or PWM command at the end.
    The only actuator call made is set_position_gains with the default PID values.
    """
    monkeypatch.chdir(tmp_path)
    actuator = RecordingActuator("ankle", 10, _motor_constants())
    actuator._is_homed = True
    encoder = MockJointEncoder()
    robot = VSO(
        tag="test_vso",
        actuators={"ankle": actuator},
        sensors={"joint_encoder_ankle": encoder},
    )

    clock = iter([i * 0.5 for i in range(200)])
    with patch("opensourceleg.robots.vso.time") as mock_time, patch("builtins.input"):
        mock_time.time.side_effect = lambda: next(clock)
        robot._create_linear_joint_mapping("ankle", "joint_encoder_ankle", overwrite=True)

    assert ("set_position_gains", 0.015, 2, 0.001, 0.0) in actuator.calls
    assert not any(call[0] == "set_motor_voltage" for call in actuator.calls)
