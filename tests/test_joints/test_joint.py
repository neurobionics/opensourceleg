import time

import numpy as np
import pytest
from pytest_mock import mocker

from opensourceleg.hardware.actuators import (
    MAX_CASE_TEMPERATURE,
    CurrentMode,
    DephyActpack,
    ImpedanceMode,
    PositionMode,
    VoltageMode,
)
from opensourceleg.hardware.joints import Joint
from opensourceleg.tools.logger import Logger
from tests.test_actuators.test_dephyactpack import (
    Data,
    MockDephyActpack,
    dephyactpack_mock,
    dephyactpack_patched,
    patch_dephyactpack,
)


def test_patching(dephyactpack_patched: DephyActpack):
    """
    Test the patching of the DephyActpack class with the MockDephyActpack class\n
    Assert the patched class is an instance of the MockDephyActpack class
    """

    patched_dap = dephyactpack_patched
    assert isinstance(patched_dap, MockDephyActpack)


class MockJoint(Joint, MockDephyActpack):
    """
    Mock Joint class for testing the Joint class\n
    Inherits everything from the Joint class and the MockDephyActpack class
    except for the Joint constructor.
    """

    def __init__(
        self,
        name: str = "knee",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        gear_ratio: float = 41.4999,
        has_loadcell: bool = False,
        logger: Logger = Logger(),
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:

        MockDephyActpack.__init__(self, port)
        self._gear_ratio: float = gear_ratio
        self._is_homed: bool = False
        self._has_loadcell: bool = has_loadcell
        self._encoder_map = None

        self._motor_zero_pos = 0.0
        self._joint_zero_pos = 0.0

        self._max_temperature: float = MAX_CASE_TEMPERATURE

        if "knee" in name.lower() or "ankle" in name.lower():
            self._name: str = name
        else:
            self._log.warning(msg=f"Invalid joint name: {name}")
            return

        # if os.path.isfile(path=f"./{self._name}_encoder_map.npy"):
        #     coefficients = np.load(file=f"./{self._name}_encoder_map.npy")
        #     self._encoder_map = np.polynomial.polynomial.Polynomial(coef=coefficients)
        # else:
        #     self._log.debug(
        #         msg=f"[{self._name}] No encoder map found. Please run the calibration routine."
        #     )


def test_mockjoint_init():
    """
    Tests the Joint constructor\n
    """

    mji = MockJoint()
    assert mji._gear_ratio == 41.4999
    assert mji._is_homed == False
    assert mji._has_loadcell == False
    assert mji._encoder_map == None
    assert mji._motor_zero_pos == 0.0
    assert mji._joint_zero_pos == 0.0
    assert mji._max_temperature == MAX_CASE_TEMPERATURE
    assert mji._name == "knee"

    # mji2 = MockJoint(name="invalid")
    # assert mji2._name == None


@pytest.fixture
def joint_mock() -> MockJoint:
    """
    Fixture that returns a MockJoint instance when the Joint class is called
    """

    return MockJoint()


@pytest.fixture
def patch_joint(mocker, joint_mock: MockJoint):
    """
    Fixture that patches the DephyActpack class with the newly made MockDephyActpack class
    """

    mocker.patch("opensourceleg.hardware.joints.Joint.__new__", return_value=joint_mock)


@pytest.fixture
def joint_patched(patch_joint) -> Joint:
    """
    Fixture that returns a Joint instance when the Joint class is called
    """

    obj = Joint()
    return obj


def test_mockjoint(joint_patched: Joint):
    """
    Test the joint_patched fixture\n
    Assert the joint_patched fixture returns an instance of the MockJoint class
    """

    jp = joint_patched
    assert isinstance(jp, MockJoint)


@pytest.fixture
def patch_sleep(monkeypatch):
    """
    Fixture that patches the time.sleep function with a lambda function that does nothing
    """

    monkeypatch.setattr(time, "sleep", lambda x: None)


def test_home(joint_patched: Joint, patch_sleep):
    """
    Test the home method of the Joint class\n
    This test creates an instance of the MockJoint class and sets the _log attribute
    to a log of the lowest level. The _data attribute is set to a Data instance with
    a mot_cur attribute of 4999. The is_streaming attribute is set to True. The home
    methd is called for the default joint. It then asserts the joint is homed, the
    mode is set to VoltageMode, the gear_ratio is set to 41.4999, the proper log messages
    are written, and the proper motor command is sent, and the motor_zero_position is set
    to 15 and the joint_zero_position is set to 15. The same is done for the ankle joint
    with its respective zero positions.
    """

    jp1 = joint_patched
    jp1._log = Logger(file_path="tests/test_joints/test_home_log")
    jp1._log.set_stream_level(level="DEBUG")
    jp1._data = Data(mot_cur=4999)
    jp1.is_streaming = True
    jp1.home()
    assert jp1.is_homed == True
    assert jp1._mode == VoltageMode(device=jp1)
    assert jp1.gear_ratio == 41.4999
    assert jp1._motor_command == "Control Mode: c_int(1), Value: 0"
    with open(file="tests/test_joints/test_home_log.log") as f:
        contents = f.read()
        assert "INFO: [knee] Homing complete." in contents
    assert jp1._motor_zero_position == 15
    assert jp1._joint_zero_position == 15
    jpa = joint_patched
    jpa._name = "ankle"
    jpa._log = Logger(file_path="tests/test_joints/test_home_ankle_log")
    jpa._log.set_stream_level(level="DEBUG")
    jpa._data = Data(mot_cur=4999)
    jpa.is_streaming = True
    jpa.home()
    assert jpa.is_homed == True
    assert jpa._mode == VoltageMode(device=jpa)
    assert jpa.gear_ratio == 41.4999
    assert jpa._motor_command == "Control Mode: c_int(1), Value: 0"
    with open(file="tests/test_joints/test_home_ankle_log.log") as f:
        contents = f.read()
        assert "INFO: [ankle] Homing complete." in contents
    assert jpa._motor_zero_position == 56676
    assert jpa._joint_zero_position == 1380


# Test the home KeyBoardInterrupt
# def test_home_keyboardinterrupt(joint_patched: Joint, patch_sleep, mocker):
#     jp_ki = joint_patched
#     jp_ki._log = Logger(file_path="tests/test_joints/test_home_keyboardinterrupt_log")
#     jp_ki._log.set_stream_level(level="DEBUG")
#     jp_ki._data = Data()
#     jp_ki.is_streaming = True
#     with pytest.raises(KeyboardInterrupt):
#         jp_ki.home()
#         assert jp_ki._motor_command == "Control Mode: c_int(1), Value: 0"
#         with open(file="tests/test_joints/test_home_keyboardinterrupt_log.log", mode="r") as f:
#             contents = f.read()
#             assert "WARNING: Homing interrupted." in contents


@pytest.fixture
def patch_time_time(monkeypatch):
    """
    Fixture that patches the time.time function with a lambda function that returns
    a list of values that are popped off the list each time the time.time function
    is called. This will be used to simulate the time it takes to run the make_encoder_map
    method. Many are needed because the time.time function is called each time a log
    message is written.
    """

    values = [
        0,
        0,
        0,
        4,
        8,
        12,
        16,
        20,
        24,
        28,
        32,
        36,
        40,
        44,
        48,
        52,
        56,
        60,
        64,
        68,
        72,
        76,
        80,
    ]
    monkeypatch.setattr(time, "time", lambda: values.pop(0))


def test_make_knee_encoder_map(joint_patched: Joint, patch_sleep, patch_time_time):
    """
    Test the make_encoder_map method of the Joint class\n
    This test creates an instance of the MockJoint class and sets the _log attribute
    to a log of the lowest level. The _data attribute is set to a Data instance with
    a mot_cur attribute of 4999. The make_encoder_map method is called for the unhomed
    joint. It then asserts the proper log message is written for an unhomed joint. The
    make_encoder_map method is called for the homed joint. It then asserts the proper
    log messages are written, the mode is set to CurrentMode, the proper gains are set,
    the proper motor command is sent, and the proper encoder map is created.
    """

    jp2 = joint_patched
    jp2._log = Logger(file_path="tests/test_joints/test_make_knee_encoder_map_log")
    jp2._log.set_stream_level(level="DEBUG")
    jp2._data = Data(mot_cur=4999)
    jp2.make_encoder_map()
    with open(file="tests/test_joints/test_make_knee_encoder_map_log.log") as f:
        contents = f.read()
        assert "Please home the joint before making the encoder map." in contents
    jp2.is_streaming = True
    jp2.home()
    jp2.make_encoder_map()
    with open(file="tests/test_joints/test_make_knee_encoder_map_log.log") as f:
        contents = f.read()
        assert "Please manually move the joint numerous times through its full range of motion for 10 seconds. \nPress any key to continue."
        assert (
            "INFO: [DephyActpack[knee]] You may now stop moving the joint." in contents
        )
        assert "INFO: [DephyActpack[knee]] Encoder map saved." in contents
    assert jp2._mode == CurrentMode(device=jp2)
    assert jp2._gains == {
        "kp": 40,
        "ki": 400,
        "kd": 0,
        "k": 0,
        "b": 0,
        "ff": 128,
    }
    assert jp2._motor_command == "Control Mode: c_int(2), Value: 0"
    test_joint_position_array = [0.005752427954571154, 0.011504855909142308]
    test_output_position_array = [0.00013861305580425867, 0.00027722611160851734]
    test_power = np.arange(4.0)
    test_a_mat = np.array(test_joint_position_array).reshape(-1, 1) ** test_power
    test_beta = np.linalg.lstsq(test_a_mat, test_output_position_array, rcond=None)[0]
    test_coeffs = test_beta[0]
    test_encoder_map = np.polynomial.polynomial.Polynomial(coef=test_coeffs)
    assert jp2._encoder_map == test_encoder_map


def test_set_max_temperature(joint_patched: Joint):
    """
    Test the set_max_temperature method of the Joint class\n
    This test creates an instance of the MockJoint class asserts the default max_temperature
    is 80. The set_max_temperature method is called with a max_temperature of 100. It then
    asserts the max_temperature is set to 100.
    """

    jp4 = joint_patched
    assert jp4._max_temperature == 80
    jp4.set_max_temperature(100)
    assert jp4._max_temperature == 100


def test_set_output_torque(joint_patched: Joint):
    """
    Test the set_output_torque method of the Joint class\n
    This test creates an instance of the MockJoint class and sets the gear ratio to 100,
    and the mode to Current Mode. The set_output_torque method is called with a torque of
    4.0. It then asserts the proper motor command is sent.
    """

    # Sets up the proper joint conditions for the set_output_torque method
    jp5 = joint_patched
    jp5._gear_ratio = 100
    jp5._mode = CurrentMode(device=jp5)
    jp5.set_output_torque(torque=4.0)
    jp5_mot_cmd_value = int(4.0 / 100 / 0.1133 * 1000)
    # Asserts the proper motor command is sent
    assert jp5._motor_command == "Control Mode: c_int(2), Value: {}".format(
        str(jp5_mot_cmd_value)
    )


def test_set_output_position(joint_patched: Joint):
    """
    Test the set_output_position method of the Joint class\n
    This test creates an instance of the MockJoint class and sets the gear ratio to 100,
    and the mode to Position Mode. The set_output_position method is called with a position
    of 6.0. It then asserts the proper motor command is sent.
    """

    jp6 = joint_patched
    jp6._gear_ratio = 100
    jp6._mode = PositionMode(device=jp6)
    jp6.set_output_position(position=6.0)
    jp6_mot_cmd_value = int(6.0 * 100 / (2 * np.pi / 16384))
    assert jp6._motor_command == "Control Mode: c_int(0), Value: {}".format(
        str(jp6_mot_cmd_value)
    )


def test_set_motor_impedance(joint_patched: Joint):
    """
    Test the set_motor_impedance method of the Joint class\n
    This test creates an instance of the MockJoint class and sets the gear ratio to 100,
    and the mode to Impedance Mode. The set_motor_impedance method is called. It then
    asserts the proper gains are set.
    """

    # Sets up the proper joint conditions for the set_motor_impedance method
    jp7 = joint_patched
    jp7._mode = ImpedanceMode(device=jp7)
    jp7.set_motor_impedance()
    # Asserts the proper gains are set
    assert jp7._gains == {
        "kp": 40,
        "ki": 400,
        "kd": 0,
        "k": int(0.08922 * 2 * np.pi / 16384 / 0.0007812 * 1e3 / 0.1133),
        "b": int(0.0038070 * np.pi / 180 / 0.00028444 * 1e3 / 0.1133),
        "ff": 128,
    }


def test_set_joint_impedance(joint_patched: Joint):
    """
    Test the set_joint_impedance method of the Joint class\n
    This test creates an instance of the MockJoint class and sets the gear ratio to 100,
    and the mode to Impedance Mode. The set_joint_impedance method is called. It then
    asserts the proper gains are set.
    """

    # Sets up the proper joint conditions for the set_joint_impedance method
    jp8 = joint_patched
    jp8._gear_ratio = 100
    jp8._mode = ImpedanceMode(device=jp8)
    jp8.set_joint_impedance()
    # Asserts the proper gains are set
    assert jp8._gains == {
        "kp": 40,
        "ki": 400,
        "kd": 0,
        "k": int(100 / 100**2 * 2 * np.pi / 16384 / 0.0007812 * 1e3 / 0.1133),
        "b": int(3 / 100**2 * np.pi / 180 / 0.00028444 * 1e3 / 0.1133),
        "ff": 128,
    }


def test_convert_to_joint_impedance(joint_patched: Joint):
    """
    Test the convert_to_joint_impedance method of the Joint class\n
    This test creates an instance of the MockJoint class and sets the gear ratio to 100.
    The convert_to_joint_impedance method is called with no arguments. It then asserts
    the proper gains are returned. The convert_to_joint_impedance method is called with
    arguments. It then asserts the proper gains are returned.
    """

    # Sets up the proper joint conditions for the convert_to_joint_impedance method
    jp9 = joint_patched
    jp9._gear_ratio = 100
    # Call the convert_to_joint_impedance method with no arguments
    jp9_pid_stiffness, jp9_pid_damping = jp9.convert_to_joint_impedance()
    # Assert the proper stiffness and damping values are returned
    assert (
        jp9_pid_stiffness
        == (100 / (2 * np.pi / 16384 / 0.0007812 * 1e3 / 0.1133)) * 100**2
    )
    assert jp9_pid_damping == 40 / (np.pi / 180 / 0.00028444 * 1e3 / 0.1133) * 100**2
    # Call the convert_to_joint_impedance method with arguments
    jp9_pid_stiffness, jp9_pid_damping = jp9.convert_to_joint_impedance(K=50, B=80)
    # Assert the proper stiffness and damping values are returned
    assert (
        jp9_pid_stiffness
        == 50 / (2 * np.pi / 16384 / 0.0007812 * 1e3 / 0.1133) * 100**2
    )
    assert jp9_pid_damping == 80 / (np.pi / 180 / 0.00028444 * 1e3 / 0.1133) * 100**2


def test_convert_to_motor_impedance(joint_patched: Joint):
    """
    Test the convert_to_motor_impedance method of the Joint class\n
    This test creates an instance of the MockJoint class. The convert_to_motor_impedance
    method is called with no arguments. It then asserts the proper stiffness and damping
    values are returned. The convert_to_motor_impedance method is called with arguments.
    It then asserts the proper stiffness and damping values are returned.
    """

    jp10 = joint_patched
    # Call the convert_to_motor_impedance method with no arguments
    jp10_pid_stiffness, jp10_pid_damping = jp10.convert_to_motor_impedance()
    # Assert the proper stiffness and damping values are returned
    assert jp10_pid_stiffness == 100 / (2 * np.pi / 16384 / 0.0007812 * 1e3 / 0.1133)
    assert jp10_pid_damping == 40 / (np.pi / 180 / 0.00028444 * 1e3 / 0.1133)
    # Call the convert_to_motor_impedance method with arguments
    jp10_pid_stiffness, jp10_pid_damping = jp10.convert_to_motor_impedance(K=50, B=80)
    # Assert the proper stiffness and damping values are returned
    assert jp10_pid_stiffness == 50 / (2 * np.pi / 16384 / 0.0007812 * 1e3 / 0.1133)
    assert jp10_pid_damping == 80 / (np.pi / 180 / 0.00028444 * 1e3 / 0.1133)


def test_convert_to_pid_impedance(joint_patched: Joint):
    """
    Test the convert_to_pid_impedance method of the Joint class\n
    This test creates an instance of the MockJoint class and sets the gear ratio to 100.
    The convert_to_pid_impedance method is called with no arguments. It then asserts
    the proper stiffness and damping values are returned. The convert_to_pid_impedance
    method is called with arguments. It then asserts the proper stiffness and damping
    values are returned.
    """

    # Sets up the proper joint conditions for the convert_to_pid_impedance method
    jp11 = joint_patched
    jp11._gear_ratio = 100
    # Call the convert_to_pid_impedance method with no arguments
    jp11_pid_stiffness, jp11_pid_damping = jp11.convert_to_pid_impedance()
    # Assert the proper stiffness and damping values are returned
    assert (
        jp11_pid_stiffness
        == 0.08922 / (100**2) * 2 * np.pi / 16384 / 0.0007812 * 1e3 / 0.1133
    )
    assert round(jp11_pid_damping, 15) == round(
        0.0038070 / (100.0**2) * np.pi / 180 / 0.00028444 * 1e3 / 0.1133, 15
    )
    # Call the convert_to_pid_impedance method with arguments
    jp11_pid_stiffness, jp11_pid_damping = jp11.convert_to_pid_impedance(K=0.05, B=0.01)
    # Assert the proper stiffness and damping values are returned
    assert (
        jp11_pid_stiffness
        == 0.05 / (100**2) * 2 * np.pi / 16384 / 0.0007812 * 1e3 / 0.1133
    )
    assert round(jp11_pid_damping, 15) == round(
        0.01 / (100.0**2) * np.pi / 180 / 0.00028444 * 1e3 / 0.1133, 15
    )


def test_mockjoint_default_properties(joint_patched: Joint):
    """
    Test the default properties of the MockJoint class\n
    This test creates an instance of the MockJoint class and asserts the default
    properties are set properly.
    """

    jp1 = joint_patched
    assert jp1.name == "knee"
    assert jp1.gear_ratio == 41.4999
    assert jp1.max_temperature == 80
    assert jp1.is_homed == False
    assert jp1.encoder_map == None
    assert jp1.output_position == 0.0
    assert jp1.output_velocity == 0.0
    assert jp1.joint_torque == 0.0


def test_mockjoint_nondefaultproperties(joint_patched):
    """
    Test the non-default properties of the MockJoint class\n
    This test creates an instance of the MockJoint class and sets the attributes
    to non-default values. It then asserts the properties are set properly.
    """

    jp2 = joint_patched
    jp2._data = Data(mot_ang=20, mot_vel=10, mot_cur=20)
    jp2._name = "ankle"
    jp2._gear_ratio = 50.0
    jp2._max_temperature = 100.0
    jp2._is_homed = True
    # jp2._encoder_map = np.polynomial.polynomial.Polynomial(coef=[1, 2, 3])

    assert jp2.name == "ankle"
    assert jp2.gear_ratio == 50.0
    assert jp2.max_temperature == 100
    assert jp2.is_homed == True
    assert jp2.encoder_map == None
    assert jp2.output_position == 20 * 2 * np.pi / 16384 / 50
    assert jp2.output_velocity == 10 * np.pi / 180 / 50
    assert jp2.joint_torque == 20 * 0.1133 / 1000 * 50
