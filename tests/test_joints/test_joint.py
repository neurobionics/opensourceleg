import numpy as np
import pytest
from pytest_mock import mocker

from opensourceleg import constants
from opensourceleg.actuators import (
    CurrentMode,
    DephyActpack,
    ImpedanceMode,
    PositionMode,
    VoltageMode,
)
from opensourceleg.joints import Joint
from opensourceleg.logger import Logger
from opensourceleg.units import DEFAULT_UNITS, UnitsDefinition
from tests.test_actuators.test_dephyactpack import (
    Data,
    MockDephyActpack,
    dephyactpack_mock,
    dephyactpack_patched,
    patch_dephyactpack,
)


# Assert the fixtures were correctly imported from test_dephyactpack.py
def test_patching(dephyactpack_patched: DephyActpack):
    patched_dap = dephyactpack_patched
    assert isinstance(patched_dap, MockDephyActpack)


class MockJoint(Joint, MockDephyActpack):
    def __init__(
        self,
        name: str = "knee",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        gear_ratio: float = 41.4999,
        has_loadcell: bool = False,
        logger: Logger = Logger(),
        units: UnitsDefinition = DEFAULT_UNITS,
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

        self._motor_voltage_sp = 0.0
        self._motor_current_sp = 0.0
        self._motor_position_sp = 0.0

        self._stiffness_sp: int = 200
        self._damping_sp: int = 400
        self._equilibrium_position_sp = 0.0

        self._max_temperature: float = constants.MAX_CASE_TEMPERATURE

        self._control_mode_sp: str = "voltage"

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


# Fixture that returns a MockDephyActpack
@pytest.fixture
def joint_mock() -> MockJoint:
    return MockJoint()


# Fixture that patches the DephyActpack class with the newly made MockDephyActpack class
@pytest.fixture
def patch_joint(mocker, joint_mock: MockJoint):
    mocker.patch("opensourceleg.joints.Joint.__new__", return_value=joint_mock)


# Fixture that returns a MockDephyActpack instance when the DephyActpack class is called
@pytest.fixture
def joint_patched(patch_joint) -> Joint:
    obj = Joint()
    return obj


# Fixture that returns a MockDephyActpack instance when the DephyActpack class is called
def test_mockjoint(joint_patched: Joint):
    jp = joint_patched
    assert isinstance(jp, MockJoint)


# Unfinished
def test_home(joint_patched: Joint):
    jp1 = joint_patched
    jp1._data = Data(mot_ang=20, ank_ang=10)
    jp1.home()
    assert jp1.is_homed == True
    assert jp1._mode == VoltageMode(device=jp1)

    assert jp1._motor_command == "Control Mode: c_int(1), Value: 0"


# Unfinished
def test_make_encoder_map(joint_patched: Joint):
    pass


# Test the set_max_temperature method
def test_set_max_temperature(joint_patched: Joint):
    jp4 = joint_patched
    assert jp4._max_temperature == 80
    jp4.set_max_temperature(100)
    assert jp4._max_temperature == 100


# Test the set_output_torque method
def test_set_output_torque(joint_patched: Joint):
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


# Test the set_output_position method
def test_set_output_position(joint_patched: Joint):
    jp6 = joint_patched
    jp6._gear_ratio = 100
    jp6._mode = PositionMode(device=jp6)
    jp6.set_output_position(position=6.0)
    jp6_mot_cmd_value = int(6.0 * 100 / (2 * np.pi / 16384))
    assert jp6._motor_command == "Control Mode: c_int(0), Value: {}".format(
        str(jp6_mot_cmd_value)
    )


# Test the set_motor_impedance method
def test_set_motor_impedance(joint_patched: Joint):
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


# Test the set_joint_impedance method
def test_set_joint_impedance(joint_patched: Joint):
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
        "k": int(0.08922 / 100**2 * 2 * np.pi / 16384 / 0.0007812 * 1e3 / 0.1133),
        "b": int(0.0038070 / 100**2 * np.pi / 180 / 0.00028444 * 1e3 / 0.1133),
        "ff": 128,
    }


# Test the convert_to_joint_impedance method
def test_convert_to_joint_impedance(joint_patched: Joint):
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


# Test the convert_to_motor_impedance method
def test_convert_to_motor_impedance(joint_patched: Joint):
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


# Test the convert_to_pid_impedance method
def test_convert_to_pid_impedance(joint_patched: Joint):
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


# Test the update_set_points method
def test_update_set_points(joint_patched: Joint):
    jp12 = joint_patched
    # Assert the proper mode is set when the control mode is set to the correponding modes
    assert jp12._mode == VoltageMode(device=jp12)
    jp12._control_mode_sp = "current"
    jp12.update_set_points()
    assert jp12._mode == CurrentMode(device=jp12)
    jp12._control_mode_sp = "position"
    jp12.update_set_points()
    assert jp12._mode == PositionMode(device=jp12)
    jp12._control_mode_sp = "impedance"
    jp12.update_set_points()
    assert jp12._mode == ImpedanceMode(device=jp12)


# Test the default properties of the MockJoint class
def test_mockjoint_default_properties(joint_patched: Joint):
    jp1 = joint_patched
    assert jp1.name == "knee"
    assert jp1.gear_ratio == 41.4999
    assert jp1.max_temperature == 80
    assert jp1.is_homed == False
    assert jp1.encoder_map == None
    assert jp1.output_position == 0.0
    assert jp1.output_velocity == 0.0
    assert jp1.joint_torque == 0.0
    assert jp1.motor_current_sp == 0.0
    assert jp1.motor_voltage_sp == 0.0
    assert jp1.motor_position_sp == 0.0
    assert jp1.stiffness_sp == 200
    assert jp1.damping_sp == 400
    assert jp1.equilibirum_position_sp == 0.0
    assert jp1.control_mode_sp == "voltage"


# Test the non-default properties of the MockJoint class
def test_mockjoint_nondefaultproperties(joint_patched):
    jp2 = joint_patched
    jp2._data = Data(mot_ang=20, mot_vel=10, mot_cur=20)
    jp2._name = "ankle"
    jp2._gear_ratio = 50.0
    jp2._max_temperature = 100.0
    jp2._is_homed = True
    # jp2._encoder_map = np.polynomial.polynomial.Polynomial(coef=[1, 2, 3])
    jp2._motor_current_sp = 2.0
    jp2._motor_voltage_sp = 3.0
    jp2._motor_position_sp = 4.0
    jp2._stiffness_sp = 300
    jp2._damping_sp = 500
    jp2._equilibrium_position_sp = 6.0
    jp2._control_mode_sp = "current"

    assert jp2.name == "ankle"
    assert jp2.gear_ratio == 50.0
    assert jp2.max_temperature == 100
    assert jp2.is_homed == True
    assert jp2.encoder_map == None
    assert jp2.output_position == 20 * 2 * np.pi / 16384 / 50
    assert jp2.output_velocity == 10 * 2 * np.pi / 16384 / 50
    assert jp2.joint_torque == 20 * 0.1133 / 1000 * 50
    assert jp2.motor_current_sp == 2.0
    assert jp2.motor_voltage_sp == 3.0
    assert jp2.motor_position_sp == 4.0
    assert jp2.stiffness_sp == 300
    assert jp2.damping_sp == 500
    assert jp2.equilibirum_position_sp == 6.0
    assert jp2.control_mode_sp == "current"
