import time

import numpy as np
import pytest

import opensourceleg.constants as constants
import opensourceleg.utilities as utilities
from opensourceleg.actuators import (
    CurrentMode,
    ImpedanceMode,
    PositionMode,
    VoltageMode,
)
from opensourceleg.joints import Joint
from opensourceleg.loadcell import Loadcell
from opensourceleg.logger import Logger
from opensourceleg.osl import OpenSourceLeg
from opensourceleg.state_machine import State, StateMachine
from opensourceleg.units import DEFAULT_UNITS, UnitsDefinition
from opensourceleg.utilities import SoftRealtimeLoop
from tests.test_actuators.test_dephyactpack import Data
from tests.test_joints.test_joint import (
    MockJoint,
    joint_mock,
    joint_patched,
    patch_joint,
    patch_sleep,
    patch_time_time,
)
from tests.test_loadcell.test_loadcell import (
    MockStrainAmp,
    loadcell_mock,
    loadcell_patched,
    patch_loadcell,
)
from tests.test_state_machine.test_state_machine import mock_time


# Test the OpenSourceLeg constructor
def test_opensourceleg_init(mock_time):
    # Create a new OpenSourceLeg
    test_osl = OpenSourceLeg()

    # Check that the OpenSourceLeg was initialized properly
    assert test_osl._frequency == 200
    assert test_osl._has_knee == False
    assert test_osl._has_ankle == False
    assert test_osl._has_loadcell == False
    assert test_osl._has_tui == False
    assert test_osl._has_sm == False
    assert test_osl._is_homed == False
    assert test_osl._knee == None
    assert test_osl._ankle == None
    assert test_osl._loadcell == None
    assert test_osl._log_data == False
    # assert test_osl.clock
    assert test_osl._units == DEFAULT_UNITS
    assert test_osl.tui == None
    assert test_osl.state_machine == None
    assert test_osl._timestamp == 1.0


# Test the static method to create a new OpenSourceLeg
def test_osl_create(mock_time):
    OpenSourceLeg.get_instance()
    assert OpenSourceLeg._instance == None


# Test the OpenSourceLeg __enter__ method
def test_osl_enter(
    mock_get_active_ports, joint_patched: Joint, loadcell_patched: Loadcell
):
    # Create a new OpenSourceLeg
    test_osl_ent = OpenSourceLeg()
    test_osl_ent.log = Logger(file_path="tests/test_osl/test_osl_ent")
    test_osl_ent.log.set_stream_level("DEBUG")
    test_osl_ent.add_joint(name="knee")
    test_osl_ent.add_joint(name="ankle")
    test_osl_ent.add_loadcell()
    test_osl_ent._knee._data = Data(
        batt_volt=10,
        batt_curr=10,
        mot_volt=10,
        mot_cur=10,
        mot_ang=10,
        ank_ang=10,
        mot_vel=10,
        mot_acc=10,
        ank_vel=10,
        temperature=10,
        genvar_0=10,
        genvar_1=10,
        genvar_2=10,
        genvar_3=10,
        genvar_4=10,
        genvar_5=10,
        accelx=10,
        accely=10,
        accelz=10,
        gyrox=10,
        gyroy=10,
        gyroz=10,
    )
    assert test_osl_ent._knee._data.batt_volt == 10
    test_osl_ent.__enter__()
    assert test_osl_ent._knee._data.batt_volt == 40
    with open("tests/test_osl/test_osl_ent.log", "r") as f1:
        contents1 = f1.read()
        assert (
            "INFO: [LOADCELL] Initiating zeroing routine, please ensure that there is no ground contact force."
            in contents1
        )


# Unfnished: tui
# Test the OpenSourceLeg __exit__ method
def test_osl_exit(mock_get_active_ports, joint_patched: Joint, mock_time):
    # Create a new OpenSourceLeg
    test_osl_ex = OpenSourceLeg()
    test_osl_ex.log = Logger(file_path="tests/test_osl/test_osl_ex")
    test_osl_ex.log.set_stream_level("DEBUG")
    test_osl_ex.add_state_machine()
    test_osl_ex.state_machine._current_state = State(name="test_state")
    test_osl_ex._is_sm_running = True
    test_osl_ex.add_joint(name="knee")
    test_osl_ex._knee._mode = CurrentMode(device=test_osl_ex._knee)
    test_osl_ex.add_joint(name="ankle")
    test_osl_ex._ankle._mode = ImpedanceMode(device=test_osl_ex._ankle)
    test_osl_ex.__exit__(type=None, value=None, tb=None)
    assert test_osl_ex._knee._mode == VoltageMode(device=test_osl_ex._knee)
    assert test_osl_ex._ankle._mode == VoltageMode(device=test_osl_ex._ankle)
    assert test_osl_ex.state_machine._exited == True


# Test the OpenSourceLeg __repr__ method
def test_osl_repr():
    test_osl_r = OpenSourceLeg()
    assert test_osl_r.__repr__() == "OSL object. Frequency: 200 Hz"


# Test the OpenSourceLeg log_data method
def test_osl_log_data():
    test_osl_ld = OpenSourceLeg()
    assert test_osl_ld._log_data == False
    test_osl_ld.log_data()
    assert test_osl_ld._log_data == True


# Monkeypatch the get_active_ports method
@pytest.fixture
def mock_get_active_ports(monkeypatch):
    monkeypatch.setattr(
        "opensourceleg.utilities.get_active_ports", lambda: ["COM1", "COM2", "COM3"]
    )


# Monkeypatch the get_active_ports method
@pytest.fixture
def mock_get_active_ports1(monkeypatch):
    monkeypatch.setattr("opensourceleg.utilities.get_active_ports", lambda: ["COM1"])


# Monkeypatch the get_active_ports method
@pytest.fixture
def mock_get_active_ports0(monkeypatch):
    monkeypatch.setattr("opensourceleg.utilities.get_active_ports", lambda: [])


# Test the OpenSourceLeg add_joint method with no ports available
def test_osl_add_joint_no_ports(mock_get_active_ports0):
    # Create a new OpenSourceLeg
    test_osl_ajnp = OpenSourceLeg()
    test_osl_ajnp.log = Logger(file_path="tests/test_osl/test_osl_ajnp")
    test_osl_ajnp.log.set_stream_level("DEBUG")
    try:
        test_osl_ajnp.add_joint(name="knee")
    except SystemExit:
        with open("tests/test_osl/test_osl_ajnp.log", "r") as f:
            contents = f.read()
            assert (
                "WARNING: No active ports found, please ensure that the joint is connected and powered on."
                in contents
            )
    assert test_osl_ajnp._has_knee == False


# Test the OpenSourceLeg add_joint method with one port available
def test_osl_add_joint_one_port(joint_patched: Joint, mock_get_active_ports1):
    # Create a new OpenSourceLeg
    test_osl_ajop = OpenSourceLeg()
    test_osl_ajop.add_joint(name="knee")
    assert test_osl_ajop._has_knee == True
    assert test_osl_ajop._knee.name == "knee"
    assert test_osl_ajop._knee.port == "COM1"
    assert test_osl_ajop._knee.gear_ratio == 1.0
    assert test_osl_ajop._knee.max_temperature == 80
    assert test_osl_ajop._knee.is_homed == False
    assert test_osl_ajop._knee.encoder_map == None
    assert test_osl_ajop._knee.output_position == 0.0
    assert test_osl_ajop._knee.output_velocity == 0.0
    assert test_osl_ajop._knee.joint_torque == 0.0
    assert test_osl_ajop._knee.motor_current_sp == 0.0
    assert test_osl_ajop._knee.motor_voltage_sp == 0.0
    assert test_osl_ajop._knee.motor_position_sp == 0.0
    assert test_osl_ajop._knee.stiffness_sp == 200
    assert test_osl_ajop._knee.damping_sp == 400
    assert test_osl_ajop._knee.equilibirum_position_sp == 0.0
    assert test_osl_ajop._knee.control_mode_sp == "voltage"


# Test the OpenSourceLeg add_joint method
def test_osl_add_joint_ports_available(joint_patched: Joint, mock_get_active_ports):
    # Create a new OpenSourceLeg
    test_osl_aj = OpenSourceLeg()
    test_osl_aj.log = Logger(file_path="tests/test_osl/test_osl_aj")
    test_osl_aj.log.set_stream_level("DEBUG")
    test_osl_aj.add_joint(name="knee")
    assert test_osl_aj._has_knee == True
    assert test_osl_aj._knee.name == "knee"
    assert test_osl_aj._knee.port == "COM3"
    assert test_osl_aj._knee.gear_ratio == 1.0
    assert test_osl_aj._knee.max_temperature == 80
    assert test_osl_aj._knee.is_homed == False
    assert test_osl_aj._knee.encoder_map == None
    assert test_osl_aj._knee.output_position == 0.0
    assert test_osl_aj._knee.output_velocity == 0.0
    assert test_osl_aj._knee.joint_torque == 0.0
    assert test_osl_aj._knee.motor_current_sp == 0.0
    assert test_osl_aj._knee.motor_voltage_sp == 0.0
    assert test_osl_aj._knee.motor_position_sp == 0.0
    assert test_osl_aj._knee.stiffness_sp == 200
    assert test_osl_aj._knee.damping_sp == 400
    assert test_osl_aj._knee.equilibirum_position_sp == 0.0
    assert test_osl_aj._knee.control_mode_sp == "voltage"
    test_osl_aj.add_joint(name="ankle")
    assert test_osl_aj._has_ankle == True
    assert test_osl_aj._ankle.name == "ankle"
    assert test_osl_aj._ankle.port == "COM2"
    assert test_osl_aj._ankle.gear_ratio == 1.0
    assert test_osl_aj._ankle.max_temperature == 80
    assert test_osl_aj._ankle.is_homed == False
    assert test_osl_aj._ankle.encoder_map == None
    assert test_osl_aj._ankle.output_position == 0.0
    assert test_osl_aj._ankle.output_velocity == 0.0
    assert test_osl_aj._ankle.joint_torque == 0.0
    assert test_osl_aj._ankle.motor_current_sp == 0.0
    assert test_osl_aj._ankle.motor_voltage_sp == 0.0
    assert test_osl_aj._ankle.motor_position_sp == 0.0
    assert test_osl_aj._ankle.stiffness_sp == 200
    assert test_osl_aj._ankle.damping_sp == 400
    assert test_osl_aj._ankle.equilibirum_position_sp == 0.0
    assert test_osl_aj._ankle.control_mode_sp == "voltage"
    test_osl_aj.add_joint(name="loadcell")
    with open("tests/test_osl/test_osl_aj.log", "r") as f:
        contents = f.read()
        assert "[OSL] Joint name is not recognized." in contents


# Test the OpenSourceLeg add_loadcell method
def test_osl_add_loadcell(loadcell_patched: Loadcell):
    test_osl_al = OpenSourceLeg()
    test_osl_al.add_loadcell()
    assert test_osl_al._has_loadcell == True
    assert test_osl_al._loadcell._is_dephy == False
    assert test_osl_al._loadcell._joint == None
    assert test_osl_al._loadcell._amp_gain == 125.0
    assert test_osl_al._loadcell._exc == 5.0
    assert test_osl_al._loadcell._adc_range == 2**12 - 1
    assert test_osl_al._loadcell._offset == (2**12) / 2
    assert test_osl_al._loadcell._lc.bus == 1
    assert test_osl_al._loadcell._lc.addr == 0x66
    assert test_osl_al._loadcell._lc.indx == 0
    assert test_osl_al._loadcell._lc.is_streaming == True
    assert np.array_equal(
        test_osl_al._loadcell._loadcell_matrix, constants.LOADCELL_MATRIX
    )
    assert test_osl_al._loadcell._loadcell_data == None
    assert test_osl_al._loadcell._prev_loadcell_data == None
    assert np.array_equal(
        test_osl_al._loadcell._loadcell_zero, np.zeros(shape=(1, 6), dtype=np.double)
    )
    assert test_osl_al._loadcell._zeroed == False
    assert test_osl_al._loadcell._log == test_osl_al.log


# Test the OpenSourceLeg add_state_machine method
def test_osl_add_state_machine():
    test_osl_asm = OpenSourceLeg()
    test_osl_asm.add_state_machine()
    assert test_osl_asm._has_sm == True


# Test the OpenSourceLeg estop method
def test_osl_estop():
    test_osl_es = OpenSourceLeg()
    test_osl_es.log = Logger(file_path="tests/test_osl/test_osl_es")
    test_osl_es.log.set_stream_level("DEBUG")
    test_osl_es.estop()
    with open("tests/test_osl/test_osl_es.log", "r") as f:
        contents = f.read()
        assert "[OSL] Emergency stop activated." in contents


# Unfinished: Assert the home methods were called
# Test the OpenSourceLeg home method
def test_osl_home(joint_patched: Joint, mock_get_active_ports):
    test_osl_h = OpenSourceLeg()
    test_osl_h.add_joint(name="knee")
    test_osl_h.add_joint(name="ankle")
    test_osl_h._knee._data = Data(mot_ang=20, ank_ang=10)
    test_osl_h.log = Logger(file_path="tests/test_osl/test_osl_h")
    test_osl_h.log.set_stream_level("DEBUG")
    test_osl_h.home()
    with open("tests/test_osl/test_osl_h.log", "r") as f:
        contents = f.read()
        assert "[OSL] Homing knee joint." in contents
        assert "[OSL] Homing ankle joint." in contents
    assert test_osl_h._knee._is_homed == True


# Test the OpenSourceLeg calibrate_loadcell method
def test_osl_calibrate_loadcell(loadcell_patched: Loadcell):
    test_osl_cl = OpenSourceLeg()
    test_osl_cl.add_loadcell()
    test_osl_cl._loadcell._zeroed = True
    test_osl_cl.log = Logger(file_path="tests/test_osl/test_osl_cl")
    test_osl_cl.log.set_stream_level("DEBUG")
    test_osl_cl.calibrate_loadcell()
    with open("tests/test_osl/test_osl_cl.log", "r") as f:
        contents = f.read()
        assert "[OSL] Calibrating loadcell." in contents
    assert test_osl_cl._loadcell._zeroed == False


# Test the OpenSourceLeg calibrate_encoders method
def test_osl_calibrate_encoders(
    joint_patched: Joint, mock_get_active_ports, patch_time_time, patch_sleep
):
    test_osl_ce = OpenSourceLeg()
    test_osl_ce.log = Logger(file_path="tests/test_osl/test_osl_ce")
    test_osl_ce.log.set_stream_level("DEBUG")
    test_osl_ce.add_joint(name="knee")
    test_osl_ce._knee._data = Data(mot_cur=4999)
    test_osl_ce._knee.is_streaming = True
    test_osl_ce._knee.home()
    test_osl_ce.calibrate_encoders()
    with open("tests/test_osl/test_osl_ce.log", "r") as f:
        contents = f.read()
        assert "[OSL] Calibrating encoders." in contents
    test_joint_position_array = [0.005752427954571154, 0.011504855909142308]
    test_output_position_array = [0.00013861305580425867, 0.00027722611160851734]
    test_power = np.arange(4.0)
    test_a_mat = np.array(test_joint_position_array).reshape(-1, 1) ** test_power
    test_beta = np.linalg.lstsq(test_a_mat, test_output_position_array, rcond=None)[0]
    test_coeffs = test_beta[0]
    test_encoder_map = np.polynomial.polynomial.Polynomial(coef=test_coeffs)
    # assert test_osl_ce._knee._encoder_map == test_encoder_map


# Test the OpenSourceLeg reset method
def test_osl_reset(joint_patched: Joint, mock_get_active_ports):
    test_osl_r = OpenSourceLeg()
    test_osl_r.add_joint(name="knee")
    test_osl_r.add_joint(name="ankle")
    test_osl_r._knee._mode = CurrentMode(device=test_osl_r._knee)
    test_osl_r._ankle._mode = ImpedanceMode(device=test_osl_r._ankle)
    test_osl_r.reset()
    assert test_osl_r._knee._mode == VoltageMode(device=test_osl_r._knee)
    assert test_osl_r._ankle._mode == VoltageMode(device=test_osl_r._ankle)
    assert test_osl_r._knee._motor_command == "Control Mode: c_int(1), Value: 0"
    assert test_osl_r._ankle._motor_command == "Control Mode: c_int(1), Value: 0"


# Test the OpenSourceLeg default properties
def test_osl_properties(mock_time):
    test_osl_prop = OpenSourceLeg()
    test_osl_prop.log = Logger(file_path="tests/test_osl/test_osl_prop")
    assert test_osl_prop.timestamp == 1.0
    knee_prop = test_osl_prop.knee
    assert knee_prop == None
    with open("tests/test_osl/test_osl_prop.log", "r") as f:
        contents = f.read()
        assert "WARNING: [OSL] Knee is not connected." in contents
    ankle_prop = test_osl_prop.ankle
    assert ankle_prop == None
    with open("tests/test_osl/test_osl_prop.log", "r") as f:
        contents = f.read()
        assert "WARNING: [OSL] Ankle is not connected." in contents
    loadcell_prop = test_osl_prop.loadcell
    assert loadcell_prop == None
    with open("tests/test_osl/test_osl_prop.log", "r") as f:
        contents = f.read()
        assert "WARNING: [OSL] Loadcell is not connected." in contents
    assert test_osl_prop.units == DEFAULT_UNITS
    assert test_osl_prop.has_knee == False
    assert test_osl_prop.has_ankle == False
    assert test_osl_prop.has_loadcell == False
    assert test_osl_prop.has_state_machine == False
    assert test_osl_prop.has_tui == False
    assert test_osl_prop.is_homed == False
    assert test_osl_prop.is_sm_running == False
