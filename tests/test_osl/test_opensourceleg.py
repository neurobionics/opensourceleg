import csv
import time

import numpy as np
import pytest

import opensourceleg.tools.utilities as utilities
from opensourceleg.hardware.actuators import (
    CurrentMode,
    ImpedanceMode,
    PositionMode,
    VoltageMode,
)
from opensourceleg.hardware.joints import Joint
from opensourceleg.hardware.sensors import Loadcell
from opensourceleg.osl import OpenSourceLeg
from opensourceleg.tools.logger import Logger
from opensourceleg.tools.utilities import SoftRealtimeLoop
from tests.test_actuators.test_dephyactpack import Data
from tests.test_joints.test_joint import (
    MockJoint,
    joint_mock,
    joint_patched,
    patch_joint,
    patch_sleep,
    patch_time_time,
)
from tests.test_logger.test_logger import Simple_Class
from tests.test_sensors.test_sensors import (
    MockStrainAmp,
    loadcell_mock,
    loadcell_patched,
    patch_loadcell,
)
from tests.test_state_machine.test_state_machine import mock_time

LOADCELL_MATRIX = np.array(
    [
        (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
        (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
        (
            -1047.16800,
            8.63900,
            -1047.28200,
            -20.70000,
            -1073.08800,
            -8.92300,
        ),
        (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
        (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
        (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
    ]
)


def test_opensourceleg_init(mock_time):
    """
    Tests the OpenSourceLeg constructor\n
    Asserts the constructor works properly for the default case.
    """

    test_osl = OpenSourceLeg()

    # Check that the OpenSourceLeg was initialized properly
    assert test_osl._frequency == 200
    assert test_osl._has_knee == False
    assert test_osl._has_ankle == False
    assert test_osl._has_loadcell == False
    assert test_osl._is_homed == False
    assert test_osl._knee == None
    assert test_osl._ankle == None
    assert test_osl._loadcell == None
    # assert test_osl.clock
    assert test_osl._timestamp == 1.0


# # Test the static method to create a new OpenSourceLeg
# def test_osl_create(mock_time):
#     OpenSourceLeg.get_instance()
#     assert OpenSourceLeg._instance == None


@pytest.fixture
def patch_input(monkeypatch):
    monkeypatch.setattr("builtins.input", lambda _: "y")


def test_osl_enter(
    mock_get_active_ports,
    joint_patched: Joint,
    loadcell_patched: Loadcell,
    patch_sleep,
    patch_input,
):
    """
    Tests the OpenSourceLeg __enter__ method\n
    Intializes an OpenSourceLeg object with a logger of the lowest stream level. A
    knee, ankle, and loadcell are added. Data is added to the knee, then the enter
    method is called and asserts the attributes have been updated and the proper log
    message has been written.
    """

    test_osl_ent = OpenSourceLeg()
    test_osl_ent.log = Logger(file_path="tests/test_osl/test_osl_ent")
    test_osl_ent.log.set_stream_level("DEBUG")
    test_osl_ent.add_joint(name="knee")
    test_osl_ent.add_joint(name="ankle")
    test_osl_ent.add_loadcell(loadcell_matrix=LOADCELL_MATRIX)
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
    with open("tests/test_osl/test_osl_ent.log") as f1:
        contents1 = f1.read()
        assert (
            "INFO: [Loadcell] Initiating zeroing routine, please ensure that there is no ground contact force."
            in contents1
        )


def test_osl_exit(mock_get_active_ports, joint_patched: Joint, mock_time, patch_sleep):
    """
    Tests the OpenSourceLeg __exit__ method\n
    Intializes an OpenSourceLeg object with a logger of the lowest stream level.
    A knee and ankle are added and the mode of each is set.
    """

    test_osl_ex = OpenSourceLeg()
    test_osl_ex.log = Logger(file_path="tests/test_osl/test_osl_ex")
    test_osl_ex.log.set_stream_level("DEBUG")
    test_osl_ex.add_joint(name="knee")
    test_osl_ex._knee._mode = CurrentMode(device=test_osl_ex._knee)
    test_osl_ex.add_joint(name="ankle")
    test_osl_ex._ankle._mode = ImpedanceMode(device=test_osl_ex._ankle)
    test_osl_ex.__exit__(type=None, value=None, tb=None)
    assert test_osl_ex._knee._mode == VoltageMode(device=test_osl_ex._knee)
    assert test_osl_ex._ankle._mode == VoltageMode(device=test_osl_ex._ankle)


def test_osl_repr():
    """
    Tests the OpenSourceLeg __repr__ method\n
    Intializes an OpenSourceLeg object and asserts the repr method returns the proper
    string.
    """

    test_osl_r = OpenSourceLeg()
    assert test_osl_r.__repr__() == "OSL"


@pytest.fixture
def mock_get_active_ports(monkeypatch):
    """
    Monkeypatches the get_active_ports method\n
    Returns a list of 3 ports.
    """

    monkeypatch.setattr(
        "opensourceleg.tools.utilities.get_active_ports",
        lambda: ["COM1", "COM2", "COM3"],
    )


@pytest.fixture
def mock_get_active_ports1(monkeypatch):
    """
    Monkeypatches the get_active_ports method\n
    Returns a list of 1 port.
    """

    monkeypatch.setattr(
        "opensourceleg.tools.utilities.get_active_ports", lambda: ["COM1"]
    )


@pytest.fixture
def mock_get_active_ports0(monkeypatch):
    """
    Monkeypatches the get_active_ports method\n
    Returns a list of 0 ports.
    """

    monkeypatch.setattr("opensourceleg.tools.utilities.get_active_ports", lambda: [])


def test_osl_add_joint_no_ports(mock_get_active_ports0):
    """
    Tests the OpenSourceLeg add_joint method with no ports available\n
    Intializes an OpenSourceLeg object with a logger of the lowest stream level. Then
    the add_joint method is called and it is asserted that the proper log message is
    written and the knee was not added.
    """

    test_osl_ajnp = OpenSourceLeg()
    test_osl_ajnp.log = Logger(file_path="tests/test_osl/test_osl_ajnp")
    test_osl_ajnp.log.set_stream_level("DEBUG")
    try:
        test_osl_ajnp.add_joint(name="knee")
    except SystemExit:
        with open("tests/test_osl/test_osl_ajnp.log") as f:
            contents = f.read()
            assert (
                "WARNING: No active ports found, please ensure that the motor is connected and powered on."
                in contents
            )
    assert test_osl_ajnp._has_knee == False


def test_osl_add_joint_one_port(joint_patched: Joint, mock_get_active_ports1):
    """
    Tests the OpenSourceLeg add_joint method with one port available\n
    Intializes an OpenSourceLeg object and adds a knee joint. It then asserts
    that the knee was added properly and the attributes were updated properly.
    """

    test_osl_ajop = OpenSourceLeg()
    test_osl_ajop.add_joint(name="knee")
    assert test_osl_ajop._has_knee == True
    assert test_osl_ajop._knee.name == "knee"
    assert test_osl_ajop._knee.port == "/dev/ttyACM0"
    assert test_osl_ajop._knee.gear_ratio == 1.0
    assert test_osl_ajop._knee.max_case_temperature == 80
    assert test_osl_ajop._knee.is_homed == False
    assert test_osl_ajop._knee.encoder_map == None
    assert test_osl_ajop._knee.output_position == 0.0
    assert test_osl_ajop._knee.output_velocity == 0.0
    assert test_osl_ajop._knee.joint_torque == 0.0


def test_osl_add_joint_ports_available(joint_patched: Joint, mock_get_active_ports):
    """
    Tests the OpenSourceLeg add_joint method with ports available\n
    Intializes an OpenSourceLeg object and adds a knee joint. It then asserts
    that the knee was added properly and the attributes were updated properly.
    Then an ankle joint is added and it is asserted that the ankle was added
    properly and the attributes were updated properly. Then an invalid joint is
    added and it is asserted that the proper log message was written.
    """

    test_osl_aj = OpenSourceLeg()
    test_osl_aj.log = Logger(file_path="tests/test_osl/test_osl_aj")
    test_osl_aj.log.set_stream_level("DEBUG")
    test_osl_aj.add_joint(name="knee")
    assert test_osl_aj._has_knee == True
    assert test_osl_aj._knee.name == "knee"
    assert test_osl_aj._knee.port == "/dev/ttyACM0"
    assert test_osl_aj._knee.gear_ratio == 1.0
    assert test_osl_aj._knee.max_case_temperature == 80
    assert test_osl_aj._knee.is_homed == False
    assert test_osl_aj._knee.encoder_map == None
    assert test_osl_aj._knee.output_position == 0.0
    assert test_osl_aj._knee.output_velocity == 0.0
    assert test_osl_aj._knee.joint_torque == 0.0
    test_osl_aj.add_joint(name="ankle")
    assert test_osl_aj._has_ankle == True
    assert test_osl_aj._ankle.name == "ankle"
    assert test_osl_aj._ankle.port == "/dev/ttyACM0"
    assert test_osl_aj._ankle.gear_ratio == 1.0
    assert test_osl_aj._ankle.max_case_temperature == 80
    assert test_osl_aj._ankle.is_homed == False
    assert test_osl_aj._ankle.encoder_map == None
    assert test_osl_aj._ankle.output_position == 0.0
    assert test_osl_aj._ankle.output_velocity == 0.0
    assert test_osl_aj._ankle.joint_torque == 0.0
    test_osl_aj.add_joint(name="loadcell")
    with open("tests/test_osl/test_osl_aj.log") as f:
        contents = f.read()
        assert "[OSL] Joint name is not recognized." in contents


def test_osl_add_loadcell(loadcell_patched: Loadcell):
    """
    Tests the OpenSourceLeg add_loadcell method\n
    Intializes an OpenSourceLeg object and adds a loadcell. It then asserts
    that the loadcell was added properly and the attributes were updated properly.
    """

    test_osl_al = OpenSourceLeg()
    test_osl_al.add_loadcell(loadcell_matrix=LOADCELL_MATRIX)
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
    assert np.array_equal(test_osl_al._loadcell._loadcell_matrix, LOADCELL_MATRIX)
    assert test_osl_al._loadcell._loadcell_data == None
    assert test_osl_al._loadcell._prev_loadcell_data == None
    assert np.array_equal(
        test_osl_al._loadcell._loadcell_zero, np.zeros(shape=(1, 6), dtype=np.double)
    )
    assert test_osl_al._loadcell._zeroed == False
    assert test_osl_al._loadcell._log == test_osl_al.log


@pytest.fixture
def patch_exit(monkeypatch):
    """
    Monkeypatches the exit method\n
    Returns None.
    """

    monkeypatch.setattr("builtins.exit", lambda: None)


def test_osl_update_knee(
    joint_patched: Joint, mock_get_active_ports, patch_sleep, patch_exit
):
    """
    Tests the OpenSourceLeg update method with knee\n
    Intializes an OpenSourceLeg object and a knee is added. Then the update
    method is called and it is asserted that the knee was updated properly
    and the proper log message was written.
    """

    test_osl_u_knee = OpenSourceLeg()
    test_osl_u_knee.log = Logger(file_path="tests/test_osl/test_osl_u_knee")
    test_osl_u_knee.log.set_stream_level("DEBUG")
    test_osl_u_knee.add_joint(name="knee")
    test_osl_u_knee._knee._log = test_osl_u_knee.log
    test_osl_u_knee._knee._data = Data()
    test_osl_u_knee._knee.is_streaming = True
    test_osl_u_knee._knee.set_max_case_temperature(1.0)
    test_osl_u_knee.update()
    assert test_osl_u_knee._knee._data.batt_volt == 15
    with open("tests/test_osl/test_osl_u_knee.log") as f:
        contents = f.read()
        assert (
            "ERROR: [KNEE] Case thermal limit 1.0 reached. Stopping motor." in contents
        )
        assert "ERROR: [OSL] Software thermal limit exceeded. Exiting." in contents


def test_osl_update_ankle(
    joint_patched: Joint, mock_get_active_ports, patch_sleep, patch_exit
):
    """
    Tests the OpenSourceLeg update method with ankle\n
    Intializes an OpenSourceLeg object and an ankle is added. Then the update
    method is called and it is asserted that the ankle was updated properly
    and the proper log message was written.
    """

    test_osl_u_ankle = OpenSourceLeg()
    test_osl_u_ankle.log = Logger(file_path="tests/test_osl/test_osl_u_ankle")
    test_osl_u_ankle.log.set_stream_level("DEBUG")
    test_osl_u_ankle.add_joint(name="ankle")
    test_osl_u_ankle._ankle._log = test_osl_u_ankle.log
    test_osl_u_ankle._ankle._data = Data()
    test_osl_u_ankle._ankle.is_streaming = True
    test_osl_u_ankle._ankle._max_case_temperature = 1.0
    test_osl_u_ankle.update()
    assert test_osl_u_ankle._ankle._data.batt_volt == 15
    with open("tests/test_osl/test_osl_u_ankle.log") as f:
        contents = f.read()
        assert (
            "ERROR: [ANKLE] Case thermal limit 1.0 reached. Stopping motor." in contents
        )

        assert "ERROR: [OSL] Software thermal limit exceeded. Exiting." in contents


def test_osl_update_loadcell(loadcell_patched: Loadcell, patch_sleep):
    """
    Test the OpenSourceLeg update method with loadcell\n
    Intializes an OpenSourceLeg object with a logger of the lowest stream level
    and a loadcell is added. A joint is added to the loadcell and the data attribute
    of the joint is initialized. Then the update method is called and it is asserted
    that the loadcell was updated properly to a few siginificant figures.
    """

    test_osl_u_loadcell = OpenSourceLeg()
    test_osl_u_loadcell.log = Logger(file_path="tests/test_osl/test_osl_u_loadcell")
    test_osl_u_loadcell.log.set_stream_level("DEBUG")
    test_osl_u_loadcell.add_loadcell(loadcell_matrix=LOADCELL_MATRIX)
    test_osl_u_loadcell._loadcell._joint = joint_patched
    test_osl_u_loadcell._loadcell._joint._data = Data(
        genvar_0=1, genvar_1=2, genvar_2=3, genvar_3=4, genvar_4=5, genvar_5=6
    )
    test_osl_u_loadcell.update()
    loadcell_coupled = [
        ((1 - (2**12) / 2) / (2**12 - 1) * 5.0) * 1000 / (5.0 * 125.0),
        ((2 - (2**12) / 2) / (2**12 - 1) * 5.0) * 1000 / (5.0 * 125.0),
        ((3 - (2**12) / 2) / (2**12 - 1) * 5.0) * 1000 / (5.0 * 125.0),
        ((4 - (2**12) / 2) / (2**12 - 1) * 5.0) * 1000 / (5.0 * 125.0),
        ((5 - (2**12) / 2) / (2**12 - 1) * 5.0) * 1000 / (5.0 * 125.0),
        ((6 - (2**12) / 2) / (2**12 - 1) * 5.0) * 1000 / (5.0 * 125.0),
    ]
    loadcell_signed = [
        [
            loadcell_coupled[0] * -38.72600,
            loadcell_coupled[0] * -1817.74700,
            loadcell_coupled[0] * 9.84900,
            loadcell_coupled[0] * 43.37400,
            loadcell_coupled[0] * -44.54000,
            loadcell_coupled[0] * 1824.67000,
        ],
        [
            loadcell_coupled[1] * -8.61600,
            loadcell_coupled[1] * 1041.14900,
            loadcell_coupled[1] * 18.86100,
            loadcell_coupled[1] * -2098.82200,
            loadcell_coupled[1] * 31.79400,
            loadcell_coupled[1] * 1058.6230,
        ],
        [
            loadcell_coupled[2] * -1047.16800,
            loadcell_coupled[2] * 8.63900,
            loadcell_coupled[2] * -1047.28200,
            loadcell_coupled[2] * -20.70000,
            loadcell_coupled[2] * -1073.08800,
            loadcell_coupled[2] * -8.92300,
        ],
        [
            loadcell_coupled[3] * 20.57600,
            loadcell_coupled[3] * -0.04000,
            loadcell_coupled[3] * -0.24600,
            loadcell_coupled[3] * 0.55400,
            loadcell_coupled[3] * -21.40800,
            loadcell_coupled[3] * -0.47600,
        ],
        [
            loadcell_coupled[4] * -12.13400,
            loadcell_coupled[4] * -1.10800,
            loadcell_coupled[4] * 24.36100,
            loadcell_coupled[4] * 0.02300,
            loadcell_coupled[4] * -12.14100,
            loadcell_coupled[4] * 0.79200,
        ],
        [
            loadcell_coupled[5] * -0.65100,
            loadcell_coupled[5] * -28.28700,
            loadcell_coupled[5] * 0.02200,
            loadcell_coupled[5] * -25.23000,
            loadcell_coupled[5] * 0.47300,
            loadcell_coupled[5] * -27.3070,
        ],
    ]
    loadcell_signed_added_and_transposed = [
        [
            loadcell_signed[0][0]
            + loadcell_signed[0][1]
            + loadcell_signed[0][2]
            + loadcell_signed[0][3]
            + loadcell_signed[0][4]
            + loadcell_signed[0][5],
            loadcell_signed[1][0]
            + loadcell_signed[1][1]
            + loadcell_signed[1][2]
            + loadcell_signed[1][3]
            + loadcell_signed[1][4]
            + loadcell_signed[1][5],
            loadcell_signed[2][0]
            + loadcell_signed[2][1]
            + loadcell_signed[2][2]
            + loadcell_signed[2][3]
            + loadcell_signed[2][4]
            + loadcell_signed[2][5],
            loadcell_signed[3][0]
            + loadcell_signed[3][1]
            + loadcell_signed[3][2]
            + loadcell_signed[3][3]
            + loadcell_signed[3][4]
            + loadcell_signed[3][5],
            loadcell_signed[4][0]
            + loadcell_signed[4][1]
            + loadcell_signed[4][2]
            + loadcell_signed[4][3]
            + loadcell_signed[4][4]
            + loadcell_signed[4][5],
            loadcell_signed[5][0]
            + loadcell_signed[5][1]
            + loadcell_signed[5][2]
            + loadcell_signed[5][3]
            + loadcell_signed[5][4]
            + loadcell_signed[5][5],
        ],
        [0, 0, 0, 0, 0, 0],
    ]
    # Assert the proper values are returned with a couple significant figures
    assert round(test_osl_u_loadcell._loadcell.fx, -2) == round(
        loadcell_signed_added_and_transposed[0][0], -2
    )
    assert round(test_osl_u_loadcell._loadcell.fy) == round(
        loadcell_signed_added_and_transposed[0][1]
    )
    assert round(test_osl_u_loadcell._loadcell.fz, -3) == round(
        loadcell_signed_added_and_transposed[0][2], -3
    )
    assert round(test_osl_u_loadcell._loadcell.mx) == round(
        loadcell_signed_added_and_transposed[0][3]
    )
    assert round(test_osl_u_loadcell._loadcell.my) == round(
        loadcell_signed_added_and_transposed[0][4]
    )
    assert round(test_osl_u_loadcell._loadcell.mz, -1) == round(
        loadcell_signed_added_and_transposed[0][5], -1
    )


def test_osl_update_log_data(joint_patched: Joint, mock_get_active_ports, patch_sleep):
    """
    Tests the OpenSourceLeg update method with log_data\n
    Intializes an OpenSourceLeg object with a logger of the lowest stream level
    and a knee is added. Data is added to the knee and the knee is set to streaming.
    Attributes are added to the logger and the update method is called. It is asserted
    that the data was logged properly.
    """

    test_osl_u_ld = OpenSourceLeg()
    test_osl_u_ld.log = Logger(file_path="tests/test_osl/test_osl_u_ld")
    test_osl_u_ld.log.set_stream_level("DEBUG")
    test_osl_u_ld.add_joint(name="knee")
    test_osl_u_ld._knee._data = Data()
    test_osl_u_ld._knee.is_streaming = True
    test_class_instance = Simple_Class()
    test_osl_u_ld.log.add_attributes(
        container=test_class_instance, attributes=["a", "b", "c"]
    )
    test_osl_u_ld.update()
    expected_rows = [["a", "b", "c"], ["1", "2", "3"]]
    with open("tests/test_osl/test_osl_u_ld.csv", newline="") as f:
        reader = csv.reader(f)
        rows = list(reader)
        assert rows == expected_rows


def test_osl_home(joint_patched: Joint, mock_get_active_ports):
    """
    Tests the OpenSourceLeg home method\n
    Intializes an OpenSourceLeg object with a logger of the lowest stream level
    and a knee and ankle are added. Data is added to the knee and the home method
    is called. It is asserted that the proper log messages were written and the
    knee and anke were homed.
    """

    test_osl_h = OpenSourceLeg()
    test_osl_h.add_joint(name="knee")
    test_osl_h.add_joint(name="ankle")
    test_osl_h._knee._data = Data(mot_ang=20, ank_ang=10)
    test_osl_h.log = Logger(file_path="tests/test_osl/test_osl_h")
    test_osl_h.log.set_stream_level("DEBUG")
    test_osl_h.home()
    with open("tests/test_osl/test_osl_h.log") as f:
        contents = f.read()
        assert "[OSL] Homing knee joint." in contents
        assert "[OSL] Homing ankle joint." in contents
    assert test_osl_h._knee._is_homed == True
    assert test_osl_h._ankle._is_homed == True


def test_osl_calibrate_loadcell(loadcell_patched: Loadcell, patch_input):
    """
    Tests the OpenSourceLeg calibrate_loadcell method\n
    Intializes an OpenSourceLeg object with a logger of the lowest stream level
    and a loadcell is added. The loadcell is zeroed and the calibrate_loadcell
    method is called. It is asserted that the proper log message was written and
    the loadcell was calibrated.
    """

    test_osl_cl = OpenSourceLeg()
    test_osl_cl.add_loadcell(loadcell_matrix=LOADCELL_MATRIX)
    test_osl_cl.log = Logger(file_path="tests/test_osl/test_osl_cl")
    test_osl_cl.log.set_stream_level("DEBUG")
    test_osl_cl.calibrate_loadcell()
    with open("tests/test_osl/test_osl_cl.log") as f:
        contents = f.read()
        assert "[OSL] Calibrating loadcell." in contents
    assert test_osl_cl._loadcell._zeroed == True


def test_osl_reset(joint_patched: Joint, mock_get_active_ports, patch_sleep):
    """
    Tests the OpenSourceLeg reset method\n
    Intializes an OpenSourceLeg object with a knee and ankle added. The modes
    are set to current and impedance respectively. The reset method is called
    and it is asserted that the knee and ankle were changed to voltage mode.
    It is also asserted that the motor commands were sent properly.
    """

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


def test_osl_properties(mock_time):
    """
    Tests the OpenSourceLeg properties\n
    Intializes an OpenSourceLeg object with a logger. The properties are asserted
    to be the default values. It is also asserted that the proper log messages
    were written.
    """

    test_osl_prop = OpenSourceLeg()
    test_osl_prop.log = Logger(file_path="tests/test_osl/test_osl_prop")
    assert test_osl_prop.timestamp == 1.0
    knee_prop = test_osl_prop.knee
    assert knee_prop == None
    with open("tests/test_osl/test_osl_prop.log") as f:
        contents = f.read()
        assert "WARNING: [OSL] Knee is not connected." in contents
    ankle_prop = test_osl_prop.ankle
    assert ankle_prop == None
    with open("tests/test_osl/test_osl_prop.log") as f:
        contents = f.read()
        assert "WARNING: [OSL] Ankle is not connected." in contents
    loadcell_prop = test_osl_prop.loadcell
    assert loadcell_prop == None
    with open("tests/test_osl/test_osl_prop.log") as f:
        contents = f.read()
        assert "WARNING: [OSL] Loadcell is not connected." in contents
    assert test_osl_prop.has_knee == False
    assert test_osl_prop.has_ankle == False
    assert test_osl_prop.has_loadcell == False
    assert test_osl_prop.is_homed == False
