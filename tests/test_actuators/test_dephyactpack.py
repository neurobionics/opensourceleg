from typing import Any

import numpy as np
import pytest
from flexsea.device import Device
from pytest_mock import mocker

from opensourceleg.hardware.actuators import (
    ActpackControlModes,
    ActpackMode,
    CurrentMode,
    DephyActpack,
    ImpedanceMode,
    PositionMode,
    VoltageMode,
)
from opensourceleg.hardware.thermal import ThermalModel
from opensourceleg.tools.logger import Logger


# MockDephyActpack class definition for testing
# This class inherits everything from the DephyActpack class but deletes the super().__init__() call in the constructor so the constructor does not try to connect to a device. It also overrides some of the methods.
class MockDephyActpack(DephyActpack):
    """
    MockDephyActpack class definition for testing.\n
    This class inherits everything from the DephyActpack class but
    deletes the super().__init__() call in the constructor so the
    constructor does not try to connect to a device. It also overrides
    some of the methods to allow for testing without a device, and adds
    attributes used to determine if the methods were called properly.
    """

    def __init__(
        self,
        name: str = "MockDephyActpack",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        logger: Logger = Logger(),
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:
        """
        Initializes the MockDephyActpack class

        Args:
            name (str): _description_. Defaults to "MockDephyActpack".
            port (str): _description_
            baud_rate (int): _description_. Defaults to 230400.
            frequency (int): _description_. Defaults to 500.
            logger (Logger): _description_
            debug_level (int): _description_. Defaults to 0.
            dephy_log (bool): _description_. Defaults to False.
        """
        self._debug_level: int = debug_level
        self._dephy_log: bool = dephy_log
        self._frequency: int = frequency
        self._data: Any = None
        self._name: str = name

        self._log: Logger = logger
        self._state = None

        # New attributes to be used for testing

        # This is used in the open() method to display the port the device should be connected to
        self.port: str = port

        # This is used in the send_motor_command() method to display the motor command that was sent
        self._motor_command: str = "None"

        # This is used in the set_gains() method to display the gains that were set
        self._gains: dict[str, float] = {
            "kp": 0,
            "ki": 0,
            "kd": 0,
            "k": 0,
            "b": 0,
            "ff": 0,
        }

        # This is used in the read() method to indicate a data stream
        self.is_streaming: bool = False

        self._encoder_map = None

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0

        self._joint_offset = 0.0
        self._motor_offset = 0.0

        self._joint_direction = 1.0

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=80,
            soft_border_C_windings=10,
            temp_limit_case=70,
            soft_border_C_case=10,
        )

        self.control_modes: ActpackControlModes = ActpackControlModes(device=self)

        self._mode: ActpackMode = self.control_modes.voltage
        self._max_case_temperature = 80
        self._max_winding_temperature = 110

    # Overrides the open method to function without a device
    def open(self, freq, log_level, log_enabled):
        if freq == 100 and log_level == 5 and log_enabled:
            raise OSError
        else:
            self._log.debug(msg=f"Opening Device at {self.port}")

    # Overrides the send_motor_command method to set the new _motor_command attribute
    def send_motor_command(self, ctrl_mode, value):
        self._motor_command = f"Control Mode: {ctrl_mode}, Value: {value}"

    # Overrides the set_gains method to set the gains in the new _gains attribute
    def set_gains(self, kp, ki, kd, k, b, ff):
        self._gains["kp"] = kp
        self._gains["ki"] = ki
        self._gains["kd"] = kd
        self._gains["k"] = k
        self._gains["b"] = b
        self._gains["ff"] = ff

    # Overrides the read method to modify the data incrementally instead of through a device data stream
    def read(self):
        self._data.batt_volt += 15
        self._data.batt_curr += 15
        self._data.mot_volt += 15
        self._data.mot_cur += 15
        self._data.mot_ang += 15
        self._data.ank_ang += 15
        self._data.mot_vel += 15
        self._data.mot_acc += 15
        self._data.ank_vel += 15
        self._data.temperature += 15
        self._data.genvar_0 += 15
        self._data.genvar_1 += 15
        self._data.genvar_2 += 15
        self._data.genvar_3 += 15
        self._data.genvar_4 += 15
        self._data.genvar_5 += 15
        self._data.accelx += 15
        self._data.accely += 15
        self._data.accelz += 15
        self._data.gyrox += 15
        self._data.gyroy += 15
        self._data.gyroz += 15
        self._data.status_ex = 0b00000000

        return self._data

    # Overrides the close method to do nothing
    def close(self):
        pass


# MockData class definition for testing without a data stream
class Data:
    def __init__(
        self,
        batt_volt=0,
        batt_curr=0,
        mot_volt=0,
        mot_cur=0,
        mot_ang=0,
        ank_ang=0,
        mot_vel=0,
        mot_acc=0,
        ank_vel=0,
        temperature=0,
        genvar_0=0,
        genvar_1=0,
        genvar_2=0,
        genvar_3=0,
        genvar_4=0,
        genvar_5=0,
        accelx=0,
        accely=0,
        accelz=0,
        gyrox=0,
        gyroy=0,
        gyroz=0,
    ):
        self.batt_volt = batt_volt
        self.batt_curr = batt_curr
        self.mot_volt = mot_volt
        self.mot_cur = mot_cur
        self.mot_ang = mot_ang
        self.ank_ang = ank_ang
        self.mot_vel = mot_vel
        self.mot_acc = mot_acc
        self.ank_vel = ank_vel
        self.temperature = temperature
        self.genvar_0 = genvar_0
        self.genvar_1 = genvar_1
        self.genvar_2 = genvar_2
        self.genvar_3 = genvar_3
        self.genvar_4 = genvar_4
        self.genvar_5 = genvar_5
        self.accelx = accelx
        self.accely = accely
        self.accelz = accelz
        self.gyrox = gyrox
        self.gyroy = gyroy
        self.gyroz = gyroz
        self.status_ex = 0b00000000


@pytest.fixture
def dephyactpack_mock() -> MockDephyActpack:
    """
    Fixture that returns a MockDephyActpack object
    """

    return MockDephyActpack()


@pytest.fixture
def patch_dephyactpack(mocker, dephyactpack_mock: MockDephyActpack):
    """
    Fixture that patches the DephyActpack class with the newly made MockDephyActpack class
    """

    mocker.patch(
        "opensourceleg.hardware.actuators.DephyActpack.__new__",
        return_value=dephyactpack_mock,
    )


@pytest.fixture
def dephyactpack_patched(patch_dephyactpack) -> DephyActpack:
    """
    Fixture that returns a MockDephyActpack object when the DephyActpack class is called
    """

    obj = DephyActpack()
    return obj


def test_patching(dephyactpack_patched: DephyActpack):
    """
    Initializes a DephyActpack object using the dephyactpack_patched fixture
    and asserts that it is an instance of the MockDephyActpack class.
    """

    patched_dap = dephyactpack_patched
    assert isinstance(patched_dap, MockDephyActpack)


def test_mockdephyactpack_open(dephyactpack_mock: MockDephyActpack):
    """
    Tests the open method of the MockDephyActpack class.\n
    This test initializes a MockDephyActpack object, creates a logger of
    the lowest stream level, and calls the open method without and with
    erroring arguments. It then asserts that the proper log messages were
    written.
    """

    mocked_dap = dephyactpack_mock
    mocked_dap._log = Logger(
        file_path="tests/test_actuators/test_mockdephyactpack_open_log"
    )
    mocked_dap._log.set_stream_level("DEBUG")
    # Tests the open method with non-erroring arguments
    mocked_dap.open(freq=10, log_level=4, log_enabled=False)
    with open("tests/test_actuators/test_mockdephyactpack_open_log.log") as f:
        contents = f.read()
        assert "DEBUG: Opening Device at /dev/ttyACM0" in contents
    # Tests the open method with erroring arguments
    with pytest.raises(OSError):
        mocked_dap.open(freq=100, log_level=5, log_enabled=True)


def test_properties_zero(dephyactpack_patched: DephyActpack):
    """
    Tests the default properties of the MockDephyActpack class.\n
    This test initializes a MockDephyActpack object and asserts that the
    properties are set to their default values.
    """

    mock_dap = dephyactpack_patched

    assert mock_dap.frequency == 500
    assert mock_dap.mode == VoltageMode(device=mock_dap)
    assert mock_dap.control_modes == ActpackControlModes(device=mock_dap)
    assert mock_dap.motor_zero_position == 0
    assert mock_dap.joint_zero_position == 0
    assert mock_dap.battery_voltage == 0
    assert mock_dap.battery_current == 0
    assert mock_dap.motor_voltage == 0
    assert mock_dap.motor_current == 0
    assert mock_dap.motor_torque == 0
    assert mock_dap.motor_position == 0
    assert mock_dap.motor_velocity == 0
    assert mock_dap.motor_acceleration == 0
    assert mock_dap.joint_position == 0
    assert mock_dap.joint_velocity == 0
    assert mock_dap.case_temperature == 0
    assert mock_dap.winding_temperature == 0
    assert mock_dap.genvars.shape == (6,)
    assert np.all(mock_dap.genvars == 0)
    assert mock_dap.accelx == 0
    assert mock_dap.accely == 0
    assert mock_dap.accelz == 0
    assert mock_dap.gyrox == 0
    assert mock_dap.gyroy == 0
    assert mock_dap.gyroz == 0


def test_properties_nonzero(dephyactpack_patched: DephyActpack):
    """
    Tests the properties of the MockDephyActpack class when the data
    attribute has non-zero values.\n This test initializes a MockDephyActpack
    object, sets the data attribute to non-zero values, and asserts that
    the properties are calulated correctly and set to the proper values.
    """

    mock_dap1 = dephyactpack_patched

    mock_dap1._data = Data(
        batt_volt=10,
        batt_curr=20,
        mot_volt=10,
        mot_cur=20,
        mot_ang=10,
        ank_ang=20,
        mot_vel=10,
        mot_acc=20,
        ank_vel=10,
        temperature=20,
        genvar_0=10,
        genvar_1=20,
        genvar_2=10,
        genvar_3=20,
        genvar_4=10,
        genvar_5=20,
        accelx=10,
        accely=20,
        accelz=10,
        gyrox=20,
        gyroy=10,
        gyroz=20,
    )
    assert mock_dap1.mode == VoltageMode(device=mock_dap1)
    assert mock_dap1.control_modes == ActpackControlModes(device=mock_dap1)
    assert mock_dap1.battery_voltage == 10
    assert mock_dap1.battery_current == 20
    assert mock_dap1.motor_voltage == 10
    assert mock_dap1.motor_current == 20
    assert mock_dap1.motor_torque == 20 * 0.1133 / 1000
    assert mock_dap1.motor_position == 10 * 2 * np.pi / 16384
    assert mock_dap1.motor_encoder_counts == 10
    assert mock_dap1.joint_encoder_counts == 20
    assert mock_dap1.motor_velocity == 10 * np.pi / 180
    assert mock_dap1.motor_acceleration == 20
    assert mock_dap1.joint_position == 20 * 2 * np.pi / 16384
    assert mock_dap1.joint_velocity == 10 * np.pi / 180
    assert mock_dap1.case_temperature == 20
    assert mock_dap1.winding_temperature == 21
    assert mock_dap1.genvars.shape == (6,)
    assert np.all(mock_dap1.genvars == np.array([10, 20, 10, 20, 10, 20]))
    assert mock_dap1.accelx == 10 * 9.80665 / 8192
    assert mock_dap1.accely == 20 * 9.80665 / 8192
    assert mock_dap1.accelz == 10 * 9.80665 / 8192
    assert mock_dap1.gyrox == 20.0 * float(np.pi / 180 / 32.8)
    assert mock_dap1.gyroy == 10 * float(np.pi / 180 / 32.8)
    assert mock_dap1.gyroz == 20 * float(np.pi / 180 / 32.8)


def test_mode_prop(dephyactpack_patched: DephyActpack):
    """
    Tests the mode property of the DephyActpack class where the device
    is an instance of the MockDephyActpack class.\n This test initializes
    a MockDephyActpack object and asserts that the mode property returns
    the proper ActpackMode object.
    """

    mock_dap1 = dephyactpack_patched
    mock_dap1._mode = VoltageMode(device=mock_dap1)
    assert mock_dap1.mode == VoltageMode(device=mock_dap1)
    mock_dap1._mode = PositionMode(device=mock_dap1)
    assert mock_dap1.mode == PositionMode(device=mock_dap1)
    mock_dap1._mode = CurrentMode(device=mock_dap1)
    assert mock_dap1.mode == CurrentMode(device=mock_dap1)
    mock_dap1._mode = ImpedanceMode(device=mock_dap1)
    assert mock_dap1.mode == ImpedanceMode(device=mock_dap1)


def test_set_motor_zero_position(dephyactpack_patched: DephyActpack):
    """
    Tests the set_motor_zero_position method of the DephyActpack class.\n
    This test initializes a MockDephyActpack object and asserts that the
    _motor_zero_position attribute is set to the proper value when initialized
    and when updated.
    """

    mock_dap2 = dephyactpack_patched
    mock_dap2.set_motor_zero_position(10)
    assert mock_dap2._motor_zero_position == 10
    mock_dap2.set_motor_zero_position(-20)
    assert mock_dap2._motor_zero_position == -20


def test_set_joint_zero_position(dephyactpack_patched: DephyActpack):
    """
    Tests the set_joint_zero_position method of the DephyActpack class.\n
    This test initializes a MockDephyActpack object and asserts that the
    _joint_zero_position attribute is set to the proper value when initialized
    and when updated.
    """

    mock_dap3 = dephyactpack_patched
    mock_dap3.set_joint_zero_position(10)
    assert mock_dap3._joint_zero_position == 10
    mock_dap3.set_joint_zero_position(-20)
    assert mock_dap3._joint_zero_position == -20


def test_voltagemode(dephyactpack_patched: DephyActpack):
    """
    Tests the VoltageMode class.\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, and tests the entry and exit
    methods of the VoltageMode class. It then asserts that the proper log
    messages were written and that the proper motor command was sent.
    """

    mock_dap4 = dephyactpack_patched
    mock_dap4._log = Logger(file_path="tests/test_actuators/test_voltagemode_log")
    mock_dap4._log.set_stream_level("DEBUG")
    mock_dap4._mode = VoltageMode(mock_dap4)
    # Tests the entry method of the VoltageMode class
    mock_dap4._mode._entry()
    # Asserts the proper log message is written
    with open("tests/test_actuators/test_voltagemode_log.log") as f:
        contents = f.read()
        assert "DEBUG: [Actpack] Entering Voltage mode." in contents
    # Tests the exit method of the VoltageMode class
    mock_dap4._mode._exit()
    # Asserts the proper log message is written
    with open("tests/test_actuators/test_voltagemode_log.log") as f:
        contents = f.read()
        assert "DEBUG: [Actpack] Exiting Voltage mode." in contents
    # Asserts the proper motor command is sent
    assert mock_dap4._motor_command == "Control Mode: c_int(1), Value: 0"


def test_currentmode(dephyactpack_patched: DephyActpack):
    """
    Tests the CurrentMode class.\n
    This test initializes a MockDephyActpack object, sets the _log
    attribute to a Logger of the lowest stream level, and tests the
    entry and exit methods of the CurrentMode class. It then asserts
    that the proper log messages were written, that the proper gains
    were set, and that the proper motor command was sent.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap5 = dephyactpack_patched
    mock_dap5._log = Logger(file_path="tests/test_actuators/test_currentmode_log")
    mock_dap5._log.set_stream_level("DEBUG")
    mock_dap5._mode = CurrentMode(mock_dap5)
    # Tests the entry method of the VoltageMode class
    mock_dap5._mode._entry()
    # Asserts the proper log message is written
    with open("tests/test_actuators/test_currentmode_log.log") as f:
        contents = f.read()
        assert "DEBUG: [Actpack] Entering Current mode." in contents
    # Asserts the set_gains method works properly
    assert mock_dap5._gains == {"kp": 40, "ki": 400, "kd": 0, "k": 0, "b": 0, "ff": 128}
    assert mock_dap5._mode._has_gains == True
    # Asserts the proper motor command is sent
    assert mock_dap5._motor_command == "Control Mode: c_int(2), Value: 0"
    # Tests the exit method of the VoltageMode class
    mock_dap5._mode._exit()
    # Asserts the proper log message is written
    with open("tests/test_actuators/test_currentmode_log.log") as f:
        contents = f.read()
        assert "DEBUG: [Actpack] Exiting Current mode." in contents
    # Asserts the proper motor command is sent
    assert mock_dap5._motor_command == "Control Mode: c_int(1), Value: 0"


def test_positionmode(dephyactpack_patched: DephyActpack):
    """
    Tests the PositionMode class.\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, and tests the entry and exit
    methods of the PositionMode class. It then asserts that the proper log
    messages were written, that the proper gains were set, and that the
    proper motor command was sent.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap6 = dephyactpack_patched
    mock_dap6._data = Data(
        mot_ang=10,
    )
    mock_dap6._log = Logger(file_path="tests/test_actuators/test_positionmode_log")
    mock_dap6._log.set_stream_level("DEBUG")
    mock_dap6._mode = PositionMode(mock_dap6)
    # Tests the entry method of the PositionMode class
    mock_dap6._mode._entry()
    # Asserts the proper log message is written
    with open("tests/test_actuators/test_positionmode_log.log") as f:
        contents = f.read()
        assert "DEBUG: [Actpack] Entering Position mode." in contents
    # Asserts the set_gains method works properly
    assert mock_dap6._gains == {"kp": 50, "ki": 0, "kd": 0, "k": 0, "b": 0, "ff": 0}
    assert mock_dap6._mode._has_gains == True
    # Asserts the proper motor command is sent
    assert mock_dap6._motor_command == "Control Mode: c_int(0), Value: 10"
    # Tests the exit method of the PositionMode class
    mock_dap6._mode._exit()
    # Asserts the proper log message is written
    with open("tests/test_actuators/test_positionmode_log.log") as f:
        contents = f.read()
        assert "DEBUG: [Actpack] Exiting Position mode." in contents
    # Asserts the proper motor command is sent
    assert mock_dap6._motor_command == "Control Mode: c_int(1), Value: 0"


def test_impedancemode(dephyactpack_patched: DephyActpack):
    """
    Tests the ImpedanceMode class.\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, and tests the entry and exit
    methods of the ImpedanceMode class. It then asserts that the proper log
    messages were written, that the proper gains were set, and that the
    proper motor command was sent.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap7 = dephyactpack_patched
    mock_dap7._data = Data(
        mot_ang=10,
    )
    mock_dap7._log = Logger(file_path="tests/test_actuators/test_impedancemode_log")
    mock_dap7._log.set_stream_level("DEBUG")
    mock_dap7._mode = ImpedanceMode(mock_dap7)
    # Tests the entry method of the ImpedanceMode class
    mock_dap7._mode._entry()
    # Asserts the proper log message is written
    with open("tests/test_actuators/test_impedancemode_log.log") as f:
        contents = f.read()
        assert "DEBUG: [Actpack] Entering Impedance mode." in contents
    # Asserts the set_gains method works properly
    assert mock_dap7._gains == {
        "kp": 40,
        "ki": 400,
        "kd": 0,
        "k": 200,
        "b": 400,
        "ff": 128,
    }
    assert mock_dap7._mode._has_gains == True
    # Asserts the proper motor command is sent
    assert mock_dap7._motor_command == "Control Mode: c_int(3), Value: 10"
    # Tests the exit method of the ImpedanceMode class
    mock_dap7._mode._exit()
    # Asserts the proper log message is written
    with open("tests/test_actuators/test_positionmode_log.log") as f:
        contents = f.read()
        assert "DEBUG: [Actpack] Exiting Position mode." in contents
    # Asserts the proper motor command is sent
    assert mock_dap7._motor_command == "Control Mode: c_int(1), Value: 0"


def test_dephyactpack_start(dephyactpack_patched: DephyActpack):
    """
    Tests the DephyActpack start method\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, sets the _data attribute to a
    Data object with non-zero values, and calls the start method. It then
    asserts that the proper log messages were written and that the _data
    attribute was updated properly.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap8 = dephyactpack_patched
    mock_dap8._log = Logger(
        file_path="tests/test_actuators/test_dephyactpack_start8_log"
    )
    mock_dap8._log.set_stream_level("DEBUG")
    # Defines the _data attribute of the MockDephyActpack instance
    mock_dap8._data = Data(
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
    # Calls the start method of the MockDephyActpack instance
    mock_dap8.start()
    # Asserts the proper log messages are written
    with open("tests/test_actuators/test_dephyactpack_start8_log.log") as f:
        contents = f.read()
        assert "DEBUG: Opening Device at /dev/ttyACM0" in contents
        assert "DEBUG: [Actpack] Entering Voltage mode." in contents
    # Asserts the read method updated the _data attribute properly
    assert mock_dap8._data.batt_volt == 25
    assert mock_dap8._data.batt_curr == 25
    assert mock_dap8._data.mot_volt == 25
    assert mock_dap8._data.mot_cur == 25
    assert mock_dap8._data.mot_ang == 25
    assert mock_dap8._data.ank_ang == 25
    assert mock_dap8._data.mot_vel == 25
    assert mock_dap8._data.mot_acc == 25
    assert mock_dap8._data.ank_vel == 25
    assert mock_dap8._data.temperature == 25
    assert mock_dap8._data.genvar_0 == 25
    assert mock_dap8._data.genvar_1 == 25
    assert mock_dap8._data.genvar_2 == 25
    assert mock_dap8._data.genvar_3 == 25
    assert mock_dap8._data.genvar_4 == 25
    assert mock_dap8._data.genvar_5 == 25
    assert mock_dap8._data.accelx == 25
    assert mock_dap8._data.accely == 25
    assert mock_dap8._data.accelz == 25
    assert mock_dap8._data.gyrox == 25
    assert mock_dap8._data.gyroy == 25
    assert mock_dap8._data.gyroz == 25


def test_dephyactpack_stop(dephyactpack_patched: DephyActpack):
    """
    Tests the DephyActpack stop method\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, sets the mode to Current Mode,
    and calls the stop method. It then asserts that the mode was set to
    Voltage Mode and that the proper motor command was sent.
    """
    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap10 = dephyactpack_patched
    mock_dap10._log = Logger(
        file_path="tests/test_actuators/test_dephyactpack_stop_log"
    )
    mock_dap10._log.set_stream_level("DEBUG")
    mock_dap10._mode = CurrentMode(device=mock_dap10)
    # Calls the stop method of the MockDephyActpack instance
    mock_dap10.stop()
    # Asserts the set_mode method was called properly
    assert mock_dap10._mode == VoltageMode(device=mock_dap10)
    # Asserts the motor command was sent properly
    assert mock_dap10._motor_command == "Control Mode: c_int(1), Value: 0"


def test_dephyactpack_update(dephyactpack_patched: DephyActpack):
    """
    Tests the DephyActpack update method\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, sets the _data attribute to a
    Data object with some non-zero values, and calls the update method. It then
    asserts that the proper log messages were written and that the _data
    attribute was updated properly.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap11 = dephyactpack_patched
    mock_dap11._log = Logger(
        file_path="tests/test_actuators/test_dephyactpack_update_log"
    )
    mock_dap11._log.set_stream_level("DEBUG")
    # Sets the data values needed for testing the update method
    mock_dap11._data = Data(mot_cur=13, temperature=12)
    # Calls the update method of the MockDephyActpack instance
    mock_dap11.update()
    # Asserts the proper log messages are written for an unopened device
    with open("tests/test_actuators/test_dephyactpack_update_log.log") as f:
        contents = f.read()
        assert (
            "WARNING: [MockDephyActpack[DephyActpack]] Please open() the device before streaming data."
            in contents
        )
    # Set the is_streaming attribute to True to simulate an open device
    mock_dap11.is_streaming = True
    # Assert the default thermal model was properly initialized
    assert mock_dap11._thermal_model.T_w == 21
    # Calls the update method of the MockDephyActpack instance
    mock_dap11.update()
    # Asserts the update method properly updated the _data attribute
    assert mock_dap11._data.batt_volt == 15
    assert mock_dap11._data.batt_curr == 15
    assert mock_dap11._data.mot_volt == 15
    assert mock_dap11._data.mot_cur == 28
    assert mock_dap11._data.mot_ang == 15
    assert mock_dap11._data.ank_ang == 15
    assert mock_dap11._data.mot_vel == 15
    assert mock_dap11._data.mot_acc == 15
    assert mock_dap11._data.ank_vel == 15
    assert mock_dap11._data.temperature == 27
    assert mock_dap11._data.genvar_0 == 15
    assert mock_dap11._data.genvar_1 == 15
    assert mock_dap11._data.genvar_2 == 15
    assert mock_dap11._data.genvar_3 == 15
    assert mock_dap11._data.genvar_4 == 15
    assert mock_dap11._data.genvar_5 == 15
    assert mock_dap11._data.accelx == 15
    assert mock_dap11._data.accely == 15
    assert mock_dap11._data.accelz == 15
    assert mock_dap11._data.gyrox == 15
    assert mock_dap11._data.gyroy == 15
    assert mock_dap11._data.gyroz == 15
    assert (
        mock_dap11._thermal_model.T_w
        == (
            (
                (((28 * 1e-3) ** 2) * 0.376 * (1 + 0.393 / 100 * (21 - 65)))
                + (27 - 21) / 1.0702867186480716
            )
            / (0.20 * 81.46202695970649)
        )
        / 500
        + 21
    )
    assert (
        mock_dap11._thermal_model.T_c
        == ((21 - 27) / 1.0702867186480716 + (21 - 27) / 1.9406620046327363)
        / 512.249065845453
        / 500
        + 27
    )


def test_dephyactpack_set_mode(dephyactpack_patched: DephyActpack):
    """
    Tests the DephyActpack set_mode method\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, changes the mode to Current Mode,
    and calls the set_mode method with valid modes and invalid modes. It then
    asserts that the mode was set properly for valid modes and that the proper
    log messages were written for an invalid mode.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap12 = dephyactpack_patched
    mock_dap12._log = Logger(
        file_path="tests/test_actuators/test_dephyactpack_set_mode_log"
    )
    mock_dap12._log.set_stream_level("DEBUG")
    mock_dap12._mode = CurrentMode(device=mock_dap12)
    mock_dap12.set_mode(mock_dap12.control_modes.voltage)
    # Asserts the mode was properly set
    assert mock_dap12._mode == VoltageMode(device=mock_dap12)
    mock_dap12.set_mode(mock_dap12.control_modes.current)
    # Asserts the mode was properly set
    assert mock_dap12._mode == CurrentMode(device=mock_dap12)
    # Calls the set_mode method with invalid mode
    mock_dap12.set_mode("badmode")
    # Asserts the proper log messages are written for an invalid mode
    with open("tests/test_actuators/test_dephyactpack_set_mode_log.log") as f:
        contents = f.read()
        assert (
            "WARNING: [MockDephyActpack[DephyActpack]] Mode badmode not found"
            in contents
        )


def test_dephyactpack_set_position_gains(dephyactpack_patched: DephyActpack):
    """
    Tests the DephyActpack set_position_gains method\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, changes the mode to Position Mode,
    and calls the set_position_gains method with no arguments and with arguments.
    It then asserts that the gains were set properly and that the proper log
    messages were written for an invalid mode.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap13 = dephyactpack_patched
    mock_dap13._log = Logger(
        file_path="tests/test_actuators/test_dephyactpack_set_position_gains_log"
    )
    mock_dap13._log.set_stream_level("DEBUG")
    mock_dap13._mode = PositionMode(device=mock_dap13)
    # Calls the set_position_gains method with no arguments in PositionMode
    mock_dap13.set_position_gains()
    # Asserts the default gains were properly set
    assert mock_dap13._gains == {
        "kp": 50,
        "ki": 0,
        "kd": 0,
        "k": 0,
        "b": 0,
        "ff": 0,
    }
    # Calls the set_position_gains method with arguments
    mock_dap13.set_position_gains(kp=1, ki=2, kd=3)
    # Asserts the gains were properly set
    assert mock_dap13._gains == {
        "kp": 1,
        "ki": 2,
        "kd": 3,
        "k": 0,
        "b": 0,
        "ff": 0,
    }
    # Changed mode to CurrentMode
    mock_dap13._mode = CurrentMode(device=mock_dap13)
    # Calls the set_position_gains method with no arguments in CurrentMode
    mock_dap13.set_position_gains()
    # Asserts the proper log messages are written for an invalid mode
    with open("tests/test_actuators/test_dephyactpack_set_position_gains_log.log") as f:
        contents = f.read()
        assert (
            "WARNING: [MockDephyActpack[DephyActpack]] Cannot set position gains in mode c_int(2)"
            in contents
        )


def test_dephyactpack_set_current_gains(dephyactpack_patched: DephyActpack):
    """
    Tests the DephyActpack set_current_gains method\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, changes the mode to Current Mode,
    and calls the set_current_gains method with no arguments and with arguments.
    It then asserts that the gains were set properly and that the proper log
    messages were written for an invalid mode.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap14 = dephyactpack_patched
    mock_dap14._log = Logger(
        file_path="tests/test_actuators/test_dephyactpack_set_current_gains_log"
    )
    mock_dap14._log.set_stream_level("DEBUG")
    mock_dap14._mode = CurrentMode(device=mock_dap14)
    # Calls the set_current_gains method with no arguments in CurrentMode
    mock_dap14.set_current_gains()
    # Asserts the default gains were properly set
    assert mock_dap14._gains == {
        "kp": 40,
        "ki": 400,
        "kd": 0,
        "k": 0,
        "b": 0,
        "ff": 128,
    }
    # Calls the set_current_gains method with arguments
    mock_dap14.set_current_gains(kp=1, ki=2, ff=3)
    # Asserts the gains were properly set
    assert mock_dap14._gains == {
        "kp": 1,
        "ki": 2,
        "kd": 0,
        "k": 0,
        "b": 0,
        "ff": 3,
    }
    # Changed mode to PositionMode
    mock_dap14._mode = PositionMode(device=mock_dap14)
    # Calls the set_current_gains method with no arguments in PositionMode
    mock_dap14.set_current_gains()
    # Asserts the proper log messages are written for an invalid mode
    with open("tests/test_actuators/test_dephyactpack_set_current_gains_log.log") as f:
        contents = f.read()
        assert (
            "WARNING: [MockDephyActpack[DephyActpack]] Cannot set current gains in mode c_int(0)"
            in contents
        )


def test_dephyactpack_set_impedance_gains(dephyactpack_patched: DephyActpack):
    """
    Tests the DephyActpack set_impedance_gains method\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, changes the mode to Impedance Mode,
    and calls the set_impedance_gains method with no arguments and with arguments.
    It then asserts that the gains were set properly and that the proper log
    messages were written for an invalid mode.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap15 = dephyactpack_patched
    mock_dap15._log = Logger(
        file_path="tests/test_actuators/test_dephyactpack_set_impedance_gains_log"
    )
    mock_dap15._log.set_stream_level("DEBUG")
    mock_dap15._mode = ImpedanceMode(device=mock_dap15)
    # Calls the set_impedance_gains method with no arguments in ImpedanceMode
    mock_dap15.set_impedance_gains()
    # Asserts the default gains were properly set
    assert mock_dap15._gains == {
        "kp": 40,
        "ki": 400,
        "kd": 0,
        "k": 200,
        "b": 400,
        "ff": 128,
    }
    # Calls the set_impedance_gains method with arguments
    mock_dap15.set_impedance_gains(kp=1, ki=2, K=3, B=4, ff=5)
    # Asserts the gains were properly set
    assert mock_dap15._gains == {
        "kp": 1,
        "ki": 2,
        "kd": 0,
        "k": 3,
        "b": 4,
        "ff": 5,
    }
    # Changed mode to CurrentMode
    mock_dap15._mode = CurrentMode(device=mock_dap15)
    # Calls the set_impedance_gains method with no arguments in CurrentMode
    mock_dap15.set_impedance_gains()
    # Asserts the proper log messages are written for an invalid mode
    with open(
        "tests/test_actuators/test_dephyactpack_set_impedance_gains_log.log"
    ) as f:
        contents = f.read()
        assert (
            "WARNING: [MockDephyActpack[DephyActpack]] Cannot set impedance gains in mode c_int(2)"
            in contents
        )


def test_dephyactpack_set_voltage(dephyactpack_patched: DephyActpack):
    """
    Tests the DephyActpack set_voltage method\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, changes the mode to Voltage Mode,
    and calls the set_voltage method. It then asserts that the proper motor
    command was sent and that the proper log messages were written for an
    invalid mode.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap16 = dephyactpack_patched
    mock_dap16._log = Logger(
        file_path="tests/test_actuators/test_dephyactpack_set_voltage_log"
    )
    mock_dap16._log.set_stream_level("DEBUG")
    mock_dap16._mode = VoltageMode(device=mock_dap16)
    # Calls the set_voltage method in VoltageMode
    mock_dap16.set_voltage(value=4.0)
    # Asserts the proper motor command was sent
    assert mock_dap16._motor_command == "Control Mode: c_int(1), Value: 4"
    # Changed mode to PositionMode
    mock_dap16._mode = PositionMode(device=mock_dap16)
    # Calls the set_voltage method in PositionMode
    mock_dap16.set_voltage(value=4.0)
    # Asserts the proper log messages are written for an invalid mode
    with open("tests/test_actuators/test_dephyactpack_set_voltage_log.log") as f:
        contents = f.read()
        assert (
            "WARNING: [MockDephyActpack[DephyActpack]] Cannot set voltage in mode c_int(0)"
            in contents
        )


def test_dephyactpack_set_current(dephyactpack_patched: DephyActpack):
    """
    Tests the DephyActpack set_current method\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, changes the mode to Current Mode,
    and calls the set_current method. It then asserts that the proper motor
    command was sent and that the proper log messages were written for an
    invalid mode.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap17 = dephyactpack_patched
    mock_dap17._log = Logger(
        file_path="tests/test_actuators/test_dephyactpack_set_current_log"
    )
    mock_dap17._log.set_stream_level("DEBUG")
    mock_dap17._mode = CurrentMode(device=mock_dap17)
    # Calls the set_current method in CurrentMode
    mock_dap17.set_current(value=6.0)
    # Asserts the proper motor command was sent
    assert mock_dap17._motor_command == "Control Mode: c_int(2), Value: 6"
    # Changed mode to PositionMode
    mock_dap17._mode = PositionMode(device=mock_dap17)
    # Calls the set_current method in PositionMode
    mock_dap17.set_current(value=6.0)
    # Asserts the proper log messages are written for an invalid mode
    with open("tests/test_actuators/test_dephyactpack_set_current_log.log") as f:
        contents = f.read()
        assert (
            "WARNING: [MockDephyActpack[DephyActpack]] Cannot set current in mode c_int(0)"
            in contents
        )


def test_dephyactpack_set_motor_torque(dephyactpack_patched: DephyActpack):
    """
    Tests the DephyActpack set_motor_torque method\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, changes the mode to Current Mode,
    and calls the set_motor_torque method. It then asserts that the proper motor
    command was sent and that the proper log messages were written for an
    invalid mode.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap18 = dephyactpack_patched
    mock_dap18._log = Logger(
        file_path="tests/test_actuators/test_dephyactpack_set_motor_torque_log"
    )
    mock_dap18._log.set_stream_level("DEBUG")
    mock_dap18._mode = CurrentMode(device=mock_dap18)
    # Calls the set_motor_torque method in CurrentMode
    mock_dap18.set_motor_torque(torque=7.0)
    # Asserts the proper motor command was sent
    motor_torque = int(7 * 1000 / 0.1133)
    assert mock_dap18._motor_command == f"Control Mode: c_int(2), Value: {motor_torque}"
    # Changed mode to PositionMode
    mock_dap18._mode = PositionMode(device=mock_dap18)
    # Calls the set_motor_torque method in PositionMode
    mock_dap18.set_motor_torque(torque=7.0)
    # Asserts the proper log messages are written for an invalid mode
    with open("tests/test_actuators/test_dephyactpack_set_motor_torque_log.log") as f:
        contents = f.read()
        assert (
            "WARNING: [MockDephyActpack[DephyActpack]] Cannot set motor_torque in mode c_int(0)"
            in contents
        )


def test_dephyactpack_set_motor_position(dephyactpack_patched: DephyActpack):
    """
    Tests the DephyActpack set_motor_position method\n
    This test initializes a MockDephyActpack object, sets the _log attribute
    to a Logger of the lowest stream level, changes the mode to Position Mode,
    and calls the set_motor_position method. It then asserts that the proper motor
    command was sent and that the proper log messages were written for an
    invalid mode.
    """

    # Creates a MockDephyActpack instance with a Logger of the lowest stream level
    mock_dap19 = dephyactpack_patched
    mock_dap19._log = Logger(
        file_path="tests/test_actuators/test_dephyactpack_set_motor_position_log"
    )
    mock_dap19._log.set_stream_level("DEBUG")
    mock_dap19._mode = PositionMode(device=mock_dap19)
    # Calls the set_motor_position method in PositionMode
    mock_dap19.set_motor_position(position=8.0)
    # Asserts the proper motor command was sent
    motor_pos = int(8 / 2 / np.pi * 16384)
    assert mock_dap19._motor_command == f"Control Mode: c_int(0), Value: {motor_pos}"
    # Changed mode to ImpedanceMode
    mock_dap19._mode = ImpedanceMode(device=mock_dap19)
    # Calls the set_motor_position method in ImpedanceMode
    mock_dap19.set_motor_position(position=9.0)
    # Asserts the proper motor command was sent
    motor_pos2 = int(9 / 2 / np.pi * 16384)
    assert mock_dap19._motor_command == f"Control Mode: c_int(3), Value: {motor_pos2}"
    # Changed mode to CurrentMode
    mock_dap19._mode = CurrentMode(device=mock_dap19)
    # Calls the set_motor_position method in CurrentMode
    mock_dap19.set_motor_position(position=10.0)
    # Asserts the proper log messages are written for an invalid mode
    with open("tests/test_actuators/test_dephyactpack_set_motor_position_log.log") as f:
        contents = f.read()
        assert (
            "WARNING: [MockDephyActpack[DephyActpack]] Cannot set motor position in mode c_int(2)"
            in contents
        )
