from typing import Any

import numpy as np
import pytest
from pytest_mock import mocker

import opensourceleg.constants as constants
from opensourceleg.actuators import (
    ActpackMode,
    CurrentMode,
    DephyActpack,
    ImpedanceMode,
    PositionMode,
    VoltageMode,
)
from opensourceleg.logger import Logger
from opensourceleg.thermal import ThermalModel
from opensourceleg.units import DEFAULT_UNITS, UnitsDefinition


class MockDevice:

    v1: int = 0
    v2: int = 0

    def __init__(self, port=None, baud_rate=None):
        self.port = port
        self.baud_rate = baud_rate

    def read(self, size):
        return b""

    def write(self, data):
        pass

    def close(self):
        pass


class MockDephyActpack(DephyActpack):
    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        logger: Logger = Logger(),
        units: UnitsDefinition = DEFAULT_UNITS,
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:
        """
        Initializes the MockActpack class

        Args:
            port (str): _description_
            baud_rate (int): _description_. Defaults to 230400.
            frequency (int): _description_. Defaults to 500.
            logger (Logger): _description_
            units (UnitsDefinition): _description_
            debug_level (int): _description_. Defaults to 0.
            dephy_log (bool): _description_. Defaults to False.
        """
        self._debug_level: int = debug_level
        self._dephy_log: bool = dephy_log
        self._frequency: int = frequency
        self._data: Any = None

        self._log: Logger = logger
        self._state = None
        self._units: UnitsDefinition = units if units else DEFAULT_UNITS

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=80,
            soft_border_C_windings=10,
            temp_limit_case=70,
            soft_border_C_case=10,
        )

        self._modes: dict[str, ActpackMode] = {
            "voltage": VoltageMode(device=self),
            "position": PositionMode(device=self),
            "current": CurrentMode(device=self),
            "impedance": ImpedanceMode(device=self),
        }

        self._mode: ActpackMode = self._modes["voltage"]


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


@pytest.fixture
def device_mock() -> MockDevice:
    return MockDevice(port=None, baud_rate=None)


@pytest.fixture
def patch_device(mocker, device_mock: MockDevice):
    mocker.patch("flexsea.device.Device.__new__", return_value=device_mock)


@pytest.fixture
def dephyactpack_mock() -> MockDephyActpack:
    return MockDephyActpack()


@pytest.fixture
def patch_dephyactpack(mocker, dephyactpack_mock: MockDephyActpack):
    mocker.patch(
        "opensourceleg.actuators.DephyActpack.__new__", return_value=dephyactpack_mock
    )


@pytest.fixture
def dephyactpack_patched(patch_dephyactpack) -> MockDephyActpack:
    obj = DephyActpack()
    return obj


def test_patching(dephyactpack_patched: DephyActpack):
    patched_dap = dephyactpack_patched
    assert isinstance(patched_dap, MockDephyActpack)


def test_device_mock(device_mock: MockDevice):
    mocked_device = device_mock
    assert isinstance(mocked_device, MockDevice)


def test_properties_zero(dephyactpack_patched: DephyActpack):

    mock_dap = dephyactpack_patched

    assert mock_dap.units == DEFAULT_UNITS
    assert mock_dap.frequency == 500
    # assert mock_dap.mode == ActpackMode
    assert mock_dap.motor_zero_position == 0
    assert mock_dap.joint_zero_position == 0
    assert mock_dap.battery_voltage == 0
    assert mock_dap.batter_current == 0
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
    assert mock_dap1.battery_voltage == 10
    assert mock_dap1.batter_current == 20
    assert mock_dap1.motor_voltage == 10
    assert mock_dap1.motor_current == 20
    assert mock_dap1.motor_torque == 20 * 0.1133 / 1000
    assert mock_dap1.motor_encoder_counts == 10
    assert mock_dap1.joint_encoder_counts == 20
    assert mock_dap1.motor_velocity == 10 * 2 * np.pi / 16384
    assert mock_dap1.motor_acceleration == 20
    assert mock_dap1.joint_position == 20 * 2 * np.pi / 16384
    assert mock_dap1.joint_velocity == 10 * 2 * np.pi / 16384
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


def test_set_motor_zero_position(dephyactpack_patched: DephyActpack):
    mock_dap2 = dephyactpack_patched
    mock_dap2.set_motor_zero_position(10)
    assert mock_dap2._motor_zero_position == 10
    mock_dap2.set_motor_zero_position(-20)
    assert mock_dap2._motor_zero_position == -20


def test_set_joint_zero_position(dephyactpack_patched: DephyActpack):
    mock_dap3 = dephyactpack_patched
    mock_dap3.set_joint_zero_position(10)
    assert mock_dap3._joint_zero_position == 10
    mock_dap3.set_joint_zero_position(-20)
    assert mock_dap3._joint_zero_position == -20


# def test_set_position_gains(dephyactpack_patched: DephyActpack):
#     mock_dap4 = dephyactpack_patched
#     mock_dap4._mode = PositionMode(mock_dap4)
#     mock_dap4.set_position_gains(10, 20, 30)
#     # assert mock_dap4._impedance_gains == (10, 20, 30)
#     mock_dap4.set_position_gains()
#     # assert mock_dap4._position_gains == (50, 0, 0)

# def test_set_current_gains(dephyactpack_patched: DephyActpack):
#     mock_dap5 = dephyactpack_patched
#     mock_dap5._mode = CurrentMode(mock_dap5)
#     mock_dap5.set_current_gains(10, 20, 30)
#     # assert mock_dap5._impedance_gains == (10, 20, 30)
#     mock_dap5.set_current_gains()
#     # assert mock_dap5._current_gains == (40, 400, 128)

# def test_set_impedance_gains(dephyactpack_patched: DephyActpack):
#     mock_dap5 = dephyactpack_patched
#     mock_dap5._mode = ImpedanceMode(mock_dap5)
#     mock_dap5.set_impedance_gains(10, 20, 30, 40, 50)
#     # assert mock_dap5._impedance_gains == (10, 20, 30, 40, 50)
#     mock_dap5.set_impedance_gains()
#     # assert mock_dap5._impedance_gains == (40, 400, 200, 400, 128)
