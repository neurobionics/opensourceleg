from typing import Any

from unittest.mock import Mock, patch

import numpy as np
import pytest
from pytest_mock import mocker

from opensourceleg import constants
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

# class MockDephyActpack(DephyActpack):
#     def __init__(self, port, baud_rate):
#         self._device = MockDevice(port, baud_rate)


class MockDevice:
    def __init__(self, port, baud_rate):
        self._motor_zero_position = 0

    def read(self, size):
        return b""

    def write(self, data):
        pass

    def close(self):
        pass


class MockDephyActpack:
    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        logger: Logger = Logger(),
        units: UnitsDefinition = DEFAULT_UNITS,
        debug_level: int = 0,
        dephy_log: bool = False,
    ):
        self._device = MockDevice(port, baud_rate)
        self._voltage_mode = "SINGLE"
        self._current_limit = 0.0
        self._position = 0.0
        self._velocity = 0.0
        self._torque = 0.0
        self._gains = (0.0, 0.0, 0.0)
        self._units = DEFAULT_UNITS
        self._logger = None
        self._thermal_model = None
        self._units: UnitsDefinition = units if units else DEFAULT_UNITS
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

    def set_voltage_mode(self, mode):
        self._voltage_mode = mode

    def get_voltage_mode(self):
        return self._voltage_mode

    def set_current_limit(self, limit):
        self._current_limit = limit

    def get_current_limit(self):
        return self._current_limit

    def set_position(self, position):
        self._position = position

    def get_position(self):
        return self._position

    def set_velocity(self, velocity):
        self._velocity = velocity

    def get_velocity(self):
        return self._velocity

    def set_torque(self, torque):
        self._torque = torque

    def get_torque(self):
        return self._torque

    def set_gains(self, kp, ki, kd):
        self._gains = (kp, ki, kd)

    def get_gains(self):
        return self._gains

    def set_units(self, units):
        self._units = units

    def get_units(self):
        return self._units

    def set_logger(self, logger):
        self._logger = logger

    def get_logger(self):
        return self._logger

    def set_thermal_model(self, thermal_model):
        self._thermal_model = thermal_model

    def get_thermal_model(self):
        return self._thermal_model

    @property
    def units(self) -> UnitsDefinition:
        return self._units

    @property
    def frequency(self) -> int:
        return self._frequency

    @property
    def mode(self) -> ActpackMode:
        return self._mode

    @property
    def modes(self) -> dict[str, ActpackMode]:
        return self._modes

    @property
    def motor_zero_position(self) -> float:
        return self._motor_zero_position

    @property
    def joint_zero_position(self) -> float:
        return self._joint_zero_position

    @property
    def battery_voltage(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.batt_volt,
                attribute="voltage",
            )
        else:
            return 0.0

    @property
    def batter_current(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.batt_curr,
                attribute="current",
            )
        else:
            return 0.0

    @property
    def motor_voltage(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.mot_volt,
                attribute="voltage",
            )
        else:
            return 0.0

    @property
    def motor_current(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.mot_cur,
                attribute="current",
            )
        else:
            return 0.0

    @property
    def motor_torque(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.mot_cur * constants.NM_PER_MILLIAMP,
                attribute="torque",
            )
        else:
            return 0.0

    @property
    def motor_position(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=int(self._data.mot_ang - self.motor_zero_position)
                * constants.RAD_PER_COUNT,
                attribute="position",
            )
        else:
            return 0.0

    @property
    def motor_encoder_counts(self):
        return self._data.mot_ang

    @property
    def joint_encoder_counts(self):
        return self._data.ank_ang

    @property
    def motor_velocity(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=int(self._data.mot_vel) * constants.RAD_PER_COUNT,
                attribute="velocity",
            )
        else:
            return 0.0

    @property
    def motor_acceleration(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.mot_acc,
                attribute="acceleration",
            )
        else:
            return 0.0

    @property
    def joint_position(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=int(self._data.ank_ang - self._joint_zero_position)
                * constants.RAD_PER_COUNT,
                attribute="position",
            )
        else:
            return 0.0

    @property
    def joint_velocity(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.ank_vel * constants.RAD_PER_COUNT,
                attribute="velocity",
            )
        else:
            return 0.0

    @property
    def case_temperature(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.temperature,
                attribute="temperature",
            )
        else:
            return 0.0

    @property
    def winding_temperature(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._thermal_model.T_w,
                attribute="temperature",
            )
        else:
            return 0.0

    @property
    def genvars(self):
        if self._data is not None:
            return np.array(
                object=[
                    self._data.genvar_0,
                    self._data.genvar_1,
                    self._data.genvar_2,
                    self._data.genvar_3,
                    self._data.genvar_4,
                    self._data.genvar_5,
                ]
            )
        else:
            return np.zeros(shape=6)

    @property
    def accelx(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.accelx * constants.M_PER_SEC_SQUARED_ACCLSB,
                attribute="gravity",
            )
        else:
            return 0.0

    @property
    def accely(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.accely * constants.M_PER_SEC_SQUARED_ACCLSB,
                attribute="gravity",
            )
        else:
            return 0.0

    @property
    def accelz(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.accelz * constants.M_PER_SEC_SQUARED_ACCLSB,
                attribute="gravity",
            )
        else:
            return 0.0

    @property
    def gyrox(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.gyrox * constants.RAD_PER_SEC_GYROLSB,
                attribute="velocity",
            )
        else:
            return 0.0

    @property
    def gyroy(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.gyroy * constants.RAD_PER_SEC_GYROLSB,
                attribute="velocity",
            )
        else:
            return 0.0

    @property
    def gyroz(self) -> float:
        if self._data is not None:
            return self._units.convert_from_default_units(
                value=self._data.gyroz * constants.RAD_PER_SEC_GYROLSB,
                attribute="velocity",
            )
        else:
            return 0.0


def test_mockdevice():
    test_md = MockDevice(port=None, baud_rate=None)
    assert test_md._motor_zero_position == 0
    assert test_md.read(1) == b""
    assert test_md.write(b"") == None
    assert test_md.close() == None


def test_set_motor_zero_position(monkeypatch):
    monkeypatch.setattr("test_dephyactpack.DephyActpack", MockDephyActpack)
    mock_dap = DephyActpack()
    mock_dap.set_position(1)
    assert mock_dap.get_position() == 1


def test_properties_zero(monkeypatch):
    monkeypatch.setattr("test_dephyactpack.DephyActpack", MockDephyActpack)
    mock_dap = DephyActpack()
    assert mock_dap.units == DEFAULT_UNITS
    assert mock_dap.frequency == 500
    # assert mock_dap.mode == VoltageMode
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


# def test_properties_nonzero(monkeypatch):
#     monkeypatch.setattr("test_dephyactpack.DephyActpack", MockDephyActpack)
#     mock_dap1 = DephyActpack()
#     mock_dap1._data = {"batt_volt": 1}
#     assert mock_dap1.battery_voltage == 1
