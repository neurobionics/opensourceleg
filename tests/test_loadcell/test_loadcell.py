import numpy as np
import pytest
from pytest_mock import mocker

import opensourceleg.constants as constants
from opensourceleg.joints import Joint
from opensourceleg.loadcell import Loadcell, StrainAmp
from opensourceleg.logger import Logger
from tests.test_actuators.test_dephyactpack import Data
from tests.test_joints.test_joint import MockJoint


class MockSMBus:

    MEM_R_CH1_H = 8
    MEM_R_CH1_L = 9
    MEM_R_CH2_H = 10
    MEM_R_CH2_L = 11
    MEM_R_CH3_H = 12
    MEM_R_CH3_L = 13
    MEM_R_CH4_H = 14
    MEM_R_CH4_L = 15
    MEM_R_CH5_H = 16
    MEM_R_CH5_L = 17
    MEM_R_CH6_H = 18
    MEM_R_CH6_L = 19

    def __init__(self, bus: int = 1) -> None:
        self._bus = bus
        self._byte_data = bytearray(
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        )

    def read_byte_data(
        self, I2C_addr: int = 1, register: int = 0, force: bool = False
    ) -> int:
        return self._byte_data[register]

    def read_i2c_block_data(
        self,
        I2C_addr: int = 1,
        register: int = 0,
        length: int = 10,
        force: bool = False,
    ) -> list[int]:
        data = []
        for i in range(length):
            data.append(int(self._byte_data[i]))
        return data


class MockStrainAmp(StrainAmp):
    def __init__(self, bus: int = 1, I2C_addr=0x66) -> None:
        self._SMBus = MockSMBus(bus=bus)
        self.bus = bus
        self.addr = I2C_addr
        self.genvars = np.zeros((3, 6))
        self.indx = 0
        self.is_streaming = True
        self.data = []
        self.failed_reads = 0


class MockLoadcell(Loadcell):
    def __init__(
        self,
        dephy_mode: bool = False,
        joint: Joint = None,  # type: ignore
        amp_gain: float = 125.0,
        exc: float = 5.0,
        loadcell_matrix: np.ndarray = constants.LOADCELL_MATRIX,
        logger: "Logger" = None,  # type: ignore
        bus: int = 1,
    ) -> None:
        self._is_dephy: bool = dephy_mode
        self._joint: Joint = joint
        self._amp_gain: float = amp_gain
        self._exc: float = exc
        self._adc_range: int = 2**12 - 1
        self._offset: float = (2**12) / 2
        self._lc = None

        if not self._is_dephy:
            self._lc = strainamp_patched

        self._loadcell_matrix = loadcell_matrix
        self._loadcell_data = None
        self._prev_loadcell_data = None

        self._loadcell_zero = np.zeros(shape=(1, 6), dtype=np.double)
        self._zeroed = False
        self._log: Logger = logger


@pytest.fixture
def strainamp_mock() -> MockStrainAmp:
    return MockStrainAmp()


@pytest.fixture
def patch_strainamp(mocker, strainamp_mock: MockStrainAmp):
    mocker.patch(
        "opensourceleg.loadcell.StrainAmp.__new__", return_value=strainamp_mock
    )


@pytest.fixture
def strainamp_patched(patch_strainamp) -> StrainAmp:
    obj = StrainAmp(bus=1)
    return obj


@pytest.fixture
def loadcell_mock() -> MockLoadcell:
    return MockLoadcell()


@pytest.fixture
def patch_loadcell(mocker, loadcell_mock: MockLoadcell):
    mocker.patch("opensourceleg.loadcell.Loadcell.__new__", return_value=loadcell_mock)


@pytest.fixture
def loadcell_patched(patch_loadcell) -> Loadcell:
    obj = Loadcell()
    return obj


def test_mockstrainamp_init():
    test_mockstrainamp_default = MockStrainAmp(bus=1)
    assert test_mockstrainamp_default.MEM_R_CH1_H == 8
    assert test_mockstrainamp_default.MEM_R_CH1_L == 9
    assert test_mockstrainamp_default.MEM_R_CH2_H == 10
    assert test_mockstrainamp_default.MEM_R_CH2_L == 11
    assert test_mockstrainamp_default.MEM_R_CH3_H == 12
    assert test_mockstrainamp_default.MEM_R_CH3_L == 13
    assert test_mockstrainamp_default.MEM_R_CH4_H == 14
    assert test_mockstrainamp_default.MEM_R_CH4_L == 15
    assert test_mockstrainamp_default.MEM_R_CH5_H == 16
    assert test_mockstrainamp_default._SMBus._bus == 1
    assert test_mockstrainamp_default._SMBus._byte_data == bytearray(
        [0 for i in range(20)]
    )
    assert test_mockstrainamp_default.bus == 1
    assert test_mockstrainamp_default.addr == 0x66
    assert np.array_equal(test_mockstrainamp_default.genvars, np.zeros((3, 6)))
    assert test_mockstrainamp_default.indx == 0
    assert test_mockstrainamp_default.is_streaming == True
    assert test_mockstrainamp_default.data == []
    assert test_mockstrainamp_default.failed_reads == 0


def test_read_byte_data():
    smbus_mock = MockSMBus()
    assert smbus_mock.read_byte_data() == 0
    smbus_mock._byte_data[0] = 1
    smbus_mock._byte_data[1] = 2
    smbus_mock._byte_data[2] = 3
    smbus_mock._byte_data[3] = 4
    assert smbus_mock.read_byte_data(register=0) == 1
    assert smbus_mock.read_byte_data(register=1) == 2
    assert smbus_mock.read_byte_data(register=2) == 3
    assert smbus_mock.read_byte_data(register=3) == 4


def test_read_uncompressed_strain(strainamp_patched: StrainAmp):
    msa_byte_data = strainamp_patched
    msa_byte_data._SMBus._byte_data = bytearray(
        [
            0x01,
            0x02,
            0x03,
            0x04,
            0x05,
            0x06,
            0x07,
            0x08,
            0x09,
            0x0A,
            0x0B,
            0x0C,
            0x0D,
            0x0E,
            0x0F,
            0x10,
            0x11,
            0x12,
            0x13,
            0x14,
            0x15,
        ]
    )
    # data = [0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15]
    expected_data = [2314, 2828, 3342, 3856, 4370, 4884]
    uncompressed_strain = msa_byte_data.read_uncompressed_strain()
    assert np.array_equal(uncompressed_strain, expected_data)


def test_read_compressed_strain(strainamp_patched: StrainAmp):
    msa_rcs = strainamp_patched
    msa_rcs._SMBus._byte_data = bytearray(
        [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A]
    )
    expected_data = [16, 515, 64, 1286, 112, 2057]
    compressed_strain = msa_rcs.read_compressed_strain()
    assert np.array_equal(compressed_strain, expected_data)


def test_strainamp_update(strainamp_patched: StrainAmp):
    msa_update = strainamp_patched
    msa_update._SMBus._byte_data = bytearray(
        [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A]
    )
    actual_array = msa_update.update()
    new_indx = 1
    expected_array = np.array([0, 0, 0, 0, 0, 0])
    assert np.array_equal(actual_array, expected_array)
    assert msa_update.indx == new_indx
    actual_array2 = msa_update.update()
    new_indx2 = 2
    expected_array2 = np.array([16, 515, 64, 1286, 112, 2057])
    assert np.array_equal(actual_array2, expected_array2)
    assert msa_update.indx == new_indx2
    actual_array3 = msa_update.update()
    new_indx3 = 0
    expected_array3 = np.array([16, 515, 64, 1286, 112, 2057])
    assert np.array_equal(actual_array3, expected_array3)
    assert msa_update.indx == new_indx3


def test_strainamp_unpack_uncompressed_strain(strainamp_patched: StrainAmp):
    msa = strainamp_patched
    byte_data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C]
    unpacked_strain = msa.unpack_uncompressed_strain(byte_data)
    assert np.array_equal(unpacked_strain, [258, 772, 1286, 1800, 2314, 2828])


def test_strainamp_unpack_compressed_strain(strainamp_patched: StrainAmp):
    msa_ucs = strainamp_patched
    byte_data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09]
    unpacked_strain = msa_ucs.unpack_compressed_strain(byte_data)
    assert np.array_equal(unpacked_strain, [16, 515, 64, 1286, 112, 2057])


def test_strainamp_strain_data_to_wrench(strainamp_patched: StrainAmp):
    test_unpacked_strain = np.array([1, 2, 3, 4, 5, 6])
    test_loadcell_matrix = np.array(
        [
            (10.0, 20.0, 30.0, 40.0, 50.0, 60.0),
            (20.0, 30.0, 40.0, 50.0, 60.0, 70.0),
            (30.0, 40.0, 50.0, 60.0, 70.0, 80.0),
            (40.0, 50.0, 60.0, 70.0, 80.0, 90.0),
            (50.0, 60.0, 70.0, 80.0, 90.0, 100.0),
            (60.0, 70.0, 80.0, 90.0, 100.0, 110.0),
        ]
    )
    test_loadcell_zero = np.zeros(shape=(1, 6), dtype=np.double)
    # test_loadcell_signed = np.array([-2.4993895 , -2.4981685 , -2.4969475 , -2.4957265 , -2.49450549, -2.49328449])
    # test_loadcell_coupled = np.array([[-3.9990232,  -3.9970696,  -3.995116,   -3.99316239, -3.99120879, -3.98925519]])
    expected_result = np.array(
        [
            -838.42735043,
            -1078.07570208,
            -1317.72405372,
            -1557.37240537,
            -1797.02075702,
            -2036.66910867,
        ]
    )
    result = MockStrainAmp.strain_data_to_wrench(
        unpacked_strain=test_unpacked_strain,
        loadcell_matrix=test_loadcell_matrix,
        loadcell_zero=test_loadcell_zero,
    )
    assert round(result[0], 7) == round(expected_result[0], 7)
    assert round(result[1], 7) == round(expected_result[1], 7)
    assert round(result[2], 7) == round(expected_result[2], 7)
    assert round(result[3], 7) == round(expected_result[3], 7)
    assert round(result[4], 7) == round(expected_result[4], 7)
    assert round(result[5], 7) == round(expected_result[5], 7)


def test_strainamp_wrench_to_strain_data(strainamp_patched: StrainAmp):
    test_measurement = 5.0
    test_loadcell_matrix = np.array(
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
    expected_result = [
        [2048, 2048, 2047, 2110, 2013, 2048],
        [2047, 2048, 2048, 2048, 2049, 2016],
        [2048, 2048, 2047, 2048, 2118, 2048],
        [2048, 2047, 2048, 2046, 2048, 2016],
        [2048, 2048, 2047, 1988, 2013, 2048],
        [2049, 2048, 2048, 2047, 2047, 2017],
    ]
    result = MockStrainAmp.wrench_to_strain_data(
        measurement=test_measurement,
        loadcell_matrix=test_loadcell_matrix,
    )
    assert np.array_equal(result, expected_result)


def test_mockloadcell_init():
    test_loadcell_default = MockLoadcell(dephy_mode=True)
    assert test_loadcell_default._is_dephy == True
    assert test_loadcell_default._joint == None
    assert test_loadcell_default._amp_gain == 125.0
    assert test_loadcell_default._exc == 5.0
    assert test_loadcell_default._adc_range == 2**12 - 1
    assert test_loadcell_default._offset == (2**12) / 2
    assert test_loadcell_default._lc == None
    assert np.array_equal(
        test_loadcell_default._loadcell_matrix, constants.LOADCELL_MATRIX
    )
    assert test_loadcell_default._loadcell_data == None
    assert test_loadcell_default._prev_loadcell_data == None
    assert np.array_equal(
        test_loadcell_default._loadcell_zero, np.zeros(shape=(1, 6), dtype=np.double)
    )
    assert test_loadcell_default._zeroed == False
    assert test_loadcell_default._log == None


def test_loadcell_default_properties(loadcell_patched: Loadcell):
    loadcell_default = loadcell_patched
    assert loadcell_default.is_zeroed == False
    assert loadcell_default.fx == 0.0
    assert loadcell_default.fy == 0.0
    assert loadcell_default.fz == 0.0
    assert loadcell_default.mx == 0.0
    assert loadcell_default.my == 0.0
    assert loadcell_default.mz == 0.0
    assert loadcell_default.loadcell_data == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_loadcell_nondefault_properties(loadcell_patched: Loadcell):
    loadcell_nondefault = loadcell_patched
    loadcell_nondefault._zeroed = True
    loadcell_nondefault._loadcell_data = [
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        [7.0, 8.0, 9.0, 10.0, 11.0, 12.0],
        [13.0, 14.0, 15.0, 16.0, 17.0, 18.0],
        [19.0, 20.0, 21.0, 22.0, 23.0, 24.0],
        [25.0, 26.0, 27.0, 28.0, 29.0, 30.0],
        [31.0, 32.0, 33.0, 34.0, 35.0, 36.0],
    ]
    assert loadcell_nondefault.is_zeroed == True
    assert loadcell_nondefault.fx == 1.0
    assert loadcell_nondefault.fy == 2.0
    assert loadcell_nondefault.fz == 3.0
    assert loadcell_nondefault.mx == 4.0
    assert loadcell_nondefault.my == 5.0
    assert loadcell_nondefault.mz == 6.0
    assert loadcell_nondefault.loadcell_data == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_reset(loadcell_patched: Loadcell):
    loadcell_reset = loadcell_patched
    loadcell_reset._zeroed = True
    loadcell_reset._loadcell_zero = [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]]
    loadcell_reset.reset()
    assert loadcell_reset._zeroed == False
    assert np.array_equal(
        loadcell_reset._loadcell_zero, np.zeros(shape=(1, 6), dtype=np.double)
    )


def test_update(loadcell_patched: Loadcell):
    loadcell_update = loadcell_patched
    loadcell_update._is_dephy = True
    loadcell_update._joint = MockJoint()
    loadcell_update._joint._data = Data(
        genvar_0=1, genvar_1=2, genvar_2=3, genvar_3=4, genvar_4=5, genvar_5=6
    )
    loadcell_update.update()
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
    assert round(loadcell_update.fx, -2) == round(
        loadcell_signed_added_and_transposed[0][0], -2
    )
    assert round(loadcell_update.fy) == round(
        loadcell_signed_added_and_transposed[0][1]
    )
    assert round(loadcell_update.fz, -1) == round(
        loadcell_signed_added_and_transposed[0][2], -1
    )
    assert round(loadcell_update.mx) == round(
        loadcell_signed_added_and_transposed[0][3]
    )
    assert round(loadcell_update.my) == round(
        loadcell_signed_added_and_transposed[0][4]
    )
    assert round(loadcell_update.mz) == round(
        loadcell_signed_added_and_transposed[0][5]
    )

    loadcell_update_loadcell_zero = loadcell_patched
    loadcell_update_loadcell_zero._is_dephy = True
    loadcell_update_loadcell_zero._joint = MockJoint()
    loadcell_update_loadcell_zero._joint._data = Data(
        genvar_0=1, genvar_1=2, genvar_2=3, genvar_3=4, genvar_4=5, genvar_5=6
    )
    loadcell_update_loadcell_zero.update(
        loadcell_zero=np.array([[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]])
    )

    assert round(loadcell_update_loadcell_zero.fx, -2) == round(
        loadcell_signed_added_and_transposed[0][0] - 1.0, -2
    )
    assert round(loadcell_update_loadcell_zero.fy) == round(
        loadcell_signed_added_and_transposed[0][1] - 2.0
    )
    assert round(loadcell_update_loadcell_zero.fz, -1) == round(
        loadcell_signed_added_and_transposed[0][2] - 3.0, -1
    )
    assert round(loadcell_update_loadcell_zero.mx) == round(
        loadcell_signed_added_and_transposed[0][3] - 4.0
    )
    assert round(loadcell_update_loadcell_zero.my) == round(
        loadcell_signed_added_and_transposed[0][4] - 5.0
    )
    assert round(loadcell_update_loadcell_zero.mz) == round(
        loadcell_signed_added_and_transposed[0][5] - 6.0
    )
