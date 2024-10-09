# Global Units Dictionary
import enum
import time
from dataclasses import dataclass

import numpy as np
import pytest
from numpy import typing as npt
from smbus2 import SMBus

from opensourceleg.sensors import loadcell

VALUES = np.append(np.array([0, 1, -1, 1000, -1000]), np.random.random(5))
DEFAULT_CAL_MATRIX = np.ones(shape=(6, 6), dtype=np.double)

# TODO: Test loadcell not responding exception


def test_SRILoadcell_init():

    invalid_cal_matrix = np.ones(shape=(5, 6), dtype=np.double)
    with pytest.raises(TypeError):
        SRI = loadcell.SRILoadcell(calibration_matrix=invalid_cal_matrix)
    with pytest.raises(ValueError):
        SRI = loadcell.SRILoadcell(calibration_matrix=DEFAULT_CAL_MATRIX, amp_gain=0)
    with pytest.raises(ValueError):
        SRI = loadcell.SRILoadcell(calibration_matrix=DEFAULT_CAL_MATRIX, exc=0)

    SRI = loadcell.SRILoadcell(calibration_matrix=DEFAULT_CAL_MATRIX)

    assert SRI._amp_gain == 125.0
    assert SRI._exc == 5.0
    assert np.array_equal(SRI._calibration_matrix, DEFAULT_CAL_MATRIX)
    assert SRI._bus == 1
    assert SRI._i2c_address == 0x66

# TODO: Commenting out this test case as it needs to be reworked
# def test_SRILoadcell_start():

#     # Initialize SRILoadcell object
#     SRI = loadcell.SRILoadcell(calibration_matrix=DEFAULT_CAL_MATRIX)

#     start = time.time()
#     SRI.start()
#     end = time.time()

#     # assert SRI._bus == SMBus(SRI._bus) #?
#     assert (end - start) >= 1  # TODO: Link to var
#     assert SRI._is_streaming == True

#     # Test start with bus or i2c_address set to None
#     SRI = loadcell.SRILoadcell(DEFAULT_CAL_MATRIX, 125.0, 5, 1, None)
#     result = SRI.start()  # TODO: Am I doing this correctly?
#     assert result == None


def test_SRILoadcell_reset():

    SRI = loadcell.SRILoadcell(calibration_matrix=DEFAULT_CAL_MATRIX)
    SRI._calibration_offset == np.ones(shape=(1, 6), dtype=np.double)
    SRI.reset()
    assert np.array_equal(
        SRI._calibration_offset, np.zeros(shape=(1, 6), dtype=np.double)
    )


def test_SRILoadcell_update():

    # Test basic call execution
    SRI = loadcell.SRILoadcell(calibration_matrix=DEFAULT_CAL_MATRIX)
    SRI.update(data_callback=_read_data)

    # Ensuring self._calibration_offset was used
    data = _update_calculations(SRI, SRI._calibration_offset)
    assert np.array_equal(SRI._data, data)

    # Testing if passed calibration offset was used
    passed_calibration_offset = 0.1
    SRI.update(data_callback=_read_data, calibration_offset=passed_calibration_offset)

    # Ensuring passed calibration_offset was used
    data = _update_calculations(SRI, passed_calibration_offset)
    assert np.array_equal(SRI._data, data)


def test_SRILoadcell_calibrate():
    # Test reset, else statement

    return


# Function to bypass _read_compressed_strain() with an array of ones
def _read_data() -> npt.NDArray[np.uint8]:
    return np.ones(shape=(1, 6))


# Function to bypass _read_compressed_strain() with random data
def _read_random_data() -> npt.NDArray[np.uint8]:
    return np.random.randint(low=0, high=255, size=(1, 6), dtype=np.uint8)


# Function to run update calculations from inside the update method
def _update_calculations(SRI: loadcell.SRILoadcell, calibration_offset: float):
    test_data = _read_data()
    signed_data = ((test_data - SRI.OFFSET) / SRI.ADC_RANGE) * SRI._exc
    coupled_data = signed_data * 1000 / (SRI._exc * SRI._amp_gain)
    data = (
        np.transpose(a=SRI._calibration_matrix.dot(b=np.transpose(a=coupled_data)))
        - calibration_offset
    )
    return data


if __name__ == "__main__":
    test_SRILoadcell_update()
