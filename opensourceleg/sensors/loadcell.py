from typing import Any, Callable, Union

import time
from dataclasses import dataclass
from enum import Enum

import numpy as np
import numpy.typing as npt
from smbus2 import SMBus

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import LoadcellBase


class LoadcellNotRespondingException(Exception):
    def __init__(self, message: str = "Load cell unresponsive.") -> None:
        super().__init__(message)


class MEMORY_CHANNELS(Enum):
    CH1_H = 8
    CH1_L = 9
    CH2_H = 10
    CH2_L = 11
    CH3_H = 12
    CH3_L = 13
    CH4_H = 14
    CH4_L = 15
    CH5_H = 16
    CH5_L = 17
    CH6_H = 18
    CH6_L = 19


class SRILoadcell(LoadcellBase):
    ADC_RANGE = 2**12 - 1
    OFFSET = 2**12 / 2

    def __init__(
        self,
        calibration_matrix: npt.NDArray[np.double],
        amp_gain: float = 125.0,
        exc: float = 5.0,
        bus: int = 1,
        i2c_address: int = 0x66,
    ) -> None:
        '''
        TODO: Write docstring for initial values
        '''
        # Check that parameters are set correctly:
        if calibration_matrix.shape != (6, 6):
            LOGGER.info(f"[{self.__repr__()}] calibration_matrix must be a 6x6 array of np.double.")
            raise TypeError("calibration_matrix must be a 6x6 array of np.double.")
        if amp_gain <= 0:
            LOGGER.info(f"[{self.__repr__()}] amp_gain must be a floating point value greater than 0.")
            raise ValueError("amp_gain must be a floating point value greater than 0.")
        if exc <= 0 :
            LOGGER.info(f"[{self.__repr__()}] exc must be a floating point value greater than 0.")
            raise ValueError("exc must be a floating point value greater than 0.")

        self._amp_gain: float = amp_gain
        self._exc: float = exc

        self._bus = bus
        self._i2c_address = i2c_address

        self._calibration_matrix = calibration_matrix

        self._data: npt.NDArray[np.double] = np.zeros(shape=(1, 6), dtype=np.double)
        self._prev_data: npt.NDArray[np.double] = self._data
        self._failed_reads = 0

        self._calibration_offset: npt.NDArray[np.double] = np.zeros(
            shape=(1, 6), dtype=np.double
        )
        self._zero_calibration_offset: npt.NDArray[np.double] = self._calibration_offset
        self._is_calibrated: bool = False
        self._is_streaming: bool = False

    def start(self) -> None:
        # TODO: What is the purpose of this check?
        if (self._bus) or (self._i2c_address is None):
            return

        self._smbus = SMBus(self._bus)
        time.sleep(1)
        self._is_streaming = True

    def reset(self):
        self._calibration_offset = self._zero_calibration_offset
        self._is_calibrated = False

    def update(
        self,
        calibration_offset: npt.NDArray[np.double] = None,
        data_callback: Callable[..., npt.NDArray[np.uint8]] = None,
    ) -> None:
        """
        Queries the loadcell for the latest data.
        Latest data can then be accessed via properties, e.g. loadcell.Fx.
        """
        data = data_callback() if data_callback else self._read_compressed_strain()

        if calibration_offset is None:
            calibration_offset = self._calibration_offset

        signed_data = ((data - self.OFFSET) / self.ADC_RANGE) * self._exc
        coupled_data = signed_data * 1000 / (self._exc * self._amp_gain)

        self._data = (
            np.transpose(a=self._calibration_matrix.dot(b=np.transpose(a=coupled_data)))
            - calibration_offset
        )

    def calibrate(
        self,
        number_of_iterations: int = 2000,
        reset: bool = False,
        data_callback: Callable[[], npt.NDArray[np.uint8]] = None,
    ) -> None:
        """
        Obtains the initial loadcell reading (aka) loadcell_zero.
        This is automatically subtracted off for subsequent calls of the update method.
        """

        if not self.is_calibrated:
            LOGGER.info(
                f"[{self.__repr__()}] Initiating zeroing routine, please ensure that there is no ground contact force.\n{input('Press any key to start.')}"
            )

            self.update(data_callback=data_callback)
            self._calibration_offset = self._data

            for _ in range(number_of_iterations):
                self.update(
                    calibration_offset=self._zero_calibration_offset,
                    data_callback=data_callback,
                )
                iterative_calibration_offset = self._data
                self._calibration_offset = (
                    iterative_calibration_offset + self._calibration_offset
                ) / 2.0

            self._is_calibrated = True
            LOGGER.info(f"[{self.__repr__()}] Calibration routine complete.")

        elif reset:
            self.reset()
            self.calibrate()

        else:
            LOGGER.info(
                f"[{self.__repr__()}] Loadcell has already been zeroed. To recalibrate, set reset=True in the calibrate method or call reset() first."
            )

    def stop(self) -> None:
        self._is_streaming = False
        if hasattr(self, "_smbus"):
            self._smbus.close()

    def _read_compressed_strain(self):
        """Used for more recent versions of strain amp firmware"""
        try:
            data = self._smbus.read_i2c_block_data(
                self._i2c_address, MEMORY_CHANNELS.CH1_H, 10
            )
            self.failed_reads = 0
        except OSError as e:
            self.failed_reads += 1

            if self.failed_reads >= 5:
                raise Exception("Load cell unresponsive.")

        return self._unpack_compressed_strain(data)

    @staticmethod
    def _unpack_uncompressed_strain(data):
        """Used for an older version of the strain amp firmware (at least pre-2017)"""
        ch1 = (data[0] << 8) | data[1]
        ch2 = (data[2] << 8) | data[3]
        ch3 = (data[4] << 8) | data[5]
        ch4 = (data[6] << 8) | data[7]
        ch5 = (data[8] << 8) | data[9]
        ch6 = (data[10] << 8) | data[11]
        return np.array(object=[ch1, ch2, ch3, ch4, ch5, ch6])

    @staticmethod
    def _unpack_compressed_strain(data):
        """Used for more recent versions of strainamp firmware"""
        return np.array(
            object=[
                (data[0] << 4) | ((data[1] >> 4) & 0x0F),
                ((data[1] << 8) & 0x0F00) | data[2],
                (data[3] << 4) | ((data[4] >> 4) & 0x0F),
                ((data[4] << 8) & 0x0F00) | data[5],
                (data[6] << 4) | ((data[7] >> 4) & 0x0F),
                ((data[7] << 8) & 0x0F00) | data[8],
            ]
        )

    @property
    def is_calibrated(self) -> bool:
        """Indicates if load cell zeroing routine has been called."""
        return self._is_calibrated

    @property
    def is_streaming(self) -> bool:
        return self._is_streaming

    @property
    def fx(self):
        """
        Latest force in the x (medial/lateral) direction in Newtons.
        If using the standard OSL setup, this is positive towards the user's right.
        """
        return self._data[0]

    @property
    def fy(self):
        """
        Latest force in the y (anterior/posterior) direction in Newtons.
        If using the standard OSL setup, this is positive in the posterior direction.
        """
        return self._data[1]

    @property
    def fz(self):
        """
        Latest force in the z (vertical) direction in Newtons.
        If using the standard OSL setup, this should be positive downwards.
        i.e. quiet standing on the OSL should give a negative Fz.
        """
        return self._data[2]

    @property
    def mx(self):
        """
        Latest moment about the x (medial/lateral) axis in Nm.
        If using the standard OSL setup, this axis is positive towards the user's right.
        """
        return self._data[3]

    @property
    def my(self):
        """
        Latest moment about the y (anterior/posterior) axis in Nm.
        If using the standard OSL setup, this axis is positive in the posterior direction.
        """
        return self._data[4]

    @property
    def mz(self):
        """
        Latest moment about the z (vertical) axis in Nm.
        If using the standard OSL setup, this axis is positive towards the ground.
        """
        return self._data[5]

    @property
    def data(self):
        """
        Returns a vector of the latest loadcell data.
        [Fx, Fy, Fz, Mx, My, Mz]
        Forces in N, moments in Nm.
        """
        if self._data is not None:
            return self._data[0]
        else:
            return self._zero_calibration_offset
