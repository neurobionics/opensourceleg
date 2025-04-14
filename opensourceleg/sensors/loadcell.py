"""
Module for interfacing with SRI Loadcell sensors.

This module provides an implementation of a load cell sensor (DephyLoadcellAmplifier) that
inherits from LoadcellBase. It uses an I2C interface via SMBus to communicate with
a strain amplifier and processes raw ADC data to compute forces and moments.

Classes:
    LoadcellNotRespondingException: Exception raised when the load cell does not respond.
    MEMORY_CHANNELS: Enum representing memory channel addresses for load cell readings.
    DephyLoadcellAmplifier: Concrete implementation of a load cell sensor that provides force and moment data.

Dependencies:
    - numpy
    - smbus2
    - opensourceleg.logging
    - opensourceleg.sensors.base
"""

import time
from enum import Enum
from typing import Any, Callable, Optional

import numpy as np
import numpy.typing as npt
from smbus2 import SMBus

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import LoadcellBase


class LoadcellNotRespondingException(Exception):
    """
    Exception raised when the load cell fails to respond.

    Attributes:
        message: Description of the error.
    """

    def __init__(self, message: str = "Load cell unresponsive.") -> None:
        """
        Initialize the LoadcellNotRespondingException.

        Args:
            message: Error message. Defaults to "Load cell unresponsive.".
        """
        super().__init__(message)


class DEPHY_AMPLIFIER_MEMORY_CHANNELS(int, Enum):
    """
    Enumeration of memory channel addresses used by the load cell.

    Each channel corresponds to a specific high or low byte of the ADC data.
    """

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


class DephyLoadcellAmplifier(LoadcellBase):
    """
    Implementation of a load cell sensor using the Dephy Loadcell Amplifier.

    This class communicates with the dephy strain amplifier via I2C using the SMBus interface,
    processes the raw ADC data, and computes forces (Fx, Fy, Fz) and moments (Mx, My, Mz)
    based on a provided calibration matrix and hardware configuration.

    Class Attributes:
        ADC_RANGE: The maximum ADC value (2**12 - 1).
        OFFSET: The ADC mid-scale offset (half of 2**12).
    """

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
        """
        Initialize the Dephy loadcell amplifier.

        Validates the provided parameters and initializes internal variables for data
        acquisition, calibration, and streaming.

        Args:
            calibration_matrix: A 6x6 calibration matrix.
            amp_gain: Amplifier gain; must be greater than 0. Defaults to 125.0.
            exc: Excitation voltage; must be greater than 0. Defaults to 5.0.
            bus: I2C bus number to use. Defaults to 1.
            i2c_address: I2C address of the strain amplifier. Defaults to 0x66.

        Raises:
            TypeError: If calibration_matrix is not a 6x6 array.
            ValueError: If amp_gain or exc are not greater than 0.
        """
        # Validate input parameters.
        if calibration_matrix.shape != (6, 6):
            LOGGER.info(f"[{self.__repr__()}] calibration_matrix must be a 6x6 array of np.double.")
            raise TypeError("calibration_matrix must be a 6x6 array of np.double.")
        if amp_gain <= 0:
            LOGGER.info(f"[{self.__repr__()}] amp_gain must be a floating point value greater than 0.")
            raise ValueError("amp_gain must be a floating point value greater than 0.")
        if exc <= 0:
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

        self._calibration_offset: npt.NDArray[np.double] = np.zeros(shape=(1, 6), dtype=np.double)
        self._zero_calibration_offset: npt.NDArray[np.double] = self._calibration_offset
        self._is_calibrated: bool = False
        self._is_streaming: bool = False

    def start(self) -> None:
        """
        Start the load cell sensor.

        Opens the I2C connection using SMBus, waits briefly for hardware stabilization,
        and sets the streaming flag to True.
        """
        self._smbus = SMBus(self._bus)
        time.sleep(1)
        self._is_streaming = True

    def reset(self) -> None:
        """
        Reset the load cell calibration.

        Resets the calibration offset to the zero value and marks the sensor as uncalibrated.
        """
        self._calibration_offset = self._zero_calibration_offset
        self._is_calibrated = False

    def update(
        self,
        calibration_offset: Optional[npt.NDArray[np.double]] = None,
        data_callback: Optional[Callable[..., npt.NDArray[np.uint8]]] = None,
    ) -> None:
        """
        Query the load cell for the latest data and update internal state.

        Reads raw ADC data (either via a provided callback or by reading from I2C),
        converts it to engineering units using the calibration matrix, amplifier gain,
        and excitation voltage, and subtracts any calibration offset.

        Args:
            calibration_offset: An offset to subtract from the processed data.
            If None, uses the current calibration offset.
            data_callback: A callback function to provide raw data.
            If not provided, the sensor's internal method is used.
        """
        data = data_callback() if data_callback else self._read_compressed_strain()

        if calibration_offset is None:
            calibration_offset = self._calibration_offset

        signed_data = ((data - self.OFFSET) / self.ADC_RANGE) * self._exc
        coupled_data = signed_data * 1000 / (self._exc * self._amp_gain)

        # Process the data using the calibration matrix and subtract the offset.
        self._data = np.transpose(a=self._calibration_matrix.dot(b=np.transpose(a=coupled_data))) - calibration_offset

    def calibrate(
        self,
        number_of_iterations: int = 2000,
        reset: bool = False,
        data_callback: Optional[Callable[[], npt.NDArray[np.uint8]]] = None,
    ) -> None:
        """
        Perform a zeroing (calibration) routine for the load cell.

        This method obtains an initial zero-load reading that is subtracted from subsequent
        measurements. If the sensor has already been calibrated and 'reset' is False, a log message
        is displayed.

        Args:
            number_of_iterations: Number of iterations to average for calibration.
            reset: If True, forces recalibration by resetting the current calibration.
            data_callback: Optional callback function to provide raw data.
            If not provided, the sensor's internal method is used.
        """
        if not self.is_calibrated:
            LOGGER.info(
                f"[{self.__repr__()}] Initiating zeroing routine, please ensure that there is no ground contact force."
            )
            input("Press any key to start.")

            self.update(data_callback=data_callback)
            self._calibration_offset = self._data

            for _ in range(number_of_iterations):
                self.update(
                    calibration_offset=self._zero_calibration_offset,
                    data_callback=data_callback,
                )
                iterative_calibration_offset = self._data
                self._calibration_offset = (iterative_calibration_offset + self._calibration_offset) / 2.0

            self._is_calibrated = True
            LOGGER.info(f"[{self.__repr__()}] Calibration routine complete.")

        elif reset:
            self.reset()
            self.calibrate()
        else:
            LOGGER.info(
                f"[{self.__repr__()}] Loadcell has already been zeroed. "
                "To recalibrate, set reset=True in the calibrate method or call reset() first."
            )

    def stop(self) -> None:
        """
        Stop the load cell sensor.

        Sets the streaming flag to False and closes the I2C connection if it is open.
        """
        self._is_streaming = False
        if hasattr(self, "_smbus"):
            self._smbus.close()

    def _read_compressed_strain(self) -> Any:
        """
        Read raw strain data from the load cell via I2C.

        Returns:
            Raw strain data from the load cell.
        """
        try:
            data = self._smbus.read_i2c_block_data(self._i2c_address, DEPHY_AMPLIFIER_MEMORY_CHANNELS.CH1_H, 10)
            self.failed_reads = 0
        except OSError:
            self.failed_reads += 1

            if self.failed_reads >= 5:
                raise LoadcellNotRespondingException("Load cell unresponsive.") from None

        return self._unpack_compressed_strain(np.array(object=data, dtype=np.uint8))

    @staticmethod
    def _unpack_uncompressed_strain(data: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint16]:
        """
        Unpack uncompressed strain data from raw bytes.

        Args:
            data: Raw data read from the sensor.

        Returns:
            Unpacked strain data.
        """
        ch1 = (data[0] << 8) | data[1]
        ch2 = (data[2] << 8) | data[3]
        ch3 = (data[4] << 8) | data[5]
        ch4 = (data[6] << 8) | data[7]
        ch5 = (data[8] << 8) | data[9]
        ch6 = (data[10] << 8) | data[11]
        return np.array(object=[ch1, ch2, ch3, ch4, ch5, ch6])

    @staticmethod
    def _unpack_compressed_strain(data: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint16]:
        """
        Unpack compressed strain data from raw bytes.

        Args:
            data: Raw data read from the sensor.

        Returns:
            Unpacked strain data.
        """
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
        """
        Check if the load cell has been calibrated.

        Returns:
            True if the load cell has been calibrated, False otherwise.
        """
        return self._is_calibrated

    @property
    def is_streaming(self) -> bool:
        """
        Check if the load cell is currently streaming data.

        Returns:
            True if the load cell is streaming data, False otherwise.
        """
        return self._is_streaming

    @property
    def fx(self) -> float:
        """
        Get the force in the x-direction.

        Returns:
            Force in the x-direction in Newtons.
        """
        return self.data[0]

    @property
    def fy(self) -> float:
        """
        Get the force in the y-direction.

        Returns:
            Force in the y-direction in Newtons.
        """
        return self.data[1]

    @property
    def fz(self) -> float:
        """
        Get the force in the z-direction.

        Returns:
            Force in the z-direction in Newtons.
        """
        return self.data[2]

    @property
    def mx(self) -> float:
        """
        Get the moment about the x-axis.

        Returns:
            Moment about the x-axis in Newton-meters.
        """
        return self.data[3]

    @property
    def my(self) -> float:
        """
        Get the moment about the y-axis.

        Returns:
            Moment about the y-axis in Newton-meters.
        """
        return self.data[4]

    @property
    def mz(self) -> float:
        """
        Get the moment about the z-axis.

        Returns:
            Moment about the z-axis in Newton-meters.
        """
        return self.data[5]

    @property
    def data(self) -> list[float]:
        """
        Get all force and moment data as a list.

        Returns:
            List containing [Fx, Fy, Fz, Mx, My, Mz] in Newtons and Newton-meters.
        """
        if self._data is not None:
            return list(map(float, self._data[0].tolist()))
        else:
            return list(map(float, self._zero_calibration_offset.tolist()))
