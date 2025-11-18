"""
Module for interfacing with Loadcell amplifiers.

This module provides implements reading force/moment data from a 6-axis loadcell
using DAQs by the Neurobionics lab, and Dephy.

Classes:
    LoadcellNotRespondingException: Exception raised when the load cell does not respond.
    DEPHY_AMPLIFIER_MEMORY_CHANNELS: Enum representing memory channel addresses for load cell readings.
    DephyLoadcellAmplifier: Read and process force/moment data from a 6-axis load cell with a DAQ by Dephy.
    NBLoadcellDAQ: Read and process force/moment data from a 6-axis load cell with DAQs by Neurobionics Lab.

Dependencies:
    - numpy
    - smbus2
    - spidev
    - opensourceleg.logging
    - opensourceleg.sensors.base
    - opensourceleg.sensors.adc
"""

import time
from enum import Enum
from typing import Any, Callable, Optional

import numpy as np
import numpy.typing as npt
from smbus2 import SMBus

from opensourceleg.logging import LOGGER
from opensourceleg.math.math import Counter
from opensourceleg.sensors.adc import ADS131M0x
from opensourceleg.sensors.base import LoadcellBase


class LoadcellNotRespondingException(Exception):
    """
    Exception raised when the load cell fails to respond.

    Attributes:
        message (str): Description of the error.
    """

    def __init__(self, message: str = "Load cell unresponsive.") -> None:
        """
        Initialize the LoadcellNotRespondingException.

        Args:
            message (str, optional): Error message. Defaults to "Load cell unresponsive.".
        """
        super().__init__(message)


class LoadcellBrokenWireDetectedException(Exception):
    """
    Exception raised when a broken wire is detected in the load cell.
    Indicated by saturated ADC values (0 or 4095).

    Attributes:
        message (str): Description of the error.
    """

    def __init__(self, message: str = "Load cell broken wire detected.") -> None:
        """
        Initialize the LoadcellBrokenWireDetectedException.

        Args:
            message (str, optional): Error message. Defaults to "Load cell broken wire detected.".
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

    This class communicates with the Dephy strain amplifier.
    It can connect via either I2C using the SMBus interface, or using custom data callbacks.
    It processes the raw ADC data, and computes forces (Fx, Fy, Fz) and moments (Mx, My, Mz)
    based on a provided calibration matrix and hardware configuration.

    Class Attributes:
        ADC_RANGE (int): The maximum ADC value (2**12 - 1).
        OFFSET (float): The ADC mid-scale offset (half of 2**12).
    """

    ADC_RANGE = 2**12 - 1
    OFFSET = 2**12 / 2

    def __init__(
        self,
        calibration_matrix: npt.NDArray[np.double],
        tag: str = "DephyLoadcellAmplifier",
        amp_gain: float = 125.0,
        exc: float = 5.0,
        bus: int = 1,
        i2c_address: int = 0x66,
        offline: bool = False,
        enable_diagnostics: bool = True,
    ) -> None:
        """
        Initialize the Dephy loadcell amplifier.

        Validates the provided parameters and initializes internal variables for data
        acquisition, calibration, and streaming.

        Args:
            calibration_matrix (npt.NDArray[np.double]): A 6x6 calibration matrix.
            tag (str, optional): A tag for identifying the load cell instance. Defaults to "DephyLoadcellAmplifier".
            amp_gain (float, optional): Amplifier gain; must be greater than 0. Defaults to 125.0.
            exc (float, optional): Excitation voltage; must be greater than 0. Defaults to 5.0.
            bus (int, optional): I2C bus number to use. Defaults to 1.
            i2c_address (int, optional): I2C address of the strain amplifier. Defaults to 0x66.

        Raises:
            TypeError: If calibration_matrix is not a 6x6 array.
            ValueError: If amp_gain or exc are not greater than 0.
        """
        super().__init__(tag=tag, offline=offline)

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
        self._enable_diagnostics: bool = enable_diagnostics
        self._data_potentially_invalid: bool = False
        if self._enable_diagnostics:
            self._diagnostics_counter = Counter()
        self._num_broken_wire_pre_exception: int = 5

    def start(self) -> None:
        """
        Start the load cell sensor.

        If using I2C Mode, it opens the I2C connection using SMBus, waits briefly for hardware stabilization,
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
        data_callback: Optional[Callable[..., npt.NDArray[np.uint16]]] = None,
    ) -> None:
        """
        Query the load cell for the latest data and update internal state.

        Reads raw ADC data (either via a provided callback or by reading from I2C),
        converts it to engineering units using the calibration matrix, amplifier gain,
        and excitation voltage, and subtracts any calibration offset.

        Args:
            calibration_offset (Optional[npt.NDArray[np.double]], optional):
                An offset to subtract from the processed data. If None, uses the current calibration offset.
            data_callback (Optional[Callable[..., npt.NDArray[np.uint8]]], optional):
                A callback function to provide raw data. If not provided, the sensor's internal i2c method is used.

        Raises:
            ValueError: If the update method fails due to misconfiguration.
        """
        data = data_callback() if data_callback else self._read_compressed_strain()

        if self._enable_diagnostics:
            self.check_data(data)

        if calibration_offset is None:
            calibration_offset = self._calibration_offset

        signed_data = ((data - self.OFFSET) / self.ADC_RANGE) * self._exc
        coupled_data = signed_data * 1000 / (self._exc * self._amp_gain)

        # Process the data using the calibration matrix and subtract the offset.
        self._data = np.transpose(a=self._calibration_matrix.dot(b=np.transpose(a=coupled_data))) - calibration_offset

    def check_data(self, data: npt.NDArray[np.uint16]) -> None:
        """
        Watches raw values from the load cell to try to catch broken wires.
        Symptom is indicated by saturation at either max or min ADC values.
        """
        ADC_saturated_high = np.any(data == self.ADC_RANGE)  # Use np.any for NumPy arrays
        ADC_saturated_low = np.any(data == 0)  # Use np.any for NumPy arrays
        concerning_data_found = bool(ADC_saturated_high or ADC_saturated_low)
        self._diagnostics_counter.update(concerning_data_found)
        if self._diagnostics_counter.current_count >= self._num_broken_wire_pre_exception:
            raise LoadcellBrokenWireDetectedException(
                f"[{self.__repr__()}] Consistent saturation in readings, check wiring. ADC values: {self._data}."
            )

    def calibrate(
        self,
        number_of_iterations: int = 2000,
        reset: bool = False,
        data_callback: Optional[Callable[[], npt.NDArray[np.uint16]]] = None,
    ) -> None:
        """
        Perform a zeroing (calibration) routine for the load cell.

        This method obtains an initial zero-load reading that is subtracted from subsequent
        measurements. If the sensor has already been calibrated and 'reset' is False, a log message
        is displayed.

        Args:
            number_of_iterations (int, optional): Number of iterations to average for calibration.
                Defaults to 2000.
            reset (bool, optional): If True, forces recalibration by resetting the current calibration.
                Defaults to False.
            data_callback (Optional[Callable[[], npt.NDArray[np.uint8]]], optional): Optional callback
                to provide raw data. Defaults to None.
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

    def _read_compressed_strain(self) -> npt.NDArray[np.uint16]:
        """
        Read and unpack compressed strain data from the sensor.

        This method reads a block of data from the sensor via I2C and then unpacks it
        using the compressed strain format. If multiple read attempts fail, a
        LoadcellNotRespondingException is raised.

        Returns:
            Any: The unpacked strain data.
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
        Unpack raw ADC data using the uncompressed format.

        This method is used for older versions of the strain amplifier firmware (pre-2017).

        Args:
            data (npt.NDArray[np.uint8]): Raw data read from the sensor.

        Returns:
            npt.NDArray[np.uint16]: An array containing the unpacked values for 6 channels.
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
        Unpack raw ADC data using the compressed format.

        This method is used for more recent versions of the strain amplifier firmware.

        Args:
            data (npt.NDArray[np.uint8]): Raw data read from the sensor.

        Returns:
            npt.NDArray[np.uint16]: An array containing the unpacked values for 6 channels.
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
        Indicates whether the load cell has been calibrated (zeroed).

        Returns:
            bool: True if the calibration routine has been successfully completed; otherwise, False.
        """
        return self._is_calibrated

    @property
    def is_streaming(self) -> bool:
        """
        Check if the load cell is currently streaming data.

        Returns:
            bool: True if streaming; otherwise, False.
        """
        return self._is_streaming

    @property
    def fx(self) -> float:
        """
        Get the latest force in the x (medial/lateral) direction in Newtons.

        For the standard OSL setup, this value is positive towards the user's right.

        Returns:
            float: Force (N) along the x-axis.
        """
        return self.data[0]

    @property
    def fy(self) -> float:
        """
        Get the latest force in the y (anterior/posterior) direction in Newtons.

        For the standard OSL setup, this value is positive in the posterior direction.

        Returns:
            float: Force (N) along the y-axis.
        """
        return self.data[1]

    @property
    def fz(self) -> float:
        """
        Get the latest force in the z (vertical) direction in Newtons.

        For the standard OSL setup, this value is positive downwards. In quiet standing,
        a negative Fz value is expected.

        Returns:
            float: Force (N) along the z-axis.
        """
        return self.data[2]

    @property
    def mx(self) -> float:
        """
        Get the latest moment about the x (medial/lateral) axis in Nm.

        For the standard OSL setup, this moment is positive towards the user's right.

        Returns:
            float: Moment (Nm) about the x-axis.
        """
        return self.data[3]

    @property
    def my(self) -> float:
        """
        Get the latest moment about the y (anterior/posterior) axis in Nm.

        For the standard OSL setup, this moment is positive in the posterior direction.

        Returns:
            float: Moment (Nm) about the y-axis.
        """
        return self.data[4]

    @property
    def mz(self) -> float:
        """
        Get the latest moment about the z (vertical) axis in Nm.

        For the standard OSL setup, this moment is positive towards the ground.

        Returns:
            float: Moment (Nm) about the z-axis.
        """
        return self.data[5]

    @property
    def data(self) -> list[float]:
        """
        Get the latest processed load cell data.

        Returns:
            list[float]: A 1D vector containing [Fx, Fy, Fz, Mx, My, Mz], where forces are in Newtons and moments in Nm.
        """
        if self._data is not None:
            return list(map(float, self._data[0].tolist()))
        else:
            return list(map(float, self._zero_calibration_offset.tolist()))


class NBLoadcellDAQ(LoadcellBase):
    """
    Implementation of a load cell DAQ system using the ADS131M0x ADC.

    This class provides methods for acquiring, calibrating, and processing load cell data.
    It uses an ADC to read raw data, applies a calibration matrix, and computes forces and moments.

    Attributes:
        calibration_matrix (np.ndarray): A matrix used to convert raw ADC data into forces and moments.
        excitation_voltage (float): The excitation voltage applied to the load cell.
        amp_gain (np.ndarray): Amplifier gain values for each channel.
        adc (ADS131M0x): The ADC instance used for data acquisition.
        is_calibrated (bool): Indicates whether the load cell has been calibrated.
        is_streaming (bool): Indicates whether the ADC is currently streaming data.
    """

    def __init__(
        self,
        calibration_matrix: npt.NDArray[np.double],
        tag: str = "NBLoadcellDAQ",
        excitation_voltage: float = 5.0,
        amp_gain: Optional[list[int]] = None,
        offline: bool = False,
        **kwargs: Any,
    ) -> None:
        """
        Initialize the NBLoadcellDAQ instance.

        Args:
            calibration_matrix (np.ndarray): A matrix used to convert raw ADC data into forces and moments.
            tag (str): Identifier for the load cell instance. Defaults to 'NBLoadcellDAQ'.
            excitation_voltage (float): The excitation voltage applied to the load cell. Defaults to 5.0 V.
            amp_gain (list[int): Amplifier gain values for each channel. Defaults to [34, 34, 34, 151, 151, 151].
            offline (bool): If True, the load cell operates in offline mode. Defaults to False.
            **kwargs: Additional arguments passed to the ADS131M0x ADC instance.
        """
        if amp_gain is None:
            amp_gain = [34, 34, 34, 151, 151, 151]
        super().__init__(tag=tag, offline=offline)

        self._adc = ADS131M0x(**kwargs)
        self.calibration_matrix = calibration_matrix
        self.excitation_voltage = excitation_voltage
        self._offset = np.zeros(self.adc.num_channels)
        self.amp_gain = np.array(amp_gain)
        self._is_calibrated = False

    @property
    def adc(self) -> ADS131M0x:
        """
        Get the ADC instance used for data acquisition.

        Returns:
            ADS131M0x: The ADC instance.
        """
        return self._adc

    def __repr__(self) -> str:
        return "Loadcell"

    @property
    def is_streaming(self) -> bool:
        """
        Check if the ADC is currently streaming data.

        Returns:
            bool: True if the ADC is streaming, False otherwise.
        """
        return self.adc._streaming

    def start(self) -> None:
        """
        Start the load cell DAQ system.

        This method initializes the ADC and begins data acquisition.
        """
        LOGGER.info(f"[{self.__repr__()}] Starting data acquisition...")
        self.adc.start()

    def stop(self) -> None:
        """
        Stop the load cell DAQ system.

        This method stops the ADC and ends data acquisition.
        """
        LOGGER.info(f"[{self.__repr__()}] Stopping data acquisition...")
        self.adc.stop()

    def update(self) -> None:
        """
        Update the load cell data.

        Reads the latest data from the ADC, applies amplifier gains and calibration offsets,
        and computes forces and moments using the calibration matrix.
        """
        self.adc.update()

        # Apply calibration offsets and amplifier gains
        coupled = self.adc.data - self._offset
        coupled[1::2] *= -1  # Invert odd-indexed channels for correct orientation

        gains = self.amp_gain * self.adc.gains
        normalized = coupled / (self.excitation_voltage * gains)

        # Compute forces and moments using the calibration matrix
        self._data = self.calibration_matrix @ normalized

    def reset(self) -> None:
        """
        Reset the load cell DAQ system.

        Resets the ADC and clears any calibration offsets.
        """
        LOGGER.info(f"[{self.__repr__()}] Resetting calibration offsets...")
        self.adc.reset()
        self._offset = np.zeros(self.adc.num_channels)
        self._is_calibrated = False

    def calibrate(self, n_samples: int = 1024) -> None:
        """
        Perform a calibration routine to zero the load cell.

        This method determines the zero-load offset by averaging multiple samples
        and stores it for subsequent data processing.

        Args:
            n_samples (int, optional): Number of samples to average for calibration. Defaults to 1024.
        """
        LOGGER.info(f"[{self.__repr__()}] Calibrating ADS131M0x...")
        self.adc.calibrate()
        LOGGER.info(f"[{self.__repr__()}] Starting calibration with {n_samples} samples...")
        offsets = np.empty((n_samples, self.adc.num_channels))

        for i in range(n_samples):
            self.adc.update()
            offsets[i] = self.adc.data

        self._offset = offsets.mean(axis=0)
        self._is_calibrated = True
        LOGGER.info(f"[{self.__repr__()}] Calibration complete.")

    @property
    def is_calibrated(self) -> bool:
        """
        Check if the load cell has been calibrated.

        Returns:
            bool: True if the load cell has been calibrated, False otherwise.
        """
        return self._is_calibrated

    @property
    def data(self) -> Any:
        """
        Get the latest processed load cell data.

        Returns:
            np.ndarray: A 1D vector containing [Fx, Fy, Fz, Mx, My, Mz], where:
                - Fx, Fy, Fz are forces in Newtons.
                - Mx, My, Mz are moments in Newton-meters.
        """
        return self._data

    @property
    def fx(self) -> float:
        """
        Get the latest force in the x (medial/lateral) direction in Newtons.

        Returns:
            float: Force (N) along the x-axis.
        """
        return float(self.data[0])

    @property
    def fy(self) -> float:
        """
        Get the latest force in the y (anterior/posterior) direction in Newtons.

        Returns:
            float: Force (N) along the y-axis.
        """
        return float(self.data[1])

    @property
    def fz(self) -> float:
        """
        Get the latest force in the z (vertical) direction in Newtons.

        Returns:
            float: Force (N) along the z-axis.
        """
        return float(self.data[2])

    @property
    def mx(self) -> float:
        """
        Get the latest moment about the x (medial/lateral) axis in Nm.

        Returns:
            float: Moment (Nm) about the x-axis.
        """
        return float(self.data[3])

    @property
    def my(self) -> float:
        """
        Get the latest moment about the y (anterior/posterior) axis in Nm.

        Returns:
            float: Moment (Nm) about the y-axis.
        """
        return float(self.data[4])

    @property
    def mz(self) -> float:
        """
        Get the latest moment about the z (vertical) axis in Nm.

        Returns:
            float: Moment (Nm) about the z-axis.
        """
        return float(self.data[5])
