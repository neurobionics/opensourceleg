"""
Module for communicating with the ADS131M0x family of ADC chips.
"""

import math
from time import sleep
from typing import Any, ClassVar, Optional

import numpy as np

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import ADCBase


class ADS131M0x(ADCBase):
    """
    Class for communication with the ADS131M0x family of ADC chips.

    This class allows configuration of the ADS131M0x chips and reading ADC values in millivolts.
    """

    # Class attributes
    _MAX_CHANNELS = 8
    _BYTES_PER_WORD = 3
    _RESOLUTION = 24
    _SPI_MODE = 1
    _DATA_RATES = (250, 500, 1000, 2000, 4000, 8000, 16000, 32000)

    _BLANK_WORD: ClassVar[list[int]] = [0x00, 0x00, 0x00]
    _RESET_WORD: ClassVar[list[int]] = [0x00, 0x11, 0x00]
    _STANDBY_WORD: ClassVar[list[int]] = [0x00, 0x22, 0x00]
    _WAKEUP_WORD: ClassVar[list[int]] = [0x00, 0x33, 0x00]
    _RREG_PREFIX = 0b101
    _WREG_PREFIX = 0b011

    _ID_REG = 0x00
    _STATUS_REG = 0x01
    _MODE_REG = 0x02
    _CLOCK_REG = 0x03
    _GAIN1_REG = 0x04
    _GAIN2_REG = 0x05
    _CFG_REG = 0x06

    _DISABLE_CHANNELS_CLOCK = 0x000E
    _ENABLE_CHANNELS_CLOCK = 0xFF0E
    _MODE_CFG = 0x0110

    _OCAL_MSB_ADDRS: ClassVar[list[int]] = [0x0A, 0x0F, 0x14, 0x19, 0x1E, 0x23, 0x28, 0x2D]
    _OCAL_LSB_ADDRS: ClassVar[list[int]] = [0x0B, 0x10, 0x15, 0x1A, 0x1F, 0x24, 0x29, 0x2E]
    _GCAL_MSB_ADDRS: ClassVar[list[int]] = [0x0C, 0x11, 0x16, 0x1B, 0x20, 0x25, 0x2A, 0x2F]
    _GCAL_LSB_ADDRS: ClassVar[list[int]] = [0x0D, 0x12, 0x17, 0x1C, 0x21, 0x26, 0x2B, 0x30]
    _CHANNEL_CFG_ADDRS: ClassVar[list[int]] = [0x09, 0x0E, 0x13, 0x18, 0x1D, 0x22, 0x27, 0x2C]

    _GCAL_STEP_SIZE = 1.19e-7
    _READY_STATUS_BASE = 0x05 << 8

    def __init__(
        self,
        tag: str = "ADS131M0x",
        spi_bus: int = 0,
        spi_cs: int = 0,
        data_rate: int = 500,
        clock_freq: int = 8192000,
        num_channels: int = 6,
        gains: list[int] = [1] * 6,
        voltage_reference: float = 1.2,
        gain_error: Optional[list[int]] = None,
        offline: bool = False,
    ):
        """
        Initialize the ADS131M0x instance.

        Args:
            tag (str): Identifier for the ADC instance. Default is "ADS131M0x".
            spi_bus (int): SPI bus number. Default is 0.
            spi_cs (int): SPI chip select line. Default is 0.
            data_rate (int): Sampling rate in Hz. Default is 500 Hz.
            clock_freq (int): SPI clock frequency in Hz. Default is 8192000 Hz.
            num_channels (int): Number of ADC channels. Default is 6.
            gains (List[int]): Programmable gain values for each channel. Default is [1] * num_channels.
            voltage_reference (float): Reference voltage in volts. Default is 1.2 V.
            gain_error (List[int]): Gain error correction values for each channel. Default is None.
            offline (bool): If True, the ADC operates in offline mode. Default is False.

        Raises:
            ValueError: If the configuration parameters are invalid.
        """

        try:
            import spidev

            self._spi = spidev.SpiDev()
        except ImportError as e:
            LOGGER.error("spidev is not installed. Please install it to use this module.")
            raise ImportError("spidev is required but not installed.") from e

        if len(gains) != num_channels:
            raise ValueError("Length of gains must equal the number of channels.")
        if (gain_error is not None) and (len(gain_error) != num_channels):
            raise ValueError("Length of gain_error must equal the number of channels.")
        if data_rate not in self._DATA_RATES:
            raise ValueError(f"Invalid data rate. Must be one of {self._DATA_RATES}.")

        self._spi_bus = spi_bus
        self._spi_cs = spi_cs
        self._num_channels = num_channels
        self._clock_freq = clock_freq
        self._data_rate = data_rate
        self._gain_exponents = self._calculate_gain_exponents(gains)
        self._voltage_reference = voltage_reference
        self._gain_error = gain_error
        self._streaming = False
        self._words_per_frame = 2 + num_channels
        self._ready_status = self._calculate_ready_status()
        self._data_counts = np.empty(self.num_channels, dtype=int)
        self._data = np.empty(self.num_channels, dtype=float)

    def __repr__(self) -> str:
        return "ADS131M0x"

    def start(self) -> None:
        """
        Start the ADC by opening the SPI port, resetting the device, configuring gain settings,
        and transitioning to continuous conversion mode.
        """
        LOGGER.info("Starting ADC...")
        self._spi.open(self._spi_bus, self._spi_cs)
        self._spi.max_speed_hz = self._clock_freq
        self._spi.mode = self._SPI_MODE

        self.reset()
        self._set_gain()
        self._set_device_state(1)
        self._clear_stale_data()
        LOGGER.info("ADC started successfully.")

    def stop(self) -> None:
        """
        Stop the ADC by transitioning to standby mode and closing the SPI port.
        """
        LOGGER.info("Stopping ADC...")
        self._set_device_state(0)
        self._spi.close()
        LOGGER.info("ADC stopped successfully.")

    def reset(self) -> None:
        """
        Reset the ADC by sending the reset command via SPI.
        """
        self._spi.xfer2(self._RESET_WORD + self._BLANK_WORD * (self._words_per_frame - 1))

    def update(self) -> None:
        """
        Update the ADC data by reading the latest voltage values in millivolts.
        Attempts to read a maximum of 1000 times before throwing an error.
        """
        MAX_ATTEMPTS = 1000
        attempts = 0
        while not self._ready_to_read():
            sleep(0.001)
            attempts += 1
            if attempts > MAX_ATTEMPTS:
                raise RuntimeError(
                    "Couldn't connect to the ADC, please ensure that the device is connected and powered on."
                )

        self._data = self._read_data_millivolts()

    def calibrate(self) -> None:
        """
        Perform offset and gain calibration on the ADC.
        """
        self._offset_calibration()
        # if self._gain_error is not None:
        #     self._gain_calibration()

    def read_register(self, address: int) -> int:
        """
        Read the value of a register at the specified address.

        Args:
            address (int): Address of the register to read.

        Returns:
            int: Value stored in the register.
        """
        msg = (address << 7) | (self._RREG_PREFIX << 13)
        word = self._message_to_word(msg)
        rsp = self._spi_message(word)
        return int(rsp[0] << 8 | rsp[1])

    def write_register(self, address: int, reg_val: int) -> None:
        """
        Write a value to a register at the specified address.

        Args:
            address (int): Address of the register to write.
            reg_val (int): Value to write to the register.
        """
        addr_msg = (address << 7) | (self._WREG_PREFIX << 13)
        addr_bytes = self._message_to_word(addr_msg)
        reg_bytes = self._message_to_word(reg_val)
        self._spi_message(addr_bytes + reg_bytes)

    @property
    def is_streaming(self) -> bool:
        """
        Check if the ADC is currently streaming data.

        Returns:
            bool: True if streaming, False otherwise.
        """
        return self._streaming

    @property
    def gains(self) -> np.ndarray:
        """
        Get the programmable gain values for each channel.

        Returns:
            np.ndarray: Array of gain values.
        """
        return np.power(2, self._gain_exponents)

    @property
    def data(self) -> np.ndarray:
        """
        Get the latest ADC data in millivolts.

        Returns:
            np.ndarray: Array of voltage readings for each channel.
        """
        return self._data

    @property
    def data_counts(self) -> np.ndarray:
        """
        Get the latest ADC data in raw counts.

        Returns:
            np.ndarray: Array of raw ADC counts for each channel.
        """
        return self._data_counts

    @property
    def num_channels(self) -> int:
        """
        Get the number of ADC channels.

        Returns:
            int: Number of channels.
        """
        return self._num_channels

    def _calculate_gain_exponents(self, gains: list[int]) -> list[int]:
        """
        Calculate gain exponents for the programmable gains.

        Args:
            gains (List[int]): List of programmable gains.

        Returns:
            List[int]: List of gain exponents.

        Raises:
            ValueError: If a gain is not a power of 2 between 1 and 128.
        """
        gain_exponents = []
        for gain in gains:
            if not (1 <= gain <= 128 and math.log2(gain).is_integer()):
                raise ValueError("Each gain must be a power of 2 between 1 and 128.")
            gain_exponents.append(int(math.log2(gain)))
        return gain_exponents

    def _calculate_ready_status(self) -> int:
        """
        Calculate the ready status bitmask for the ADC.

        Returns:
            int: Ready status bitmask.
        """
        ready_status = self._READY_STATUS_BASE
        for i in range(self._num_channels):
            ready_status |= 1 << i
        return ready_status

    def _spi_message(self, msg: list[int]) -> Any:
        """Send SPI message to ADS131M0x.

        Args:
         - msg (List[int]): message to be sent to the ADS131M0x separated into bytes.
        Returns:
            list[int]: The response from the device, representing the entire frame.
        """
        self._spi.xfer2(msg)
        return self._spi.readbytes(self._BYTES_PER_WORD * self._words_per_frame)

    def _channel_enable(self, state: bool) -> None:
        """
        Enable or disable streaming on all channels.

        Args:
            state (bool): If True, enables the channel clocks; if False, disables them.
        """
        OSR = (self._clock_freq / 2) / self._data_rate
        OSR_reg = int(math.log2(OSR) - 7)
        self._ENABLE_CHANNELS_CLOCK &= ~(0b111 << 2)
        self._ENABLE_CHANNELS_CLOCK |= OSR_reg << 2
        self._DISABLE_CHANNELS_CLOCK &= ~(0b111 << 2)
        self._DISABLE_CHANNELS_CLOCK |= OSR_reg << 2
        if state is True:
            self.write_register(self._CLOCK_REG, self._ENABLE_CHANNELS_CLOCK)
        elif state is False:
            self.write_register(self._CLOCK_REG, self._DISABLE_CHANNELS_CLOCK)

    def _set_device_state(self, state: int) -> None:
        """
        Set the internal state of the ADC device.

        Args:
            state (int): The desired state:
                0 -- Standby mode.
                1 -- Continuous Conversion Mode.
        """
        if state == 0:
            self._spi.xfer2(self._STANDBY_WORD + self._BLANK_WORD * (self._words_per_frame - 1))
            self._streaming = False
        elif state == 1:
            self._spi.xfer2(self._WAKEUP_WORD + self._BLANK_WORD * (self._words_per_frame - 1))
            self._streaming = True

    def _set_voltage_source(self, source: int) -> None:
        """
        Change the voltage source for the ADC input.

        Args:
            source (int): The voltage source selection:
                0 -- external input.
                1 -- shorts differential pairs for a value near 0.
                2 -- positive internal test signal ((160mV / gain) * (Vref / 1.25)).
                3 -- negative internal test signal ((-160mV / gain) * (Vref / 1.25)).
        """
        for i in range(0, self.num_channels):
            self.write_register(self._CHANNEL_CFG_ADDRS[i], source)

    def _clear_stale_data(self) -> None:
        """Clears previous 2 stale data points stored in ADC registers."""
        for _ in range(2):
            self._read_data_counts()

    def _set_gain(self) -> None:
        """Set PGA gain for all channels."""

        gains = self._gain_exponents + [0] * (self._MAX_CHANNELS - len(self._gain_exponents))
        self._channel_enable(False)
        msg1 = gains[3] << 12 | gains[2] << 8 | gains[1] << 4 | gains[0]
        self.write_register(self._GAIN1_REG, msg1)
        msg2 = gains[7] << 12 | gains[6] << 8 | gains[5] << 4 | gains[4]
        self.write_register(self._GAIN2_REG, msg2)
        self._channel_enable(True)

    def _offset_calibration(self, n_samples: int = 1000) -> None:
        """Centers the ADC data around the measured zero value."""
        self._set_voltage_source(1)
        self._clear_stale_data()
        n_samples = 2**10
        offsets = np.empty((n_samples, self.num_channels))
        for i in range(n_samples):
            offsets[i] = self._read_data_counts()
        mean_offset = offsets.mean(axis=0, dtype=int)

        for i in range(0, self.num_channels):
            self.write_register(self._OCAL_MSB_ADDRS[i], mean_offset[i].item() >> 8)
            self.write_register(self._OCAL_LSB_ADDRS[i], (mean_offset[i].item() << 8) & 0xFF00)
        self._set_voltage_source(0)

    def _gain_calibration(self) -> None:
        """Corrects actual gain to desired gain using user-calculated gain error for each channel."""
        for i in range(self.num_channels):
            if self._gain_error is None:
                raise ValueError("Gain error is not set.")

            gain_correction = (1 + self._gain_error[i]) / self._GCAL_STEP_SIZE
            self.write_register(self._GCAL_MSB_ADDRS[i], int(gain_correction) >> 8)

    def _message_to_word(self, msg: int) -> list[int]:
        """Separates message into bytes to be sent to ADC."""
        word = [0] * 3
        word[0] = (msg >> 8) & 0xFF
        word[1] = msg & 0xFF
        return word

    def _ready_to_read(self) -> bool:
        """
        Check if all ADC channels are ready for a new data read.

        Returns:
            bool: True if the status register indicates readiness; otherwise, False.
        """
        reply = self.read_register(self._STATUS_REG)
        return reply == self._ready_status

    def _read_data_millivolts(self) -> Any:
        """Returns channel readings in millivolts."""
        self._data_counts = self._read_data_counts()
        mV = 1000 * self._data_counts / 2 ** (self._RESOLUTION - 1) * self._voltage_reference
        return mV

    def _read_data_counts(self) -> np.ndarray:
        """Returns channel readings in counts ranging from -2^23 -> 2^23-1"""
        reply = self._spi.readbytes(self._BYTES_PER_WORD * self._words_per_frame)
        data_counts = np.empty(self.num_channels, dtype=int)
        for byte in range(3, self.num_channels * 3 + 1, 3):
            val = (reply[byte] << 16) | (reply[byte + 1] << 8) | reply[byte + 2]
            data_counts[int(((byte) / 3) - 1)] = self._twos_complement(
                val,
                self._RESOLUTION,
            )
        return data_counts

    def _twos_complement(self, num: int, bits: int) -> int:
        """
        Convert an unsigned integer to a signed integer using two's complement representation.

        Args:
            num (int): The unsigned integer.
            bits (int): The number of bits used to represent the number.

        Returns:
            int: The signed integer value.
        """
        val = num
        if (num >> (bits - 1)) != 0:
            val = num - (1 << bits)
        return val
