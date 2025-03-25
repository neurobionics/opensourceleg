"""
Module for interfacing with the ADS131M0x family of ADC chips.

This module provides a class for communicating with and configuring the ADS131M0x
ADC chips. It supports SPI communication to reset, configure, calibrate, and read data
from the ADC in units of millivolts.
"""

import math
from time import sleep
from typing import Any, ClassVar, Optional

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import ADCBase


class ADS131M0x(ADCBase):
    """
    Class used for communication with the ADS131M0x family of ADC chips.

    This class allows you to configure the ADS131M0x family of chips as well as
    read out the ADC values in units of millivolts.

    Class Attributes:
        _MAX_CHANNELS (int): Maximum number of channels supported.
        _BYTES_PER_WORD (int): Number of bytes per word in SPI communication.
        _RESOLUTION (int): ADC resolution in bits.
        _SPI_MODE (int): SPI mode used for communication.
        _BLANK_WORD (ClassVar[list[int]]): Blank word for SPI messages.
        _RESET_WORD (ClassVar[list[int]]): SPI command for resetting the ADC.
        _STANDBY_WORD (ClassVar[list[int]]): SPI command for putting the ADC in standby.
        _WAKEUP_WORD (ClassVar[list[int]]): SPI command for waking up the ADC.
        _RREG_PREFIX (int): Prefix for read register commands.
        _WREG_PREFIX (int): Prefix for write register commands.
        _ID_REG (int): Address of the ID register.
        _STATUS_REG (int): Address of the status register.
        _MODE_REG (int): Address of the mode register.
        _CLOCK_REG (int): Address of the clock register.
        _GAIN1_REG (int): Address of the first gain register.
        _GAIN2_REG (int): Address of the second gain register.
        _CFG_REG (int): Address of the configuration register.
        _DISABLE_CHANNELS_CLOCK (int): Value to disable channel clocks.
        _ENABLE_CHANNELS_CLOCK (int): Value to enable channel clocks.
        _MODE_CFG (int): Mode configuration value.
        _OCAL_MSB_ADDRS (ClassVar[list[int]]): List of MSB addresses for offset calibration.
        _OCAL_LSB_ADDRS (ClassVar[list[int]]): List of LSB addresses for offset calibration.
        _GCAL_MSB_ADDRS (ClassVar[list[int]]): List of MSB addresses for gain calibration.
        _GCAL_LSB_ADDRS (ClassVar[list[int]]): List of LSB addresses for gain calibration.
        _CHANNEL_CFG_ADDRS (ClassVar[list[int]]): List of addresses for channel configuration.
        _GCAL_STEP_SIZE (float): Step size used in gain calibration.
    """

    _MAX_CHANNELS = 8
    _BYTES_PER_WORD = 3
    _RESOLUTION = 24
    _SPI_MODE = 1

    _BLANK_WORD: ClassVar = [0x00, 0x00, 0x00]
    # SPI Commands
    _RESET_WORD: ClassVar = [0x00, 0x11, 0x00]
    _STANDBY_WORD: ClassVar = [0x00, 0x22, 0x00]
    _WAKEUP_WORD: ClassVar = [0x00, 0x33, 0x00]
    _RREG_PREFIX = 0b101
    _WREG_PREFIX = 0b011

    # Multi-channel setting register addresses
    _ID_REG = 0x00
    _STATUS_REG = 0x01
    _MODE_REG = 0x02
    _CLOCK_REG = 0x03
    _GAIN1_REG = 0x04
    _GAIN2_REG = 0x05
    _CFG_REG = 0x06

    # Specific values to be written to registers
    _DISABLE_CHANNELS_CLOCK = 0x000E
    _ENABLE_CHANNELS_CLOCK = 0xFF0E
    _MODE_CFG = 0x0110

    # Channel specific setting register addresses
    _OCAL_MSB_ADDRS: ClassVar = [0x0A, 0x0F, 0x14, 0x19, 0x1E, 0x23, 0x28, 0x2D]
    _OCAL_LSB_ADDRS: ClassVar = [0x0B, 0x10, 0x15, 0x1A, 0x1F, 0x24, 0x29, 0x2E]
    _GCAL_MSB_ADDRS: ClassVar = [0x0C, 0x11, 0x16, 0x1B, 0x20, 0x25, 0x2A, 0x2F]
    _GCAL_LSB_ADDRS: ClassVar = [0x0D, 0x12, 0x17, 0x1C, 0x21, 0x26, 0x2B, 0x30]
    _CHANNEL_CFG_ADDRS: ClassVar = [0x09, 0x0E, 0x13, 0x18, 0x1D, 0x22, 0x27, 0x2C]

    _GCAL_STEP_SIZE = 1.19e-7

    def __init__(
        self,
        tag: str = "ADS131M0x",
        spi_bus: int = 0,
        spi_chip: int = 0,
        num_channels: int = 6,
        max_speed_hz: int = 8192000,
        channel_gains: list[int] = [32, 128] * 3,
        voltage_reference: float = 1.2,
        gain_error: Optional[list[int]] = None,
        offline: bool = False,
    ):
        """
        Initializes the ADS131M0x instance.

        Validates the provided configuration and sets up internal parameters for SPI communication,
        channel gains, and calibration.

        Args:
            spi_bus (int): SPI bus number where the ADC is connected. Default is 0.
            spi_chip (int): Chip select (CS) signal number for the ADC. Default is 0.
            num_channels (int): Number of active channels on the ADC. Default is 6.
            max_speed_hz (int): Maximum SPI clock speed in hertz. Default is 8192000.
            channel_gains (list[int]): List of gains for each channel's programmable gain amplifier.
                Default is [32, 128] repeated 3 times (for 6 channels).
            voltage_reference (float): Reference voltage used by the ADC. Default is 1.2 V.
            gain_error (Optional[list[int]]): Optional user-calculated gain error corrections per channel.
                Default is an empty list.

        Raises:
            ValueError: If the length of channel_gains does not equal num_channels,
                        or if gain_error is provided and its length does not equal num_channels,
                        or if any gain is not a power of 2 between 1 and 128.
        """

        try:
            import spidev

            self._spi = spidev.SpiDev()
        except ImportError:
            LOGGER.warning("spidev is not installed")
            exit(1)

        super().__init__(tag=tag, offline=offline)

        if gain_error is None:
            gain_error = []
        if len(channel_gains) != num_channels:
            raise ValueError("Length of channel_gains does not equal number of channels")
        if gain_error != [] and len(gain_error) != num_channels:
            raise ValueError("Length of channel_gains does not equal number of channels")

        self._spi_bus = spi_bus
        self._spi_chip = spi_chip
        self._num_channels = num_channels
        self._max_speed_hz = max_speed_hz
        self._gains = [0] * num_channels
        for i in range(num_channels):
            gain = int(math.log2(channel_gains[i]))
            if gain != math.log2(channel_gains[i]):
                raise ValueError("Gain must be a power of 2 between 1 and 128")
            self._gains[i] = gain

        self._voltage_reference = voltage_reference
        self._gain_error = gain_error
        self._streaming = False
        self._words_per_frame = 2 + num_channels

        self._ready_status = 0x05 << 8
        for i in range(self._num_channels):
            self._ready_status |= 1 << i

        self._data = [0.0] * num_channels

    def start(self) -> None:
        """
        Open the SPI port, reset the ADC, configure gain settings, and begin streaming ADC data.

        This method initializes the SPI communication, resets the device, sets the channel gains,
        transitions the device to continuous conversion mode, and clears any stale data.
        """
        self._spi.open(self._spi_bus, self._spi_chip)
        self._spi.max_speed_hz = self._max_speed_hz
        self._spi.mode = self._SPI_MODE

        self.reset()
        self._set_gain()
        self._set_device_state(1)
        self._clear_stale_data()

    def stop(self) -> None:
        """
        Stop streaming ADC data and close the SPI port.

        This method transitions the ADC to standby mode and closes the SPI connection.
        """
        self._set_device_state(0)
        self._spi.close()

    def reset(self) -> None:
        """
        Reset all ADC register values.

        Sends a reset command followed by blank words to initialize the device registers.
        """
        self._spi.xfer2(self._RESET_WORD + self._BLANK_WORD * (self._words_per_frame - 1))

    def update(self) -> None:
        """
        Read ADC data from the device.

        Waits until the ADC channels are ready to be read and then updates the internal data
        with the latest voltage values in millivolts.
        """
        while not self._ready_to_read():
            sleep(0.001)
        self._data = self._read_data_millivolts()

    def calibrate(self) -> None:
        """
        Perform offset and gain calibration on the ADC.

        This method performs an offset calibration (zeroing) and, if gain error corrections
        are provided, performs a gain calibration.
        """
        self._offset_calibration()
        if self._gain_error is not None:
            self._gain_calibration()

    def read_register(self, address: int) -> int:
        """
        Read the value stored in the register at the specified address.

        Args:
            address (int): The address of the register to read.

        Returns:
            int: The value stored at the register.
        """
        msg = (address << 7) | (self._RREG_PREFIX << 13)
        word = self._message_to_word(msg)
        rsp = self._spi_message(word)
        return int(rsp[0] << 8 | rsp[1])

    def write_register(self, address: int, reg_val: int) -> None:
        """
        Write a specific value to the register at the designated address.

        Args:
            address (int): The address of the register to write.
            reg_val (int): The value to be written to the register.
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
            bool: True if streaming; otherwise, False.
        """
        return self._streaming

    @property
    def gains(self) -> list[int]:
        """
        Get the gain settings (in logarithmic scale) for each ADC channel.

        Returns:
            list[int]: A list of gain values for each channel.
        """
        return self._gains

    @property
    def data(self) -> Any:
        """
        Get the latest ADC data.

        Returns:
            Any: The list of voltage readings (in millivolts) for each channel.
        """
        return self._data

    def _spi_message(self, message: list[int]) -> list[int]:
        """
        Send an SPI message to the ADS131M0x and read the response.

        Args:
            message (list[int]): The message to be sent, split into individual bytes.

        Returns:
            list[int]: The response from the device, representing the entire frame.
        """
        self._spi.xfer2(message)
        return list(self._spi.readbytes(self._BYTES_PER_WORD * self._words_per_frame))

    def _channel_enable(self, state: bool) -> None:
        """
        Enable or disable streaming on all channels.

        Args:
            state (bool): If True, enables the channel clocks; if False, disables them.
        """
        if state:
            self.write_register(self._CLOCK_REG, self._ENABLE_CHANNELS_CLOCK)
        else:
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
        for i in range(self._num_channels):
            self.write_register(self._CHANNEL_CFG_ADDRS[i], source)

    def _clear_stale_data(self) -> None:
        """
        Clear stale ADC values from the device.

        Reads and discards two frames of ADC data to ensure the next read returns current values.
        """
        for _ in range(2):
            self._read_data_millivolts()

    def _set_gain(self) -> None:
        """
        Set the programmable gain amplifier (PGA) gain for each ADC channel.

        This method configures the gain registers for both groups of channels and enables
        the channel clocks.
        """
        gains = self._gains + [0] * (self._MAX_CHANNELS - len(self._gains))
        self._channel_enable(False)
        gains_msg = gains[3] << 12 | gains[2] << 8 | gains[1] << 4 | gains[0]
        self.write_register(self._GAIN1_REG, gains_msg)
        gains_msg = gains[7] << 12 | gains[6] << 8 | gains[5] << 4 | gains[4]
        self.write_register(self._GAIN2_REG, gains_msg)
        self._channel_enable(True)

    def _offset_calibration(self) -> None:
        """
        Perform offset calibration of the ADC.

        This method centers the ADC data around the measured zero value by setting
        the appropriate offset registers for each channel.
        """
        self._set_voltage_source(1)
        self._clear_stale_data()
        num_data_points = 1000
        offsets = [[0] * self._num_channels for _ in range(num_data_points)]
        for i in range(num_data_points):
            offsets[i] = self._read_data_counts()
        offset_avg = [int(sum(values) / len(values)) for values in zip(*offsets)]
        for i in range(self._num_channels):
            self.write_register(self._OCAL_MSB_ADDRS[i], offset_avg[i] >> 8)
            self.write_register(self._OCAL_LSB_ADDRS[i], (offset_avg[i] << 8) & 0xFF00)
        self._set_voltage_source(0)

    def _gain_calibration(self) -> None:
        """
        Perform gain calibration of the ADC.

        Adjusts the actual gain to match the desired gain using user-calculated gain error
        values for each channel.
        """
        for i in range(self._num_channels):
            gain_correction = (1 + self._gain_error[i]) / self._GCAL_STEP_SIZE
            self.write_register(self._GCAL_MSB_ADDRS[i], int(gain_correction) >> 8)

    def _message_to_word(self, msg: int) -> list[int]:
        """
        Convert an integer message into a list of bytes (word) for SPI transmission.

        Args:
            msg (int): The integer message to convert.

        Returns:
            list[int]: A list containing the individual bytes of the message.
        """
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

    def _read_data_millivolts(self) -> list[float]:
        """
        Read and convert ADC data into millivolt values for each channel.

        Returns:
            list[float]: A list of voltage readings (in mV) for each ADC channel.
        """
        mV = [
            1000 * ((dat) / (2 ** (self._RESOLUTION - 1)) * self._voltage_reference) for dat in self._read_data_counts()
        ]
        return mV

    def _read_data_counts(self) -> list[int]:
        """
        Read raw ADC counts from the device.

        Reads the signed ADC values (in counts) for each channel. The counts are represented
        in two's complement format over a 24-bit resolution.

        Returns:
            list[int]: A list of signed ADC counts for each channel.
        """
        reply = self._spi.readbytes(self._BYTES_PER_WORD * self._words_per_frame)
        val = [0] * self._num_channels
        for byte in range(3, self._num_channels * 3 + 1, 3):
            index = int(byte / 3) - 1
            val[index] = self._twos_complement(
                (reply[byte] << 16) | (reply[byte + 1] << 8) | reply[byte + 2],
                self._RESOLUTION,
            )
        return val

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
        if (num >> (bits - 1)) != 0:  # if sign bit is set
            val = num - (1 << bits)  # compute negative value
        return val
