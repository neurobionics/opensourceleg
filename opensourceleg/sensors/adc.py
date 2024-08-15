# AFTER DONE, TEST WITH NEWLY IMAGED PI
import spidev
import math
from time import sleep
from base import ADCBase


class ADS131M0x(ADCBase):
    """Class used for communication with the ADS131M0x family of ADC chips.

    This class allows you to configure the ADS131M0x family of chips as well as
    read out the ADC values in units of mV.

    Attributes:
        spi_bus: An integer indicating which SPI bus the ADC is connected to.
        spi_chip: An integer indicating which CS signal the ADC is connected to.
        num_channels: An integer count of how many channels are present on the ADC.
        max_speed_hz: An integer specifying the maximum clock frequency of the SPI communication.
        channel_gains: A list of integers setting the gain of the programmable gain amplifier for all channels.
        voltage_reference: A float indicating the reference voltage used by the ADC.
        gain_error: A list of user-calculated integers used for correcting the gain of each channel for additional precision.

    Raises:
    """

    # TODO: List raised exceptions in docstrings at the appropriate location
    # TODO: Add type defs and return types
    # TODO: Check where it is okay to use magic numbers
    # TODO: Check if other sensors are calling calibrate outside the start method

    _MAX_CHANNELS = 8
    _BYTES_PER_WORD = 3
    _RESOLUTION = 24
    _SPI_MODE = 1

    _BLANK_WORD = [0x00, 0x00, 0x00]
    # SPI Commands
    _RESET_WORD = [0x00, 0x11, 0x00]
    _STANDBY_WORD = [0x00, 0x22, 0x00]
    _WAKEUP_WORD = [0x00, 0x33, 0x00]

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
    _OCAL_MSB_ADDRS = [0x0A, 0x0F, 0x14, 0x19, 0x1E, 0x23, 0x28, 0x2D]
    _OCAL_LSB_ADDRS = [0x0B, 0x10, 0x15, 0x1A, 0x1F, 0x24, 0x29, 0x2E]
    _GCAL_MSB_ADDRS = [0x0C, 0x11, 0x16, 0x1B, 0x20, 0x25, 0x2A, 0x2F]
    _GCAL_LSB_ADDRS = [0x0D, 0x12, 0x17, 0x1C, 0x21, 0x26, 0x2B, 0x30]
    _CHANNEL_CFG_ADDRS = [0x09, 0x0E, 0x13, 0x18, 0x1D, 0x22, 0x27, 0x2C]

    _GCAL_STEP_SIZE = 1.19e-7

    def __init__(
        self,
        spi_bus: int = 0,
        spi_chip: int = 0,
        num_channels: int = 6,
        max_speed_hz: int = 8192000,
        channel_gains: list = [32, 128] * 3,
        voltage_reference: float = 1.2,
        gain_error: list = None,
    ):
        """Initializes ADS131M0x class."""

        if len(channel_gains) != num_channels:
            raise ValueError(
                "Length of channel_gains does not equal number of channels"
            )
        if gain_error is not None and len(gain_error) != num_channels:
            raise ValueError(
                "Length of channel_gains does not equal number of channels"
            )

        self._spi_bus = spi_bus
        self._spi_chip = spi_chip
        self._num_channels = num_channels
        self._max_speed_hz = max_speed_hz
        self._gains = [0] * num_channels
        for i in range(0, num_channels):
            gain = int(math.log2(channel_gains[i]))
            if gain != math.log2(channel_gains[i]):
                raise Exception("Gain must be a power of 2 between 1 and 128")
            self._gains[i] = gain

        self._voltage_reference = voltage_reference
        self._gain_error = gain_error

        self._spi = spidev.SpiDev()
        self._streaming = False
        self._words_per_frame = 2 + num_channels

        self._ready_status = 0x01 << 8
        for i in range(self._num_channels):
            self._ready_status |= 1 << i

        self._data = [0] * num_channels

    def __repr__(self) -> str:
        return f"ADS131M0x"

    def start(self) -> None:
        """Opens SPI port, calibrates ADC, and begins streaming ADC data."""

        self._spi.open(self._spi_bus, self._spi_chip)
        self._spi.max_speed_hz = self._max_speed_hz
        self._spi.mode = self._SPI_MODE

        self.reset()
        self._set_gain()
        self._set_device_state(1)
        self.calibrate()
        self._clear_stale_data()

    def stop(self):
        """Stop streaming ADC data and close SPI port."""
        self._set_device_state(0)
        self._spi.close()

    def reset(self) -> None:
        """Resets all register values."""

        self._spi.xfer2(
            self._RESET_WORD + self._BLANK_WORD * (self._words_per_frame - 1)
        )

    def update(self) -> None:
        """Reads ADC data."""
        while not self._ready_to_read():
            sleep(0.001)

        self._data = self._read_data_millivolts()

    def read_register(self, address):
        """Read value at register located at specified address.

        Arg:
            address: Address of the register to be read.

        Returns:
            Value stored at register located at address.
        """
        msg = (address << 7) | (0b101 << 13)
        word = self._message_to_word(msg)
        rsp = self.spi_comm(word)
        return rsp[0] << 8 | rsp[1]

    def write_register(self, address, reg_val):
        """Writes specific value to register located at designated address.

        Args:
            address: Address of the register to be written.
            reg_val: Value to be written to register at address.
        """
        addr_msg = (address << 7) | (0b011 << 13)
        addr_bytes = self._message_to_word(addr_msg)
        reg_bytes = self._message_to_word(reg_val)
        self.spi_comm(addr_bytes + reg_bytes)

    def spi_comm(self, bytes):
        """Send SPI message to ADS131M0x.

        Arg:
            bytes: message to be sent to the ADS131M0x separated into bytes.

        Returns:
            The response to the message sent, including the entire frame following the response.
        """
        self._spi.xfer2(bytes)
        return self._spi.readbytes(self._BYTES_PER_WORD * self._words_per_frame)

    @property
    def is_streaming(self) -> bool:
        return self._streaming

    @property
    def gains(self) -> list:
        return self._gains

    @property
    def ch0(self):
        return self._data[0]

    @property
    def ch1(self):
        return self._data[1]

    @property
    def ch2(self):
        return self._data[2]

    @property
    def ch3(self):
        return self._data[3]

    @property
    def ch4(self):
        return self._data[4]

    @property
    def ch5(self):
        return self._data[5]

    @property
    def ch6(self):
        return self._data[6]

    @property
    def ch7(self):
        return self._data[7]

    def _channel_enable(self, state):
        if state == True:
            self.write_register(self._CLOCK_REG, self._ENABLE_CHANNELS_CLOCK)
        elif state == False:
            self.write_register(self._CLOCK_REG, self._DISABLE_CHANNELS_CLOCK)

    def _set_device_state(self, state: int) -> None:
        """Sets state of internal state machine

        Args:
            state: Device state
                0 -- Standby
                1 -- Continuous Conversion Mode
        """
        if state == 0:
            self._spi.xfer2(
                self._STANDBY_WORD + self._BLANK_WORD * (self._words_per_frame - 1)
            )
            self._streaming = False
        elif state == 1:
            self._spi.xfer2(
                self._WAKEUP_WORD + self._BLANK_WORD * (self._words_per_frame - 1)
            )
            self._streaming = True

    def _set_voltage_source(self, source):
        """Changes voltage source for ADC input.
        Args:
            source: Selects which voltage source to use for ADC data.
                0 -- external input
                1 -- shorts differential pairs for value of ~0
                2 -- positive internal test signal ((160mV / gain) * (Vref / 1.25))
                3 -- negative internal test signal ((-160mV / gain) * (Vref / 1.25))
        """
        for i in range(0, self._num_channels):
            self.write_register(self._CHANNEL_CFG_ADDRS[i], source)

    def _clear_stale_data(self):
        """Clears stale ADC values so the next read will be accurate."""
        for _ in range(2):
            self._read_data_millivolts()

    def _set_gain(self):
        """Set PGA gain for each channel of ADC."""

        gains = self._gains + [0] * (self._MAX_CHANNELS - len(self._gains))
        self._channel_enable(False)
        gains_msg = gains[3] << 12 | gains[2] << 8 | gains[1] << 4 | gains[0]
        self.write_register(self._GAIN1_REG, gains_msg)
        gains_msg = gains[7] << 12 | gains[6] << 8 | gains[5] << 4 | gains[4]
        self.write_register(self._GAIN2_REG, gains_msg)
        self._channel_enable(True)

    def calibrate(self):
        """Performs offset and gain calibration."""
        self._offset_calibration()
        if self._gain_error is not None:
            self._gain_calibration()

    def _offset_calibration(self):
        """Centers the ADC data around the measured zero value."""
        self._set_voltage_source(1)
        self._clear_stale_data()
        num_data_points = 1000
        offsets = [0] * num_data_points
        for i in range(num_data_points):
            offsets[i] = self._read_data_counts()
        offset_avg = [int(sum(values) / len(values)) for values in zip(*offsets)]
        for i in range(0, self._num_channels):
            self.write_register(self._OCAL_MSB_ADDRS[i], offset_avg[i] >> 8)
            self.write_register(self._OCAL_LSB_ADDRS[i], (offset_avg[i] << 8) & 0xFF00)
        self._set_voltage_source(0)

    def _gain_calibration(self):
        """Corrects actual gain to desired gain using user-calculated gain error for each channel."""
        for i in range(self._num_channels):
            gain_correction = (1 + self._gain_error[i]) / self._GCAL_STEP_SIZE
            self.write_register(self._GCAL_MSB_ADDRS[i], gain_correction >> 8)

    def _message_to_word(self, msg):
        """Separates message into bytes to be sent to ADC."""
        word = [0] * 3
        word[0] = (msg >> 8) & 0xFF
        word[1] = msg & 0xFF
        return word

    def _ready_to_read(self):
        """Returns true if all ADC channels are ready to be read."""
        reply = self.read_register(self._STATUS_REG)
        return reply == self._ready_status

    def _read_data_millivolts(self):
        mV = [
            1000 * ((dat) / (2 ** (self._RESOLUTION - 1)) * self._voltage_reference)
            for dat in self._read_data_counts()
        ]
        return mV

    def _read_data_counts(self):
        """Returns signed ADC value.

        Ranges from -2^23 --> 2^23
        """
        reply = self._spi.readbytes(self._BYTES_PER_WORD * self._words_per_frame)
        val = [0] * self._num_channels
        for byte in range(3, self._num_channels * 3 + 1, 3):
            val[int(((byte) / 3) - 1)] = self._twos_complement(
                ((reply[byte] << 16) | (reply[byte + 1] << 8) | reply[byte + 2]),
                self._RESOLUTION,
            )
        return val

    def _twos_complement(num, bits):
        val = num
        if (num >> (bits - 1)) != 0:  # if sign bit is set e.g.
            val = num - (1 << bits)  # compute negative value
        return val
