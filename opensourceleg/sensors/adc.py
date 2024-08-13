# AFTER DONE, TEST WITH NEWLY IMAGED PI
import spidev
import numpy as np
from time import sleep
from base import ADCBase


class ADS131M0x(ADCBase):
    _BYTES_PER_WORD = 3
    # SPI Commands
    _WAKEUP_WORD = [0x00, 0x33, 0x00]
    _RESET_WORD = [0x00, 0x11, 0x00]
    _STANDBY_WORD = [0x00, 0x22, 0x00]
    # SPI words for writing to different registers
    _WRITE_CLOCK_WORD = [0x61, 0x80, 0x00]
    _WRITE_GAIN1_WORD = [0x62, 0x00, 0x00]
    _WRITE_GAIN2_WORD = [0x62, 0x80, 0x00]
    _WRITE_MODE_WORD = [0x61, 0x00, 0x00]
    # Specific values to be written to registers
    _DISABLE_CHANNEL_WORD = [0x00, 0x0E, 0x00]
    _ENABLE_CHANNEL_WORD = [0xFF, 0x0E, 0x00]
    _MODE_SETUP_WORD = [0x01, 0x10, 0x00]

    # SPI words for writing to the most significant bytes of the offset calibration register
    _WRITE_MSB_OCAL_WORDS = [
        [0x65, 0x00, 0x00],
        [0x67, 0x80, 0x00],
        [0x6A, 0x00, 0x00],
        [0x6C, 0x80, 0x00],
        [0x6F, 0x00, 0x00],
        [0x71, 0x80, 0x00],
        [0x74, 0x00, 0x00],
        [0x76, 0x80, 0x00],
    ]
    # SPI words for writing to the least significant bytes of the offset calibration register
    _WRITE_LSB_OCAL_WORDS = [
        [0x65, 0x80, 0x00],
        [0x68, 0x00, 0x00],
        [0x6A, 0x80, 0x00],
        [0x6D, 0x00, 0x00],
        [0x6F, 0x80, 0x00],
        [0x72, 0x00, 0x00],
        [0x74, 0x80, 0x00],
        [0x77, 0x00, 0x00],
    ]
    # SPI words for writing to the most significant bytes of the gain calibration register
    _WRITE_MSB_GCAL_WORDS = [
        [0x66, 0x00, 0x00],
        [0x68, 0x80, 0x00],
        [0x6B, 0x00, 0x00],
        [0x6D, 0x80, 0x00],
        [0x70, 0x00, 0x00],
        [0x72, 0x80, 0x00],
        [0x75, 0x00, 0x00],
        [0x77, 0x80, 0x00],
    ]
    # SPI words for writing to the least significant bytes of the gain calibration register
    _WRITE_LSB_GCAL_WORDS = [
        [0x67, 0x00, 0x00],
        [0x69, 0x00, 0x00],
        [0x6B, 0x80, 0x00],
        [0x6E, 0x00, 0x00],
        [0x70, 0x80, 0x00],
        [0x73, 0x00, 0x00],
        [0x75, 0x80, 0x00],
        [0x78, 0x00, 0x00],
    ]
    # SPI words for writing to the configuration registers of each channel
    _WRITE_CFG_WORDS = [
        [0x64, 0x80, 0x00],
        [0x67, 0x00, 0x00],
        [0x69, 0x80, 0x00],
        [0x6C, 0x00, 0x00],
        [0x6E, 0x80, 0x00],
        [0x71, 0x00, 0x00],
        [0x73, 0x80, 0x00],
        [0x76, 0x00, 0x00],
    ]
    # Translate desired gain to register code for that gain
    _GAINS = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7}

    def __init__(self, num_channels=6, max_speed_hz=8192000, gain=[32, 128] * 3):
        self._num_channels = num_channels
        self._WORDS_PER_FRAME = 2 + num_channels
        self._spi = spidev.SpiDev()
        self._max_speed_hz = max_speed_hz
        self._data = [0] * num_channels
        self._ready_status = {6: 0x013F, 7: 0x017F, 8: 0x01FF}
        self._streaming = False
        self._gain = gain

    def __repr__(self) -> str:
        return f"ADS131M0x"

    """Opens SPI port and begins streaming ADC data"""

    def start(self) -> None:
        # opens /dev/spidev0.0
        self._spi.open(0, 0)
        self._streaming = True
        self._spi.max_speed_hz = self._max_speed_hz
        # SPI mode 1: CPOL = 0, CPHA = 1
        self._spi.mode = 0b01
        # Set programmable gain amplifier (PGA) gains for each channel
        self.set_gain(self._gain)
        # wakeup
        self.send_spi(self._WAKEUP_WORD)
        self._set_voltage_source(0)
        # set mode register for word length and set RESET to 0
        self.send_spi(self._WRITE_MODE_WORD + self._MODE_SETUP_WORD)
        self.calibrate()
        # self._set_voltage_source(self._source)
        self._clear_stale_data()

    """Stop streaming ADC data and close SPI port"""

    def stop(self):
        self.send_spi(self._STANDBY_WORD)
        self._spi.close()
        self._streaming = False

    """Reads newest ADC data"""

    def update(self) -> None:
        while not self._ready_to_read():
            sleep(0.0001)
        self._data = [(dat) / (2**23) * 1.2 for dat in self._read_data()]

    """Resets all register values"""

    def reset(self) -> None:
        self.send_spi(self._RESET_WORD)

    """Set default gain calibration and automatically calibrates the offset"""

    def calibrate(self):
        self.default_calibration()
        self._offset_calibration()

    """Writes gain calibration value of 1 and offset calibration value of 0"""

    def default_calibration(self) -> None:
        offset = 0x000000
        gain = 0x800000
        self._set_voltage_source(0)
        sleep(0.01)
        for i in range(0, self._num_channels):
            # offset calibration
            self.send_spi(
                self._WRITE_MSB_OCAL_WORDS[i]
                + [(offset >> 16) & 0xFF, (offset >> 8) & 0xFF, 0x00]
            )
            self.send_spi(self._WRITE_LSB_OCAL_WORDS[i] + [(offset) & 0xFF, 0x00, 0x00])
            # gain calibration
            self.send_spi(
                self._WRITE_MSB_GCAL_WORDS[i]
                + [(gain >> 16) & 0xFF, (gain >> 8) & 0xFF, 0x00]
            )
            self.send_spi(self._WRITE_LSB_GCAL_WORDS[i] + [(gain) & 0xFF, 0x00, 0x00])

    """Set PGA gain for each channel of ADC"""

    def set_gain(self, gain_array):  # Add parameters to set gains
        # disable all channels
        self.send_spi(self._WRITE_CLOCK_WORD + self._DISABLE_CHANNEL_WORD)
        # set gain to 32 for CH 0, 2, 4 and 128 for CH 1, 3, 5
        self.send_spi(
            self._WRITE_GAIN1_WORD
            + [
                self._GAINS[gain_array[3]] << 4 | self._GAINS[2],
                self._GAINS[1] << 4 | self._GAINS[0],
                0x00,
            ]
        )
        if self._num_channels > 6:
            self.send_spi(
                self._WRITE_GAIN2_WORD
                + [
                    self._GAINS[7] << 4 | self._GAINS[6],
                    self._GAINS[5] << 4 | self._GAINS[4],
                    0x00,
                ]
            )
        else:
            self.send_spi(
                self._WRITE_GAIN2_WORD
                + [0x00, self._GAINS[5] << 4 | self._GAINS[4], 0x00]
            )
        # enable all channels
        self.send_spi(self._WRITE_CLOCK_WORD + self._ENABLE_CHANNEL_WORD)

    """Send SPI message to ADS131M0x"""

    def send_spi(self, bytes):
        words_sent = np.int8(len(bytes) / 3)
        self._spi.xfer2(bytes)
        self._spi.readbytes(self._BYTES_PER_WORD * (self._WORDS_PER_FRAME - words_sent))

    """Returns hex values at register located at specified address
    
    Mainly used for debugging purposes
    """

    def read_register(self, address):
        msg = (address << 7) | (0b101 << 13)
        byte = [0] * 2
        byte[0] = (msg >> 8) & 0xFF
        byte[1] = msg & 0xFF
        self._spi.xfer2(byte + [0x00])
        rsp = self._spi.readbytes(24)
        return hex(rsp[0] << 8 | rsp[1])

    @property
    def is_streaming(self) -> bool:
        return self._streaming

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

    """Checks if all ADC channels are ready to be read"""

    def _ready_to_read(self):
        self._spi.xfer2([0x00, 0x00, 0x00])
        reply = self._spi.readbytes(24)
        return ((reply[0] << 8) | reply[1]) == self._ready_status[self._num_channels]

    """Centers the ADC data around the measured zero value"""

    def _offset_calibration(self):
        # set voltage source to ground
        self._set_voltage_source(1)
        sleep(0.01)
        self._clear_stale_data()
        offsets = [0] * 1000
        for i in range(1000):
            offsets[i] = self._read_data()
        offset_avg = [int(sum(values) / len(values)) for values in zip(*offsets)]
        print("zeros")
        print(offset_avg)
        for i in range(0, self._num_channels):
            self.send_spi(
                self._WRITE_MSB_OCAL_WORDS[i]
                + [(offset_avg[i] >> 16) & 0xFF, (offset_avg[i] >> 8) & 0xFF, 0x00]
            )
            self.send_spi(
                self._WRITE_LSB_OCAL_WORDS[i] + [(offset_avg[i]) & 0xFF, 0x00, 0x00]
            )
        self._set_voltage_source(0)

    """Clears stale ADC values so the next read will be accurate"""

    def _clear_stale_data(self):
        for _ in range(2):
            self.update()

    """Changes voltage source for ADC input.
    
    0 -- external input
    1 -- shorts differential pairs for value of 0
    2 -- positive internal test signal (~153.6mV)
    3 -- negative internal test signal (~-153.6mV)
    """

    def _set_voltage_source(self, source):
        for i in range(0, self._num_channels):
            self.send_spi(self._WRITE_CFG_WORDS[i] + [0x00, source, 0x00])

    """Returns signed ADC value
    
    Ranges from -2^23 --> 2^23
    """

    def _read_data(self):
        reply = self._spi.readbytes(self._BYTES_PER_WORD * self._WORDS_PER_FRAME)
        val = [0] * self._num_channels
        for byte in range(3, self._num_channels * 3 + 1, 3):
            val[int(((byte) / 3) - 1)] = twos_complement(
                ((reply[byte] << 16) | (reply[byte + 1] << 8) | reply[byte + 2]), 24
            )
        return val


def twos_complement(num, bits):
    val = num
    if (num >> (bits - 1)) != 0:  # if sign bit is set e.g.
        val = num - (1 << bits)  # compute negative value
    return val
