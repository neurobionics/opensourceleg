"""
Module for communicating with the ADS131M0x and ADS114S0x family of ADC chips.
"""

import math
from dataclasses import dataclass
from enum import Enum
from time import sleep
from typing import Any, Callable, ClassVar, Optional, cast

import numpy as np
from gpiozero import DigitalInputDevice

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import ADCBase


class ADS114S0x(ADCBase):
    """
    Class for communication with the ADS114S0x family of ADC chips.

    This class allows configuration of the ADS114S0x chips and reading ADC values in millivolts.
    """

    # Class attributes

    # Constants
    _NUM_REGISTERS = 18
    _ADS124S08_FCLK = 4096000  # Standard internal clock frequency
    _ADS114S08_BITRES = 16  # ADC resolution

    # Data lengths
    _DATA_LENGTH = 3  # Conversion data total bytes
    _COMMAND_LENGTH = 2  # Register read/write command length
    _STATUS_LENGTH = 1  # Status length in bytes
    _CRC_LENGTH = 1  # CRC length in bytes
    _RDATA_COMMAND_LENGTH = 1  # RDATA command length

    # Internal reference voltage
    _INT_VREF = 2.5

    # Timing delays
    _DELAY_4TCLK = 1  # microseconds
    _DELAY_4096TCLK = int(4096.0 * 1000000 / _ADS124S08_FCLK)
    _DELAY_2p2MS = int(0.0022 * 1000000)

    # SPI Commands
    _OPCODE_NOP = 0x00
    _OPCODE_WAKEUP = 0x02
    _OPCODE_POWERDOWN = 0x04
    _OPCODE_RESET = 0x06
    _OPCODE_START = 0x08
    _OPCODE_STOP = 0x0A
    _OPCODE_SYOCAL = 0x16
    _OPCODE_SYGCAL = 0x17
    _OPCODE_SFOCAL = 0x19
    _OPCODE_RDATA = 0x12
    _OPCODE_RREG = 0x20
    _OPCODE_WREG = 0x40
    _OPCODE_RWREG_MASK = 0x1F

    # Read mode enum
    class ReadMode(Enum):
        DIRECT = 0
        COMMAND = 1

    # Register addresses
    _REG_ADDR_ID = 0x00
    _REG_ADDR_STATUS = 0x01
    _REG_ADDR_INPMUX = 0x02
    _REG_ADDR_PGA = 0x03
    _REG_ADDR_DATARATE = 0x04
    _REG_ADDR_REF = 0x05
    _REG_ADDR_IDACMAG = 0x06
    _REG_ADDR_IDACMUX = 0x07
    _REG_ADDR_VBIAS = 0x08
    _REG_ADDR_SYS = 0x09
    # ADS114S08 calibration regs (16-bit each)
    _REG_ADDR_RESERVED0A = 0x0A
    _REG_ADDR_OFCAL0 = 0x0B
    _REG_ADDR_OFCAL1 = 0x0C
    _REG_ADDR_RESERVED0D = 0x0D
    _REG_ADDR_FSCAL0 = 0x0E
    _REG_ADDR_FSCAL1 = 0x0F
    _REG_ADDR_GPIODAT = 0x10
    _REG_ADDR_GPIOCON = 0x11

    # Register default values
    _ID_DEFAULT = 0x00
    _STATUS_DEFAULT = 0x80
    _INPMUX_DEFAULT = 0x01
    _PGA_DEFAULT = 0x00
    _DATARATE_DEFAULT = 0x14
    _REF_DEFAULT = 0x10
    _IDACMAG_DEFAULT = 0x00
    _IDACMUX_DEFAULT = 0xFF
    _VBIAS_DEFAULT = 0x00
    _SYS_DEFAULT = 0x10
    _OFCAL0_DEFAULT = 0x00
    _OFCAL1_DEFAULT = 0x00
    _FSCAL0_DEFAULT = 0x00
    _FSCAL1_DEFAULT = 0x40
    _RESERVED0A_DEFAULT = 0x00
    _RESERVED0D_DEFAULT = 0x00
    _GPIODAT_DEFAULT = 0x00
    _GPIOCON_DEFAULT = 0x00

    # Status register masks
    _ADS_nRDY_MASK = 0x40
    _ADS_FL_POR_MASK = 0x80

    # SYS register masks
    _ADS_SENDSTATUS_MASK = 0x01
    _ADS_CRC_MASK = 0x02

    # Input multiplexer settings
    _ADS_P_AIN0 = 0x00
    _ADS_P_AIN1 = 0x10
    _ADS_P_AIN2 = 0x20
    _ADS_P_AIN3 = 0x30
    _ADS_P_AIN4 = 0x40
    _ADS_P_AIN5 = 0x50
    _ADS_P_AIN6 = 0x60
    _ADS_P_AIN7 = 0x70
    _ADS_P_AIN8 = 0x80
    _ADS_P_AIN9 = 0x90
    _ADS_P_AIN10 = 0xA0
    _ADS_P_AIN11 = 0xB0
    _ADS_P_AINCOM = 0xC0

    _ADS_N_AIN0 = 0x00
    _ADS_N_AIN1 = 0x01
    _ADS_N_AIN2 = 0x02
    _ADS_N_AIN3 = 0x03
    _ADS_N_AIN4 = 0x04
    _ADS_N_AIN5 = 0x05
    _ADS_N_AIN6 = 0x06
    _ADS_N_AIN7 = 0x07
    _ADS_N_AIN8 = 0x08
    _ADS_N_AIN9 = 0x09
    _ADS_N_AIN10 = 0x0A
    _ADS_N_AIN11 = 0x0B
    _ADS_N_AINCOM = 0x0C

    # PGA settings
    _ADS_DELAY_14 = 0x00
    _ADS_DELAY_25 = 0x20
    _ADS_DELAY_64 = 0x40
    _ADS_DELAY_256 = 0x60
    _ADS_DELAY_1024 = 0x80
    _ADS_DELAY_2048 = 0xA0
    _ADS_DELAY_4096 = 0xC0
    _ADS_DELAY_1 = 0xE0

    _ADS_PGA_BYPASS = 0x00
    _ADS_PGA_ENABLED = 0x08

    _ADS_GAIN_1 = 0x00
    _ADS_GAIN_2 = 0x01
    _ADS_GAIN_4 = 0x02
    _ADS_GAIN_8 = 0x03
    _ADS_GAIN_16 = 0x04
    _ADS_GAIN_32 = 0x05
    _ADS_GAIN_64 = 0x06
    _ADS_GAIN_128 = 0x07
    _ADS_GAIN_MASK = 0x07

    # Data rate settings
    _ADS_GLOBALCHOP = 0x80
    _ADS_CLKSEL_EXT = 0x40
    _ADS_CONVMODE_SS = 0x20
    _ADS_CONVMODE_CONT = 0x00
    _ADS_FILTERTYPE_LL = 0x10

    _ADS_DR_2_5 = 0x00
    _ADS_DR_5 = 0x01
    _ADS_DR_10 = 0x02
    _ADS_DR_16 = 0x03
    _ADS_DR_20 = 0x04
    _ADS_DR_50 = 0x05
    _ADS_DR_60 = 0x06
    _ADS_DR_100 = 0x07
    _ADS_DR_200 = 0x08
    _ADS_DR_400 = 0x09
    _ADS_DR_800 = 0x0A
    _ADS_DR_1000 = 0x0B
    _ADS_DR_2000 = 0x0C
    _ADS_DR_4000 = 0x0D

    # Reference settings
    _ADS_FLAG_REF_DISABLE = 0x00
    _ADS_FLAG_REF_EN_L0 = 0x40
    _ADS_FLAG_REF_EN_BOTH = 0x80
    _ADS_FLAG_REF_EN_10M = 0xC0
    _ADS_REFP_BYP_DISABLE = 0x20
    _ADS_REFP_BYP_ENABLE = 0x00
    _ADS_REFN_BYP_DISABLE = 0x10
    _ADS_REFN_BYP_ENABLE = 0x00
    _ADS_REFSEL_P0 = 0x00
    _ADS_REFSEL_P1 = 0x04
    _ADS_REFSEL_INT = 0x08
    _ADS_REFINT_OFF = 0x00
    _ADS_REFINT_ON_PDWN = 0x01
    _ADS_REFINT_ON_ALWAYS = 0x02

    # IDAC settings
    _ADS_FLAG_RAIL_ENABLE = 0x80
    _ADS_FLAG_RAIL_DISABLE = 0x00
    _ADS_PSW_OPEN = 0x00
    _ADS_PSW_CLOSED = 0x40
    _ADS_IDACMAG_OFF = 0x00
    _ADS_IDACMAG_10 = 0x01
    _ADS_IDACMAG_50 = 0x02
    _ADS_IDACMAG_100 = 0x03
    _ADS_IDACMAG_250 = 0x04
    _ADS_IDACMAG_500 = 0x05
    _ADS_IDACMAG_750 = 0x06
    _ADS_IDACMAG_1000 = 0x07
    _ADS_IDACMAG_1500 = 0x08
    _ADS_IDACMAG_2000 = 0x09

    # IDAC multiplexer settings
    _ADS_IDAC2_A0 = 0x00
    _ADS_IDAC2_A1 = 0x10
    _ADS_IDAC2_A2 = 0x20
    _ADS_IDAC2_A3 = 0x30
    _ADS_IDAC2_A4 = 0x40
    _ADS_IDAC2_A5 = 0x50
    _ADS_IDAC2_A6 = 0x60
    _ADS_IDAC2_A7 = 0x70
    _ADS_IDAC2_A8 = 0x80
    _ADS_IDAC2_A9 = 0x90
    _ADS_IDAC2_A10 = 0xA0
    _ADS_IDAC2_A11 = 0xB0
    _ADS_IDAC2_AINCOM = 0xC0
    _ADS_IDAC2_OFF = 0xF0

    _ADS_IDAC1_A0 = 0x00
    _ADS_IDAC1_A1 = 0x01
    _ADS_IDAC1_A2 = 0x02
    _ADS_IDAC1_A3 = 0x03
    _ADS_IDAC1_A4 = 0x04
    _ADS_IDAC1_A5 = 0x05
    _ADS_IDAC1_A6 = 0x06
    _ADS_IDAC1_A7 = 0x07
    _ADS_IDAC1_A8 = 0x08
    _ADS_IDAC1_A9 = 0x09
    _ADS_IDAC1_A10 = 0x0A
    _ADS_IDAC1_A11 = 0x0B
    _ADS_IDAC1_AINCOM = 0x0C
    _ADS_IDAC1_OFF = 0x0F

    # VBIAS settings
    _ADS_VBIAS_LVL_DIV2 = 0x00
    _ADS_VBIAS_LVL_DIV12 = 0x80
    _ADS_VB_AINC = 0x40
    _ADS_VB_AIN5 = 0x20
    _ADS_VB_AIN4 = 0x10
    _ADS_VB_AIN3 = 0x08
    _ADS_VB_AIN2 = 0x04
    _ADS_VB_AIN1 = 0x02
    _ADS_VB_AIN0 = 0x01

    # System monitor settings
    _ADS_SYS_MON_OFF = 0x00
    _ADS_SYS_MON_SHORT = 0x20
    _ADS_SYS_MON_TEMP = 0x40
    _ADS_SYS_MON_ADIV4 = 0x60
    _ADS_SYS_MON_DDIV4 = 0x80
    _ADS_SYS_MON_BCS_2 = 0xA0
    _ADS_SYS_MON_BCS_1 = 0xC0
    _ADS_SYS_MON_BCS_10 = 0xE0
    _ADS_CALSAMPLE_1 = 0x00
    _ADS_CALSAMPLE_4 = 0x08
    _ADS_CALSAMPLE_8 = 0x10
    _ADS_CALSAMPLE_16 = 0x18
    _ADS_TIMEOUT_DISABLE = 0x00
    _ADS_TIMEOUT_ENABLE = 0x04
    _ADS_CRC_DISABLE = 0x00
    _ADS_CRC_ENABLE = 0x02
    _ADS_SENDSTATUS_DISABLE = 0x00
    _ADS_SENDSTATUS_ENABLE = 0x01

    # SPI Configuration
    _SPI_SPEED = 2000000  # 2 MHz
    _SPI_BUS = 1
    _SPI_DEVICE = 0

    # Constants
    _HIGH = True
    _LOW = False

    # Internal variables
    _spi = None

    # CRC Configuration
    _CRC_LOOKUP = True  # Use lookup table method
    _CRC_INITIAL_SEED = 0x00
    _CRC_POLYNOMIAL = 0x07  # CRC-8-ATM (HEC) polynomial: X^8 + X^2 + X + 1

    # Internal variables
    _initialized = False
    _crc_lookup_table = [0] * 256

    _MAX_CHANNELS = 12

    def __init__(
        self,
        tag: str = "ADS114S08",
        spi_bus: int = 0,
        spi_cs: int = 0,
        data_rate: int = 400,
        pga_gain: int = 1,
        voltage_reference: float = _INT_VREF,
        drdy: int = 16,
        offline: bool = False,
    ) -> None:
        """
        Initialize the ADS114S0x instance.

        Args:
            tag (str): Identifier for the ADC instance. Default is "ADS114S0x".
            spi_bus (int): SPI bus number. Default is 0.
            spi_cs (int): SPI chip select line. Default is 0.
            data_rate (int): Sampling rate in Hz. Default is 500 Hz.
            pga_gain (int): Default is 1.
            voltage_reference (float): Reference voltage in volts. Default is 2.5 V.
            drdy (int): GPIO pin number for the data-ready signal. Defaults to 16.
            offline (bool): If True, the ADC operates in offline mode. Default is False.

        Raises:
            ImportError: If spidev is not installed and offline is False.
        """

        try:
            import spidev

            self._spi = spidev.SpiDev()
        except ImportError:
            LOGGER.error("spidev is not installed. Please install it to use this module.")

            if not offline:
                exit(1)

        super().__init__(tag=tag, offline=offline)

        self._spi_bus = spi_bus
        self._spi_cs = spi_cs
        self._pga_gain = pga_gain
        self._voltage_reference = voltage_reference
        self._streaming = False
        self._data_rate = data_rate
        self._drdy = DigitalInputDevice(drdy, pull_up=False)
        self._channels: dict[str, ChannelConfig] = {}
        self._data: Optional[list[float]] = None
        self._register_map = [0] * self._NUM_REGISTERS
        LOGGER.info(f"ADC initialized with tag: {self._tag}")

    def __repr__(self) -> str:
        return "ADS114S0x"

    # Functions Required by SensorBase and dependencies
    def start(self) -> None:
        """
        Start the ADC by opening the SPI port, resetting the device, and confirming reading and writing
        """
        LOGGER.info("Starting ADC...")
        self.init_spi()

        self.delay_us(self._DELAY_2p2MS)
        self.reset()
        self.restore_register_defaults()

        # Configure initial device register settings
        self.write_single_register(self._REG_ADDR_STATUS, 0x00)  # Reset POR event
        self._set_device_state(1)
        LOGGER.info("ADC started successfully.")

    def _set_device_state(self, state: int) -> None:
        """
        Set the internal state of the ADC device.

        Args:
            state (int): The desired state:
                0 -- Standby mode.
                1 -- Continuous Conversion Mode.
        """
        if state == 0:
            self._streaming = False
        elif state == 1:
            self._streaming = True

    def stop(self) -> None:
        """
        Stop the ADC by transitioning to standby mode and closing the SPI port.
        """
        LOGGER.info("Stopping ADC...")
        self.send_stop()
        self.cleanup()
        LOGGER.info("ADC stopped successfully.")

    def update(self) -> None:
        """
        Update the ADC data by reading the latest voltage values in millivolts.
        Attempts to read a maximum of 1000 times before throwing an error.

        Raises:
            RuntimeError: If the ADC does not become ready within 1000 attempts.
        """

        max_attempts = 1000
        attempts = 0

        while self._ready_to_read() is False:
            sleep(0.001)
            attempts += 1
            if attempts > max_attempts:
                raise RuntimeError(
                    "Couldn't connect to the ADC, please ensure that the device is connected and powered on."
                )

        self._data = self._read_data_millivolts()

    def _ready_to_read(self) -> bool:
        """
        Check if all ADC channels are ready for a new data read.

        Returns:
            bool: True if the status register indicates readiness; otherwise, False.
        """

        reply = self.read_single_register(address=self._REG_ADDR_STATUS)

        return not reply & self._ADS_nRDY_MASK

    def _read_data_millivolts(self) -> list[float] | None:
        """
        Read all configured channels and return their values in millivolts.

        Returns:
            list[float] | None: Millivolt readings for each configured channel,
                or None if no channels have been configured.
        """
        if not self._channels:
            LOGGER.info("No channels have been configured for reading. Use ChannelConfig.")
            return None

        row: list[float] = []
        for ch in self._channels.values():
            self.set_mux_single_ended(ch.ain_pos_code)
            self.discard_settling_reads(timeout_ms=1000)
            self.start_conversions()
            code16, _ = self.wait_and_read_code16()
            volts = self.code16_to_volts(code16)
            millivolts = volts * 1000

            if ch.postprocess is not None:
                millivolts = ch.postprocess(millivolts)

            row += [millivolts]

        return row

    # Properties required by SensorBase
    @property
    def is_streaming(self) -> bool:
        """
        Check if the ADC is currently streaming data.

        Returns:
            bool: True if streaming, False otherwise.
        """
        return self._streaming

    @property
    def data(self) -> list[float] | None:
        """
        Get the latest ADC data in millivolts.

        Returns:
            np.ndarray: Array of voltage readings for each channel.
        """
        return self._data

    # Functions replacing ADCBase functions
    def reset(self) -> None:
        """
        Sends RESET command through SPI
        """
        self.send_command(self._OPCODE_RESET)

    # Functions transferred from VSO-CODEBASE-DEV repo: ads114s08.py
    def get_register_value(self, address: int) -> int:
        """
        Getter function to access the register map array.

        Args:
            address: The 8-bit register address

        Returns:
            The 8-bit register value
        """
        if address >= self._NUM_REGISTERS:
            raise ValueError("Register address out of range")
        return self._register_map[address]

    def is_sendstat_set(self) -> bool:
        """Check if SENDSTAT bit is set in SYS register."""
        return bool(self.get_register_value(self._REG_ADDR_SYS) & self._ADS_SENDSTATUS_MASK)

    def is_crc_set(self) -> bool:
        """Check if CRC bit is set in SYS register."""
        return bool(self.get_register_value(self._REG_ADDR_SYS) & self._ADS_CRC_MASK)

    def read_single_register(self, address: int) -> int:
        """
        Reads contents of a single register at the specified address

        Args:
            address: Address of the register to be read

        Returns:
            8-bit register contents
        """
        if address >= self._NUM_REGISTERS:
            raise ValueError("Register address out of range")

        # Build TX array
        data_tx = [self._OPCODE_RREG | (address & self._OPCODE_RWREG_MASK), 0, 0]

        data_rx = self.spi_send_receive_arrays(data_tx, self._COMMAND_LENGTH + 1)

        # Update register array and return result
        self._register_map[address] = data_rx[self._COMMAND_LENGTH]
        return data_rx[self._COMMAND_LENGTH]

    def read_multiple_registers(self, start_address: int = 0x00, count: int = 17) -> None:
        """
        Reads a group of registers starting at the specified address
        Use get_register_value() to retrieve the read values

        Args:
            start_address: Register address to start reading (HEX)
            count: Number of registers to read
        """
        if start_address + count > self._NUM_REGISTERS:
            raise ValueError("Register address(es) out of range")

        # Build TX array
        data_tx = [self._OPCODE_RREG | (start_address & self._OPCODE_RWREG_MASK), count - 1]
        data_tx.extend([0] * count)

        data_rx = self.spi_send_receive_arrays(data_tx, self._COMMAND_LENGTH + count)

        # Store received register data
        for i in range(count):
            self._register_map[i + start_address] = data_rx[self._COMMAND_LENGTH + i]

    def write_single_register(self, address: int, data: int) -> None:
        """
        Write data to a single register at the specified address

        Args:
            address: Register address to write
            data: 8-bit data to write
        """
        if address >= self._NUM_REGISTERS:
            raise ValueError("Register address out of range")

        # Build TX array
        data_tx = [self._OPCODE_WREG | (address & self._OPCODE_RWREG_MASK), 0, data & 0xFF]

        self.spi_send_receive_arrays(data_tx, self._COMMAND_LENGTH + 1)

        # Update register array
        self._register_map[address] = data & 0xFF

    def write_multiple_registers(self, start_address: int, count: int, reg_data: list[int]) -> None:
        """
        Write data to a group of registers

        Args:
            start_address: Register address to start writing
            count: Number of registers to write
            reg_data: List of data to write (element zero is data for starting address)
        """
        if start_address + count > self._NUM_REGISTERS:
            raise ValueError("Register address(es) out of range")
        if reg_data is None:
            raise ValueError("reg_data cannot be None")

        # Build TX array
        data_tx = [self._OPCODE_WREG | (start_address & self._OPCODE_RWREG_MASK), count - 1]

        for i in range(start_address, start_address + count):
            data_tx.append(reg_data[i - start_address] & 0xFF)
            self._register_map[i] = reg_data[i - start_address] & 0xFF

        self.spi_send_receive_arrays(data_tx, self._COMMAND_LENGTH + count)

    def send_command(self, op_code: int) -> None:
        """
        Sends the specified SPI command to the ADC

        Args:
            op_code: SPI command byte
        """
        if op_code == self._OPCODE_RREG:
            raise ValueError("Use read_single_register() or read_multiple_registers()")
        if op_code == self._OPCODE_WREG:
            raise ValueError("Use write_single_register() or write_multiple_registers()")

        self.spi_send_receive_byte(op_code)

        # Check for RESET command
        if op_code == self._OPCODE_RESET:
            self.delay_us(self._DELAY_4096TCLK)  # Must wait 4096 tCLK after reset
            self.restore_register_defaults()

    def start_conversions(self) -> None:
        """
        Wakes the device from power-down and starts continuous conversions
        """
        # Wakeup device if in POWERDOWN
        self.send_wakeup()

        # Begin continuous conversions
        # If using START pin control, uncomment this:
        # hal.set_start(hal.HIGH)
        # Otherwise use SPI command:
        self.send_start()

    def read_converted_data(self, mode: ReadMode = ReadMode.DIRECT) -> tuple[int, Optional[int]]:
        """
        Sends the read command and retrieves STATUS (if enabled) and data
        Call this function after /DRDY goes low

        Args:
            mode: Direct or Command read mode

        Returns:
            Tuple of (32-bit sign-extended conversion result, status byte or None)
        """
        # Determine byte length and data position
        status_byte_enabled = self.is_sendstat_set()
        crc_enabled = self.is_crc_set()

        byte_options = (status_byte_enabled << 1) | crc_enabled

        if byte_options == 0:  # No STATUS and no CRC
            byte_length = self._DATA_LENGTH
            data_position = 0
        elif byte_options == 1:  # No STATUS and CRC
            byte_length = self._DATA_LENGTH + self._CRC_LENGTH
            data_position = 0
        elif byte_options == 2:  # STATUS and no CRC
            byte_length = self._STATUS_LENGTH + self._DATA_LENGTH
            data_position = 1
        else:  # STATUS and CRC
            byte_length = self._STATUS_LENGTH + self._DATA_LENGTH + self._CRC_LENGTH
            data_position = 1

        # Build TX array
        data_tx = [0] * (self._RDATA_COMMAND_LENGTH + byte_length)

        if mode == self.ReadMode.COMMAND:
            data_tx[0] = self._OPCODE_RDATA
            byte_length += 1
            data_position += 1

        data_rx = self.spi_send_receive_arrays(data_tx, byte_length)

        # Parse status byte if enabled
        status = None
        if status_byte_enabled:
            status = data_rx[data_position - 1]

        # Verify CRC if enabled
        if crc_enabled:
            if status_byte_enabled:
                data = [
                    data_rx[data_position - 1],  # status
                    data_rx[data_position],  # msb
                    data_rx[data_position + 1],  # mid
                    data_rx[data_position + 2],  # lsb
                    data_rx[data_position + 3],  # crc
                ]
                error = bool(self.get_crc(data, 5))
            else:
                data = [
                    data_rx[data_position],  # msb
                    data_rx[data_position + 1],  # mid
                    data_rx[data_position + 2],  # lsb
                    data_rx[data_position + 3],  # crc
                ]
                error = bool(self.get_crc(data, 4))

            if error:
                raise ValueError("CRC error in converted data")

        # --- ADS114S08: 3 data bytes are returned, but the ADC result is 16-bit.
        # Treat the 3 bytes as a signed 24-bit container, then shift down to 16-bit.
        msb = data_rx[data_position]
        mid = data_rx[data_position + 1]
        lsb = data_rx[data_position + 2]  # typically padding / low byte in this framing

        raw24 = (msb << 16) | (mid << 8) | lsb
        if msb & 0x80:  # sign bit of the 24-bit container
            raw24 -= 1 << 24

        code16 = raw24 >> 8  # signed 16-bit conversion code (-32768..32767)

        return (code16, status)

    def restore_register_defaults(self) -> None:
        """
        Updates the register_map array to its default values
        Should be called after powering up or resetting the device
        """
        self._register_map[self._REG_ADDR_ID] = self._ID_DEFAULT
        self._register_map[self._REG_ADDR_STATUS] = self._STATUS_DEFAULT
        self._register_map[self._REG_ADDR_INPMUX] = self._INPMUX_DEFAULT
        self._register_map[self._REG_ADDR_PGA] = self._PGA_DEFAULT
        self._register_map[self._REG_ADDR_DATARATE] = self._DATARATE_DEFAULT
        self._register_map[self._REG_ADDR_REF] = self._REF_DEFAULT
        self._register_map[self._REG_ADDR_IDACMAG] = self._IDACMAG_DEFAULT
        self._register_map[self._REG_ADDR_IDACMUX] = self._IDACMUX_DEFAULT
        self._register_map[self._REG_ADDR_VBIAS] = self._VBIAS_DEFAULT
        self._register_map[self._REG_ADDR_SYS] = self._SYS_DEFAULT
        self._register_map[self._REG_ADDR_RESERVED0A] = self._RESERVED0A_DEFAULT
        self._register_map[self._REG_ADDR_OFCAL0] = self._OFCAL0_DEFAULT
        self._register_map[self._REG_ADDR_OFCAL1] = self._OFCAL1_DEFAULT
        self._register_map[self._REG_ADDR_RESERVED0D] = self._RESERVED0D_DEFAULT
        self._register_map[self._REG_ADDR_FSCAL0] = self._FSCAL0_DEFAULT
        self._register_map[self._REG_ADDR_FSCAL1] = self._FSCAL1_DEFAULT
        self._register_map[self._REG_ADDR_GPIODAT] = self._GPIODAT_DEFAULT
        self._register_map[self._REG_ADDR_GPIOCON] = self._GPIOCON_DEFAULT

    # Functions transferred from VSO-CODEBASE-DEV repo: hal.py
    def init_spi(self) -> None:
        """
        Configures the Raspberry Pi's SPI peripheral for interfacing with the ADC

        Raises:
            RuntimeError: If the SPI device is not initialized.

        Note: ADS124S08 operates in SPI mode 1 (CPOL = 0, CPHA = 1)
        """
        if self._spi is None:
            raise RuntimeError("SPI device is not initialized. Ensure spidev is installed.")
        self._spi.open(self._spi_bus, self._spi_cs)
        self._spi.max_speed_hz = self._SPI_SPEED
        self._spi.mode = 0b01  # SPI Mode 1 (CPOL=0, CPHA=1)
        self._spi.bits_per_word = 8

    def cleanup(self) -> None:
        """
        Cleanup SPI resources
        """
        if self._spi is not None:
            self._spi.close()
            self._spi = None

    def delay_us(self, delay_time_us: int) -> None:
        """
        Provides a timing delay with microsecond resolution

        Args:
            delay_time_us: Number of microseconds to delay
        """
        sleep(delay_time_us / 1000000.0)

    def wait_for_drdy_htol(self, timeout_ms: int) -> bool:
        """
        Waits for conversion to complete by sleeping one conversion period.

        In single-shot mode with the low-latency filter, each conversion takes 1/data_rate seconds.
        A 1.5x multiplier provides a safe margin.

        Args:
            timeout_ms: Unused; kept for API compatibility.

        Returns:
            Always True.
        """
        sleep(1.5 / self._data_rate)
        return True

    def send_start(self) -> None:
        """
        Sends START command through SPI
        """
        self.send_command(self._OPCODE_START)

    def send_stop(self) -> None:
        """
        Sends STOP command through SPI
        """
        self.send_command(self._OPCODE_STOP)

    def send_wakeup(self) -> None:
        """
        Sends WAKEUP command through SPI
        """
        self.send_command(self._OPCODE_WAKEUP)

    def send_powerdown(self) -> None:
        """
        Sends POWERDOWN command through SPI
        """
        self.send_command(self._OPCODE_POWERDOWN)

    def spi_send_receive_arrays(self, data_tx: list[int], byte_length: int) -> list[int]:
        """
        Sends SPI commands to ADC and returns response

        Args:
            data_tx: List of SPI data to send on MOSI
            byte_length: Number of bytes to send/receive on SPI

        Returns:
            List of received bytes from MISO

        Raises:
            RuntimeError: If the SPI device is not initialized.
        """
        if self._spi is None:
            raise RuntimeError("SPI device is not initialized.")

        # Ensure data_tx has the correct length
        tx_data = data_tx[:byte_length]

        rx_data = self._spi.xfer2(tx_data)

        return cast(list[int], rx_data)

    def spi_send_receive_byte(self, data_tx: int) -> int:
        """
        Sends a single byte to ADC and returns response

        Args:
            data_tx: Byte to send on MOSI

        Returns:
            Received byte from MISO

        Raises:
            RuntimeError: If the SPI device is not initialized.
        """
        if self._spi is None:
            raise RuntimeError("SPI device is not initialized.")

        rx_data = self._spi.xfer2([data_tx & 0xFF])

        return cast(int, rx_data[0])

    # Functions transferred from VSO-CODEBASE-DEV repo crc.py
    def init_crc(self) -> None:
        """
        Initializes CRC module and creates lookup table (if using lookup method)
        """
        if self._CRC_LOOKUP:
            self._init_table()
            self._initialized = True

    def get_crc(self, data_bytes: list[int], number_bytes: int) -> int:
        """
        Performs CRC lookup or calculation

        Args:
            data_bytes: List of data bytes to process
            number_bytes: Number of bytes in array to process

        Returns:
            CRC value of the calculation

        Example:
            # To calculate the CRC of a 3-byte message:
            crc = get_crc(data, 3)

            # To test a 4-byte message with a CRC byte:
            error = bool(get_crc(data, 4))
        """
        if self._CRC_LOOKUP:
            if not self._initialized:
                self._init_table()
            return self._lookup_crc(data_bytes, number_bytes)
        else:
            return self._calculate_crc(data_bytes, number_bytes)

    def _init_table(self) -> None:
        """
        Creates lookup table in memory using byte wide computation in a 256 element array
        """
        for i in range(256):
            value = i & 0xFF
            self._crc_lookup_table[i] = self._calculate_crc([value], 1)

    def _lookup_crc(self, data_bytes: list[int], number_bytes: int) -> int:
        """
        Performs CRC lookup operation in byte increments

        Args:
            data_bytes: List of data bytes (little endian)
            number_bytes: Number of bytes in array to process

        Returns:
            CRC value of the calculation
        """
        crc = self._CRC_INITIAL_SEED & 0xFF

        for i in range(number_bytes):
            crc = self._crc_lookup_table[crc ^ (data_bytes[i] & 0xFF)]

        return crc & 0xFF

    def _calculate_crc(self, data_bytes: list[int], number_bytes: int) -> int:
        """
        Calculates the CRC for the selected CRC polynomial

        Args:
            data_bytes: List of data bytes
            number_bytes: Number of bytes to be used in CRC calculation

        Returns:
            CRC value of the calculation
        """
        crc = self._CRC_INITIAL_SEED & 0xFF

        # Loop through all bytes in the data_bytes array
        for byte_index in range(number_bytes):
            # Point to most significant bit
            bit_index = 0x80

            # Loop through all bits in the current byte
            while bit_index > 0:
                # Check MSB's of data and crc
                data_msb = bool(data_bytes[byte_index] & bit_index)
                crc_msb = bool(crc & 0x80)

                # Update crc register
                crc = (crc << 1) & 0xFF
                if data_msb ^ crc_msb:
                    crc ^= self._CRC_POLYNOMIAL

                # Shift MSb pointer to the next data bit
                bit_index >>= 1

        return crc & 0xFF

    # Functions transferred from VSO_CODEBASE_DEV repo adc_common.py
    def set_mux_single_ended(self, pos_code: int, neg_code: Optional[int] = None) -> None:
        """
        Configure the input multiplexer for a single-ended measurement.

        INPMUX: upper nibble = positive input, lower nibble = negative input.
        For example, to read AIN3 against AINCOM, pass _ADS_P_AIN3 as pos_code.

        Args:
            pos_code (int): Positive input channel code (e.g. _ADS_P_AIN3).
            neg_code (int, optional): Negative input channel code. Defaults to _ADS_N_AINCOM.
        """
        if neg_code is None:
            neg_code = self._ADS_N_AINCOM

        inpmux = (pos_code & 0xF0) | (neg_code & 0x0F)
        self.write_single_register(self._REG_ADDR_INPMUX, inpmux)

    def wait_and_read_code16(self, *, timeout_ms: int = 200) -> tuple[int, Optional[int]]:
        """
        Wait for DRDY falling edge then read conversion result.

        Args:
            timeout_ms (int): Timeout in milliseconds to wait for DRDY.
                Defaults to 200.

        Returns:
            tuple[int, int | None]: Signed 16-bit conversion code and
                optional status byte.

        Raises:
            TimeoutError: If DRDY does not assert within timeout_ms.
        """
        ok = self.wait_for_drdy_htol(timeout_ms)
        if not ok:
            raise TimeoutError(f"Timeout waiting for DRDY (>{timeout_ms} ms)")

        code16, status = self.read_converted_data(mode=self.ReadMode.DIRECT)
        return code16, status

    def discard_settling_reads(self, n: int = 1, timeout_ms: int = 200) -> None:
        """
        Discard a few reads after changing MUX to reduce charge-injection artifacts.

        Args:
            n (int): Number of reads to discard. Defaults to 1.
            timeout_ms (int): Timeout in milliseconds to wait for each DRDY. Defaults to 200.
        """
        for _ in range(max(0, n)):
            self.send_start()
            _ = self.wait_and_read_code16(timeout_ms=timeout_ms)

    def code16_to_volts(self, code16: int) -> float:
        """
        Convert a signed 16-bit ADC code to a voltage in volts.

        Uses the full-scale formula: Vin = code * (Vref / gain) / 32768.

        Args:
            code16 (int): Signed 16-bit ADC code in the range [-32768, 32767].

        Returns:
            float: Corresponding input voltage in volts.
        """
        return (code16 * (self._voltage_reference / float(self._pga_gain))) / 32768.0

    # Functions transferred from VSO_CODEBASE_DEV repo multi_channel_read.py
    def adc_configure_common(
        self,
        single_shot: bool = True,
        filter_low_latency: bool = True,
        vref_select_reg: int = _REF_DEFAULT,
        enable_crc: bool = False,
        enable_status_byte: bool = False,
    ) -> None:
        """
        Configure shared ADC registers (PGA, DATARATE, REF, SYS).

        Args:
            single_shot (bool): If True, use single-shot conversion mode. Defaults to True.
            filter_low_latency (bool): If True, enable the low-latency digital filter. Defaults to True.
            vref_select_reg (int): REF register value selecting the voltage reference source. Defaults to _REF_DEFAULT.
            enable_crc (bool): If True, enable CRC error checking on data reads. Defaults to False.
            enable_status_byte (bool): If True, prepend a STATUS byte to each conversion result. Defaults to False.

        Raises:
            ValueError: If pga_gain or data_rate is not a supported value.
        """
        # --- PGA ---
        if self._pga_gain == 1:
            pga_reg = self._ADS_PGA_BYPASS | self._ADS_GAIN_1
        else:
            gain_to_code = {
                1: self._ADS_GAIN_1,
                2: self._ADS_GAIN_2,
                4: self._ADS_GAIN_4,
                8: self._ADS_GAIN_8,
                16: self._ADS_GAIN_16,
                32: self._ADS_GAIN_32,
                64: self._ADS_GAIN_64,
                128: self._ADS_GAIN_128,
            }
            if self._pga_gain not in gain_to_code:
                raise ValueError(f"Unsupported gain={self._pga_gain}. Choose from {sorted(gain_to_code.keys())}")
            pga_reg = self._ADS_PGA_ENABLED | gain_to_code[self._pga_gain]
        self.write_single_register(self._REG_ADDR_PGA, pga_reg)

        # --- DATARATE ---
        rate_to_code = {
            2.5: self._ADS_DR_2_5,
            5: self._ADS_DR_5,
            10: self._ADS_DR_10,
            16: self._ADS_DR_16,
            20: self._ADS_DR_20,
            50: self._ADS_DR_50,
            60: self._ADS_DR_60,
            100: self._ADS_DR_100,
            200: self._ADS_DR_200,
            400: self._ADS_DR_400,
            800: self._ADS_DR_800,
            1000: self._ADS_DR_1000,
            2000: self._ADS_DR_2000,
            4000: self._ADS_DR_4000,
        }
        if self._data_rate not in rate_to_code:
            raise ValueError(f"Unsupported frequency={self._data_rate}. Choose from {sorted(rate_to_code.keys())}")
        data_rate_code = rate_to_code[self._data_rate]
        convmode = self._ADS_CONVMODE_SS if single_shot else self._ADS_CONVMODE_CONT
        ftype = self._ADS_FILTERTYPE_LL if filter_low_latency else 0x00
        datarate_reg = convmode | ftype | (data_rate_code & 0x0F)
        self.write_single_register(self._REG_ADDR_DATARATE, datarate_reg)

        # REF
        self.write_single_register(self._REG_ADDR_REF, vref_select_reg)

        # SYS (CRC + SENDSTAT)
        sys_reg = self._ADS_SYS_MON_OFF
        sys_reg |= self._ADS_CRC_ENABLE if enable_crc else self._ADS_CRC_DISABLE
        sys_reg |= self._ADS_SENDSTATUS_ENABLE if enable_status_byte else self._ADS_SENDSTATUS_DISABLE
        self.write_single_register(self._REG_ADDR_SYS, sys_reg)

        if enable_crc:
            self.init_crc()


@dataclass
class ChannelConfig:
    """
    For ADS114S0x:

    Defines how to read and post-process one ADC input.
    """

    name: str
    ain_pos_code: int = ADS114S0x._ADS_P_AIN0
    ain_neg_code: int = ADS114S0x._ADS_N_AINCOM
    postprocess: Optional[Callable[[float], float]] = None
    units: str = "V"


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
    ) -> None:
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
        max_attempts = 1000
        attempts = 0
        while not self._ready_to_read():
            sleep(0.001)
            attempts += 1
            if attempts > max_attempts:
                raise RuntimeError(
                    "Couldn't connect to the ADC, please ensure that the device is connected and powered on."
                )

        self._data = self._read_data_millivolts()

    def calibrate(self) -> None:
        """
        Perform offset and gain calibration on the ADC.
        """
        self._offset_calibration()

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
            msg (list[int]): message to be sent to the ADS131M0x separated into bytes.
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
        osr = (self._clock_freq / 2) / self._data_rate
        osr_reg = int(math.log2(osr) - 7)
        self._ENABLE_CHANNELS_CLOCK &= ~(0b111 << 2)
        self._ENABLE_CHANNELS_CLOCK |= osr_reg << 2
        self._DISABLE_CHANNELS_CLOCK &= ~(0b111 << 2)
        self._DISABLE_CHANNELS_CLOCK |= osr_reg << 2
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
        mv = 1000 * self._data_counts / 2 ** (self._RESOLUTION - 1) * self._voltage_reference
        return mv

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
