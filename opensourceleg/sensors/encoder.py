import time
from typing import Union

import numpy as np
from smbus2 import SMBus

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import EncoderBase
from opensourceleg.time import SoftRealtimeLoop
from opensourceleg.utilities.utilities import from_twos_complement, to_twos_complement


class AS5048B(EncoderBase):  # ToDo: We use AS5048B -- need to look into name change A-- uses SPI, B uses I2C
    ENC_RESOLUTION = 2**14  # 14 bit resolution
    I2C_BASE_ADR_7BIT = 0b1000000  # The adress base on the format <base[6:2]> <A1[1]> <A2[0]>

    ## Register adresses I2C
    OTP_ZERO_POSITION_HIGH = 0x16  # bit 13 through 6
    OTP_ZERO_POSITION_LOW = 0x17  # bit 5 through 0 (2 msbs of this aren't used)
    AUTOMATIC_GAIN_CONTROL = 0xFA  # 0 = high mag field, 255 = low mag field, 8 bit
    DIAGNOSTICS = 0xFB  # flags: 3 = comp high, 2 = comp low, 1 = COF, 0 = OCF
    MAGNITUDE_HIGH = 0xFC  # bit 13 through 6
    MAGNITUDE_LOW = 0xFD  # bit 5 through 0 (2 msbs of this aren't used)
    ANGLE_HIGH = 0xFE  # bit 13 through 6
    ANGLE_LOW = 0xFF  # bit 5 through 0 (2 msbs of this aren't used)

    ## Status flags Diagnostics registers
    FLAG_COMP_H = 0x1 << 3
    FLAG_COMP_L = 0x1 << 2
    FLAG_COF = 0x1 << 1
    FLAG_OCF = 0x1 << 0

    def __init__(
        self,
        bus: str = "/dev/i2c",
        A1_adr_pin: bool = False,
        A2_adr_pin: bool = False,
        name: str = "AS5048B",
        zero_position: int = 0,
        enable_diagnostics: bool = False,
    ) -> None:
        """
        Class for the AS5048B encoder, implements the Encoder interface

        https://www.mouser.com/datasheet/2/588/AS5048_DS000298_4_00-2324531.pdf


        Args:
            bus: Path to the i2c bus ex. '/dev/i2c-1'
            A1_adr_pin: State of the adress pin A1 on the AS5048A module
            A2_adr_pin: State of the adress pin A1 on the AS5048A module
            name: Tag name for the encoder
            zero_position: The zero position of the encoder

        Author: Axel Sjögren Holtz (axel.sjogren.holtz@vgregion.se),
                Senthur Ayyappan (senthura@umich.edu)
        """
        self.name = name
        self.bus = bus
        self.enable_diagnostics = enable_diagnostics

        super().__init__()

        self.addr = AS5048B.I2C_BASE_ADR_7BIT | ((bool(A2_adr_pin)) << 1) | ((bool(A1_adr_pin)) << 0)
        self._reset_data()

        self._zero_to_set = zero_position
        self._is_streaming = False
        self._data: Union[bytes, None] = None
        self.rotations = 0
        self._SMBus: Union[SMBus, None] = None

        # Cache for frequently used values
        self._two_pi = 2 * np.pi
        self._scale_factor = self._two_pi / AS5048B.ENC_RESOLUTION

    def start(self) -> None:
        LOGGER.info(f"Opening encoder communication: {self.__class__.__name__} - {self.name}")
        self._SMBus = SMBus(self.bus)
        self.update()  # Use public method instead of _update
        if self.zero_position != self._zero_to_set:
            self.zero_position = self._zero_to_set
            LOGGER.info(f"Set zero position to {self.zero_position}")

        self._is_streaming = True

    def stop(self) -> None:
        if self._SMBus:
            self._SMBus.close()
            self._SMBus = None
        self._reset_data()
        self._is_streaming = False

    def update(self) -> None:
        self._read_data_registers()

        if self.enable_diagnostics:
            self._check_diagnostics()

    # def apply_state(self, state: Encoder.State) -> None:
    # raise NotImplementedError(f"apply_state not implemented for {self.__class__}")

    @staticmethod
    def _get_14bit(bytesToParse: bytes) -> int:
        return (bytesToParse[0] << 6) | bytesToParse[1]  # int() is unnecessary

    @staticmethod
    def _set_14bit(intToParse: int) -> bytes:
        """
        Convert a 14bit integer to bytes <msb[13:6]><lsb[5:0]>

        Args:
            intToParse: The integer to convert to bytes

        Raises:
            OverflowError: If intToParse >= 2^14
        """
        if intToParse >= AS5048B.ENC_RESOLUTION:
            raise OverflowError(f"Argument intToParse={intToParse} >= 2^14 bit encoder resolution")
        return bytes([(intToParse >> 6), intToParse & 0x3F])

    def _reset_data(self) -> None:
        # Use bytearray for better performance when we need to modify
        self._encdata_old = bytearray(6)
        self._encdata_old_timestamp = 0
        self._encdata_new = bytearray(6)
        self._encdata_new_timestamp = 0

    def _write_registers(self, register: int, data: bytes) -> None:
        if self._SMBus is None:
            raise RuntimeError("SMBus not initialized. Call start() first.")
        self._SMBus.write_i2c_block_data(self.addr, register, data)

    def _read_registers(self, register: int, length: int) -> bytes:
        if self._SMBus is None:
            raise RuntimeError("SMBus not initialized. Call start() first.")
        return bytes(self._SMBus.read_i2c_block_data(self.addr, register, length))

    def _read_data_registers(self) -> None:
        """
        Read data output registers
            [0]
            [1]
            [2] 0xFC MAG H
            [3] 0xFD MAG L
            [4] 0xFE ANG H
            [5] 0xFF ANG L
        """
        # Swap references instead of copying data
        self._encdata_old, self._encdata_new = self._encdata_new, self._encdata_old
        self._encdata_old_timestamp, self._encdata_new_timestamp = self._encdata_new_timestamp, time.monotonic_ns()

        # Read directly into the bytearray
        data = self._read_registers(AS5048B.AUTOMATIC_GAIN_CONTROL, 6)
        self._encdata_new[:] = data
        self._data = data

    def _check_diagnostics(self) -> None:
        if not self.diag_OCF:
            raise OSError("Invalid data returned on read, DIAG_OCF != 1")

        if self.diag_COF:
            LOGGER.info("CORDIC Overflow, sample invalid")

        if self.diag_compH:
            LOGGER.info("Low magnetic field comp triggered")

        if self.diag_compL:
            LOGGER.info("High magnetic field comp triggered")

    @property
    def position(self) -> float:
        """
        Get the current angular position in radians.

        Returns:
            The current angular position in radians.
        """
        signed_output = from_twos_complement(self.encoder_output, 14)
        return signed_output * self._scale_factor

    @property
    def encoder_output(self) -> int:
        """
        Get the raw encoder output as counts of full scale output.

        Returns:
            Encoder output in counts [0, FS].
        """
        return AS5048B._get_14bit(self._encdata_new[4:6])

    @property
    def velocity(self) -> float:
        """
        Calculate angular velocity in radians per second.

        Returns:
            The angular velocity in radians per second.
        """
        try:
            encAngleDataOld = AS5048B._get_14bit(self._encdata_old[4:6])
            encAngleDataNew = AS5048B._get_14bit(self._encdata_new[4:6])
            # Timediff is converted from ns to s
            timediff = (self._encdata_new_timestamp - self._encdata_old_timestamp) * 1e-9

            if timediff <= 0:
                return 0.0

            return (encAngleDataNew - encAngleDataOld) * self._scale_factor / timediff

        except (TypeError, ZeroDivisionError):
            return 0.0

    @property
    def abs_ang(self) -> float:
        """
        Get the absolute angle in radians.

        Returns:
            The absolute angle in radians.
        """
        try:
            encAngleDataOld = AS5048B._get_14bit(self._encdata_old[4:6])
            encAngleDataNew = AS5048B._get_14bit(self._encdata_new[4:6])
        except TypeError:
            return self.position

        encAngRadOld = from_twos_complement(encAngleDataOld, 14) * self._scale_factor
        encAngRadNew = from_twos_complement(encAngleDataNew, 14) * self._scale_factor

        # Detect rotation crossings
        diff = encAngRadNew - encAngRadOld
        if diff > 0.9 * self._two_pi:
            self.rotations -= 1
        elif diff < -0.9 * self._two_pi:
            self.rotations += 1

        return encAngRadNew + self._two_pi * self.rotations

    @property
    def zero_position(self) -> int:
        """
        Get the zero position of the encoder.

        Returns:
            The zero position in encoder counts.
        """
        registers = self._read_registers(AS5048B.OTP_ZERO_POSITION_HIGH, 2)
        return AS5048B._get_14bit(registers)

    @zero_position.setter
    def zero_position(self, value: int) -> None:
        """
        Set the zero position of the encoder.

        Args:
            value: The new zero position in encoder counts.

        Raises:
            ValueError: If the value is not within the valid range.
        """
        if not (0 <= value < (AS5048B.ENC_RESOLUTION - 1)):
            raise ValueError(f"Zero position must be between 0 and {AS5048B.ENC_RESOLUTION - 2}")
        try:
            payload = AS5048B._set_14bit(value)
        except OverflowError as err:
            raise OverflowError(f"Argument value={value} >= 2^14 bit encoder resolution") from err
        else:
            self._write_registers(AS5048B.OTP_ZERO_POSITION_HIGH, payload)

    def set_zero_position(self) -> None:
        """
        Set the current position as the zero position.

        This method reads the current encoder position and sets it as the new zero position.
        """
        input("Set joint in lower position and press enter")

        self.zero_position = 0
        self.update()
        min_value = from_twos_complement(self.encoder_output, 14)

        input("Set joint in upper position and press enter")
        self.update()
        max_value = from_twos_complement(self.encoder_output, 14)
        mid_value = (min_value + max_value) // 2
        self.zero_position = to_twos_complement(mid_value, 14)
        LOGGER.info(f"[SET] Zero registers: {self.zero_position}")

    @property
    def diag_compH(self) -> bool:
        """
        Check if the magnetic field compensation for high field is triggered.

        Returns:
            True if high field compensation is triggered, False otherwise.
        """
        return bool(self._encdata_new[1] & AS5048B.FLAG_COMP_H)

    @property
    def diag_compL(self) -> bool:
        """
        Check if the magnetic field compensation for low field is triggered.

        Returns:
            True if low field compensation is triggered, False otherwise.
        """
        return bool(self._encdata_new[1] & AS5048B.FLAG_COMP_L)

    @property
    def diag_COF(self) -> bool:
        """
        Check if a CORDIC overflow has occurred.

        Returns:
            True if a CORDIC overflow has occurred, False otherwise.
        """
        return bool(self._encdata_new[1] & AS5048B.FLAG_COF)

    @property
    def diag_OCF(self) -> bool:
        """
        Check if the data is valid (no overflow).

        Returns:
            True if the data is valid, False otherwise.
        """
        return bool(self._encdata_new[1] & AS5048B.FLAG_OCF)

    @property
    def is_streaming(self) -> bool:
        """
        Check if the encoder is currently streaming data.

        Returns:
            True if the encoder is streaming, False otherwise.
        """
        return self._is_streaming

    @property
    def data(self) -> bytes:
        """
        Get the raw encoder data.

        Returns:
            The raw encoder data as bytes.
        """
        if self._data is None:
            return b""  # Return empty bytes if no data available

        return self._data

    def __repr__(self) -> str:
        return f"\n\tAngle: {self.position:.3f} rad\n\tVelocity: {self.velocity:.3f} rad/s"


if __name__ == "__main__":
    frequency = 200

    knee_enc = AS5048B(
        name="knee",
        bus="/dev/i2c-1",
        A1_adr_pin=True,
        A2_adr_pin=False,
        zero_position=0,
    )

    ankle_enc = AS5048B(
        name="ankle",
        bus="/dev/i2c-1",
        A1_adr_pin=False,
        A2_adr_pin=True,
        zero_position=0,
    )

    clock = SoftRealtimeLoop(dt=1 / frequency)

    with knee_enc, ankle_enc:
        knee_enc.update()
        ankle_enc.update()

        knee_enc.set_zero_position()  # if you want 0 at the midpoint of a given range
        ankle_enc.set_zero_position()  # if you want 0 at the midpoint of a given range

        knee_enc.zero_position = knee_enc.encoder_output  # sets the current position to 0 rad
        ankle_enc.zero_position = ankle_enc.encoder_output  # sets the current position to 0 rad

        for _t in clock:
            knee_enc.update()
            ankle_enc.update()
            LOGGER.info(np.rad2deg(knee_enc.position), np.rad2deg(knee_enc.abs_ang))
            LOGGER.info(np.rad2deg(ankle_enc.position), np.rad2deg(ankle_enc.abs_ang))
