import time
from typing import Optional, Union

import numpy as np
from smbus2 import SMBus

from opensourceleg.logging import LOGGER
from opensourceleg.math import from_twos_complement, to_twos_complement
from opensourceleg.sensors.base import EncoderBase
from opensourceleg.utilities import SoftRealtimeLoop


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
        tag: str = "AS5048B",
        bus: str = "/dev/i2c",
        A1_adr_pin: bool = False,
        A2_adr_pin: bool = False,
        zero_position: int = 0,
        enable_diagnostics: bool = False,
        offline: bool = False,
    ) -> None:
        """
        Class for the AS5048B encoder, implements the Encoder interface

        https://www.mouser.com/datasheet/2/588/AS5048_DS000298_4_00-2324531.pdf


        Args:
            tag (str): Tag name for the encoder
            bus (str): Path to the i2c bus ex. '/dev/i2c-1'
            A1_adr_pin (int): State of the adress pin A1 on the AS5048A module
            A2_adr_pin (int): State of the adress pin A1 on the AS5048A module
            zero_position (int): The zero position of the encoder

        Author: Axel Sjögren Holtz (axel.sjogren.holtz@vgregion.se),
                Senthur Ayyappan (senthura@umich.edu)
        """
        self.bus = bus
        self.enable_diagnostics = enable_diagnostics

        super().__init__(tag=tag, offline=offline)

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

        self._encoder_map: Union[np.polynomial.polynomial.Polynomial, None] = None

    def start(self) -> None:
        LOGGER.info(f"Opening encoder communication: {self.__repr__()}")
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

    def set_encoder_map(self, encoder_map: np.polynomial.polynomial.Polynomial) -> None:
        """
        Sets the encoder map to correct for nonlinearities in the encoder

        Args:
            encoder_map (np.polynomial.polynomial.Polynomial): The encoder map to set

        Returns:
            None

        Examples:
            >>> actuator = DephyActuator(port='/dev/ttyACM0')
            >>> actuator.start()
            >>> actuator.set_encoder_map(np.polynomial.polynomial.Polynomial(coef=[1, 2, 3, 4, 5]))
        """
        self._encoder_map = encoder_map

    # def apply_state(self, state: Encoder.State) -> None:
    # raise NotImplementedError(f"apply_state not implemented for {self.__class__}")

    @staticmethod
    def _get_14bit(bytesToParse: bytes) -> int:
        return (bytesToParse[0] << 6) | bytesToParse[1]  # int() is unnecessary

    @staticmethod
    def _set_14bit(intToParse: int) -> bytes:
        """
        Convert a 14bit integer to bytes <msb[13:6]><lsb[5:0]>

        Args
            intToParse (int): The integer to convert to bytes

        Raises
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
        """Get the current angular position in radians"""
        signed_output = from_twos_complement(self.counts, 14)
        raw_position = signed_output * self._scale_factor

        if self._encoder_map is not None:
            raw_position = self._encoder_map(raw_position)

        return raw_position

    @property
    def counts(self) -> int:
        """Get the raw encoder output as counts of full scale output.

        Returns:
            int: Encoder output in counts [0, FS]
        """
        return AS5048B._get_14bit(self._encdata_new[4:6])

    @property
    def velocity(self) -> float:
        """Calculate angular velocity in radians per second"""
        try:
            # TODO: Add linearization logic here for the velocity attribute
            LOGGER.warning(
                "Velocity attribute does not use the linearization map. "
                "Please calculate the velocity using the position attribute."
            )
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
        """Get absolute angular position accounting for rotations"""
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
        """Reads the content of the Zero position registers of the Encoder

        Returns:
            int: The 14 bit value stored in the Zero offset OTP registers
        """
        registers = self._read_registers(AS5048B.OTP_ZERO_POSITION_HIGH, 2)
        return AS5048B._get_14bit(registers)

    @zero_position.setter
    def zero_position(self, value: int) -> None:
        """Sets the zero position OTP registers (but does not burn them)

        Args:
            value (int): The content of the Zero offset registers

        Raises:
            OverflowError: If value >= 2^14
            ValueError: If value is negative or too large
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
        Calculates the midpoint between the current endpoints and sets it as
        the zero position.
        """
        input("Set joint in lower position and press enter")

        self.zero_position = 0
        self.update()
        min_value = from_twos_complement(self.counts, 14)

        input("Set joint in upper position and press enter")
        self.update()
        max_value = from_twos_complement(self.counts, 14)
        mid_value = (min_value + max_value) // 2
        self.zero_position = to_twos_complement(mid_value, 14)
        LOGGER.info(f"[SET] Zero registers: {self.zero_position}")

    @property
    def diag_compH(self) -> bool:
        """
        COMP high, indicated a weak magnetic field. It is
        recommended to monitor the magnitude value.

        Returns:
            Status of COMP_H diagnostics flag
        """
        return bool(self._encdata_new[1] & AS5048B.FLAG_COMP_H)

    @property
    def diag_compL(self) -> bool:
        """
        COMP low, indicates a high magnetic field. It is
        recommended to monitor in addition the magnitude
        value.

        Returns:
            Status of COMP_L diagnostics flag
        """
        return bool(self._encdata_new[1] & AS5048B.FLAG_COMP_L)

    @property
    def diag_COF(self) -> bool:
        """
        COF (CORDIC Overflow), logic high indicates an out of
        range error in the CORDIC part. When this bit is set, the
        angle and magnitude data is invalid. The absolute output
        maintains the last valid angular value.
        Returns:
            Status of COF diagnostics flag
        """
        return bool(self._encdata_new[1] & AS5048B.FLAG_COF)

    @property
    def diag_OCF(self) -> bool:
        """
        OCF (Offset Compensation Finished), logic high indicates
        the finished Offset Compensation Algorithm. After power
        up the flag remains always to logic high.

        Returns:
            Status of OCF diagnostics flag
        """
        return bool(self._encdata_new[1] & AS5048B.FLAG_OCF)

    @property
    def is_streaming(self) -> bool:
        """
        Check if the encoder is currently streaming.

        Returns:
            bool: True if the encoder is streaming, False otherwise.
        """
        return self._is_streaming

    @property
    def data(self) -> bytes:
        """
        Get the raw data from the encoder

        Returns:
            bytes: The latest raw data from the encoder.
        """
        if self._data is None:
            return b""  # Return empty bytes if no data available

        return self._data

    @property
    def encoder_map(self) -> Optional[np.polynomial.polynomial.Polynomial]:
        """
        Polynomial coefficients defining the encoder map from counts to radians.

        Returns:
            Optional[np.polynomial.polynomial.Polynomial]: The encoder map

        Examples:
            >>> encoder = AS5048B(port='/dev/ttyACM0')
            >>> encoder.start()
            >>> print(encoder.encoder_map)
        """
        if self._encoder_map is not None:
            return self._encoder_map
        else:
            LOGGER.warning(msg="Encoder map is not set. Please create one using a rbot")
            return None


if __name__ == "__main__":
    frequency = 200

    knee_enc = AS5048B(
        tag="knee",
        bus="/dev/i2c-1",
        A1_adr_pin=True,
        A2_adr_pin=False,
        zero_position=0,
    )

    ankle_enc = AS5048B(
        tag="ankle",
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

        knee_enc.zero_position = knee_enc.counts  # sets the current position to 0 rad
        ankle_enc.zero_position = ankle_enc.counts  # sets the current position to 0 rad

        for _t in clock:
            knee_enc.update()
            ankle_enc.update()
            LOGGER.info(np.rad2deg(knee_enc.position), np.rad2deg(knee_enc.abs_ang))
            LOGGER.info(np.rad2deg(ankle_enc.position), np.rad2deg(ankle_enc.abs_ang))
