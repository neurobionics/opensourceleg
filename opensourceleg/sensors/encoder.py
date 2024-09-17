# 
# Author: Axel Sjögren Holtz (axel.sjogren.holtz@vgregion.se)
# 
import time
import numpy as np
from smbus2 import SMBus
from opensourceleg.utilities.utilities import from_twos_compliment, to_twos_compliment


class AS5048B_Encoder: #ToDo: We use AS5048B_Encoder -- need to look into name change A-- uses SPI, B uses I2C
    """Class for the AS5048B encoder, implements the Encoder interface

    https://www.mouser.com/datasheet/2/588/AS5048_DS000298_4_00-2324531.pdf


    Args:
        Encoder (_type_): _description_
        bus (str): Path to the i2c bus ex. '/dev/i2c-1'
        A1_adr_pin (int): State of the adress pin A1 on the AS5048A module
        A2_adr_pin (int): State of the adress pin A1 on the AS5048A module
        name (str): _description_
        debug_level (int): _description_
    Raises:
        KeyError: _description_
        ValueError: _description_
        KeyError: _description_

    Returns:
        _type_: _description_

    Author: Axel Sjögren Holtz (axel.sjogren.holtz@vgregion.se)
    """

    ENC_RESOLUTION = 2**14  # 14 bit resolution

    I2C_BASE_ADR_7BIT = (
        0b1000000  # The adress base on the format <base[6:2]> <A1[1]> <A2[0]>
    )

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

    """Class for the AS5048A encoder, implements the Encoder interface

    https://www.mouser.com/datasheet/2/588/AS5048_DS000298_4_00-2324531.pdf


    Args:
        Encoder (_type_): _description_
        bus (str): Path to the i2c bus ex. '/dev/i2c-1'
        A1_adr_pin (int): State of the adress pin A1 on the AS5048A module
        A2_adr_pin (int): State of the adress pin A1 on the AS5048A module
        name (str): _description_
        debug_level (int): _description_

    Raises:
        KeyError: _description_
        ValueError: _description_
        KeyError: _description_

    Returns:
        _type_: _description_
    """

    def __init__(
        self,
        bus: str = "/dev/i2c",
        A1_adr_pin: bool = False,
        A2_adr_pin: bool = False,
        name: str = "AS5048B_Encoder",
        zero_position: int = 0,
        **kwargs,
    ) -> None:
        # super().__init__(name=name, **kwargs)
        self.name = name
        self.bus = bus
        self.addr = AS5048B_Encoder._calculate_I2C_adress(A1_adr_pin, A2_adr_pin)
        #print('addr',self.addr)
        self._reset_data()

        self._zero_to_set = zero_position
        self.rotations = 0

    def _start(self) -> None:
        print("Open encoder communication", self.__class__.__name__, self.name)
        self._SMBus = SMBus(self.bus)
        self._update()
        if self.zero_position != self._zero_to_set:
            self.zero_position = self._zero_to_set
            print(f"Set zero position to", self.zero_position)

    def _stop(self) -> None:
        if hasattr(self, "_SMBus"):
            self._SMBus.close()
        self._reset_data()

    def _update(self) -> None:
        self._read_data_registers()
        #self._check_diagnostics()

    # def apply_state(self, state: Encoder.State) -> None:
        # raise NotImplementedError(f"apply_state not implemented for {self.__class__}")

    @staticmethod
    def _calculate_I2C_adress(a1: bool, a2: bool) -> int:
        # (a1, a2) = adress_pins
        return AS5048B_Encoder.I2C_BASE_ADR_7BIT | ((bool(a2)) << 1) | ((bool(a1)) << 0)

    @staticmethod
    def _get_14bit(bytesToParse: bytes) -> int:
        return int((bytesToParse[0] << 6) | bytesToParse[1])

    @staticmethod
    def _set_14bit(intToParse: int) -> bytes:
        """
        Convert a 14bit integer to bytes <msb[13:6]><lsb[5:0]>

        Args
            intToParse (int): The integer to convert to bytes

        Raises
            OverflowError: If intToParse >= 2^14
        """
        if intToParse >= AS5048B_Encoder.ENC_RESOLUTION:
            raise OverflowError(
                f"Argument intToParse={intToParse} >= 2^14 bit encoder resolution"
            )
        return bytes([(intToParse >> 6), intToParse & 0x3F])

    def _reset_data(self) -> None:
        self._encdata_old = bytes(6)
        self._encdata_old_timestamp = 0
        self._encdata_new = bytes(6)
        self._encdata_new_timestamp = 0

    def _write_registers(self, register: int, data: bytes) -> None:
        self._SMBus.write_i2c_block_data(self.addr, register, data)

    def _read_registers(self, register, len) -> bytes:
        return bytes(self._SMBus.read_i2c_block_data(self.addr, register, len))

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
        self._encdata_old = self._encdata_new
        self._encdata_old_timestamp = self._encdata_new_timestamp

        self._encdata_new = self._read_registers(
            AS5048B_Encoder.AUTOMATIC_GAIN_CONTROL, 6
        )
        self._encdata_new_timestamp = time.monotonic_ns()

    def _check_diagnostics(self):
        if not self.diag_OCF:
            raise OSError("Invalid data returned on read, DIAG_OCF != 1")

        if self.diag_COF:
            print("CORDIC Overflow, sample invalid")

        if self.diag_compH:
            print("Low magnetic field comp triggered")

        if self.diag_compL:
            print("High magnetic field comp triggered")

    @property
    def position(self) -> float:
        signed_output = from_twos_compliment(self.encoder_output, 14)
        return (signed_output * 2 * np.pi) / AS5048B_Encoder.ENC_RESOLUTION

    @property
    def velocity(self) -> float:
        return 0.0


    @property
    def encoder_output(self) -> int:
        """Get the raw encoder output as counts of full scale output.

        Returns:
            int: Encoder output in counts [0, FS]
        """
        return AS5048B_Encoder._get_14bit(self._encdata_new[4:6])

    @property
    def velocity(self) -> float:
        try:
            encAngleDataOld = AS5048B_Encoder._get_14bit(self._encdata_old[4:6])
            encAngleDataNew = AS5048B_Encoder._get_14bit(self._encdata_new[4:6])
            # Timediff is converted from ns to s (x10^-9)
            timediff = (
                (self._encdata_new_timestamp - self._encdata_old_timestamp) + 1
            ) * 10**-9
        except TypeError:
            ## Typeerror indicate we only took one sample and have 0 velocity.
            return float(0)
        else:
            return (
                (encAngleDataNew - encAngleDataOld)
                * (2 * np.pi)
                / AS5048B_Encoder.ENC_RESOLUTION
                / (timediff)
            )
            
    @property
    def abs_ang(self) -> float:
        try:
            encAngleDataOld = AS5048B_Encoder._get_14bit(self._encdata_old[4:6])
            encAngleDataNew = AS5048B_Encoder._get_14bit(self._encdata_new[4:6])
            
        except TypeError:
            ## TypeError indicate we only took one sample and have 0 velocity.
            return self.position
        else:

            encAngRadOld = (from_twos_compliment(encAngleDataOld, 14) * 2 * np.pi) / AS5048B_Encoder.ENC_RESOLUTION
            encAngRadNew = (from_twos_compliment(encAngleDataNew, 14) * 2 * np.pi) / AS5048B_Encoder.ENC_RESOLUTION

            if encAngRadNew - encAngRadOld > 0.9 * (2 * np.pi):
                self.rotations -= 1
            elif encAngRadNew - encAngRadOld < -0.9 * (2 * np.pi):
                self.rotations += 1

            return (encAngRadNew + (2 * np.pi)*self.rotations)            

    @property
    def zero_position(self) -> int:
        """Reads the content of the Zero position registers of the Encoder

        Returns:
            int: The 14 bit value stored in the Zero offset OTP registers
        """
        registers = self._read_registers(AS5048B_Encoder.OTP_ZERO_POSITION_HIGH, 2)
        return AS5048B_Encoder._get_14bit(registers)

    @zero_position.setter
    def zero_position(self, value: int):
        """Sets the zero position OTP registers (but does not burn them)

        Args:
            value (int): The content of the Zero offset registers

        Raises:
            OverflowError: If value >= 2^14
        """
        assert (
            0 <= value < (AS5048B_Encoder.ENC_RESOLUTION - 1)
        ), "Zero position must be between 0 and 16383"
        try:
            payload = AS5048B_Encoder._set_14bit(value)
        except OverflowError:
            raise OverflowError(
                f"Argument value={value} >= 2^14 bit encoder resolution"
            )
        else:
            self._write_registers(AS5048B_Encoder.OTP_ZERO_POSITION_HIGH, payload)

    def set_zero(self) -> None:
        """
        Calculates the midpoint between the current endpoints and sets it as
        the zero position.
        """
        input("Set joint in lower position and press enter")

        self.zero_position = 0
        self._update()
        min = from_twos_compliment(self.encoder_output, 14)

        input("Set joint in upper position and press enter")
        self._update()
        max = from_twos_compliment(self.encoder_output, 14)
        mid = (min + max) // 2
        self.zero_position = to_twos_compliment(mid, 14)
        print(f"[SET] Zero registers:", self.zero_position)

    @property
    def diag_compH(self) -> bool:
        """
        COMP high, indicated a weak magnetic field. It is
        recommended to monitor the magnitude value.

        Returns:
            Status of COMP_H diagnostics flag
        """
        return bool(self._encdata_new[1] & AS5048B_Encoder.FLAG_COMP_H)

    @property
    def diag_compL(self) -> bool:
        """
        COMP low, indicates a high magnetic field. It is
        recommended to monitor in addition the magnitude
        value.

        Returns:
            Status of COMP_L diagnostics flag
        """
        return bool(self._encdata_new[1] & AS5048B_Encoder.FLAG_COMP_L)

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
        return bool(self._encdata_new[1] & AS5048B_Encoder.FLAG_COF)

    @property
    def diag_OCF(self) -> bool:
        """
        OCF (Offset Compensation Finished), logic high indicates
        the finished Offset Compensation Algorithm. After power
        up the flag remains always to logic high.

        Returns:
            Status of OCF diagnostics flag
        """
        return bool(self._encdata_new[1] & AS5048B_Encoder.FLAG_OCF)

    def __repr__(self) -> str:
        ang = self.position
        vel = self.velocity
        str = f"\n\tAngle: {ang:.3f} rad\n\tVelocity: {vel:.3f} rad/s"
        return str


if __name__ == "__main__":
    knee_enc = AS5048B_Encoder(
        name="knee",
        basepath="/",
        bus="/dev/i2c-1",
        A1_adr_pin=True,
        A2_adr_pin=False,
        zero_position=0,
    )

    ankle_enc = AS5048B_Encoder(
        name="ankle",
        basepath="/",
        bus="/dev/i2c-1",
        A1_adr_pin=False,
        A2_adr_pin=True,
        zero_position=0,
    )

    knee_enc._start()
    ankle_enc._start()
    knee_enc._update()
    ankle_enc._update()
    knee_enc.set_zero() # if you want 0 at the midpoint of a given range
    ankle_enc.set_zero() # if you want 0 at the midpoint of a given range
    knee_enc.zero_position = knee_enc.encoder_output # sets the current position to 0 rad
    ankle_enc.zero_position = ankle_enc.encoder_output # sets the current position to 0 rad

    while True:
        try:
            knee_enc._update()
            ankle_enc._update()
            print(np.rad2deg(knee_enc.position),np.rad2deg(knee_enc.abs_ang))
            print(np.rad2deg(ankle_enc.position),np.rad2deg(ankle_enc.abs_ang))

            time.sleep(1 / 200)
        except KeyboardInterrupt:
            break
        
    knee_enc._stop()
    ankle_enc._stop()
