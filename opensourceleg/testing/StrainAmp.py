from smbus2 import SMBus
import time
import numpy as np
import I2CManager
# a class to eventually put into the hardware.py file


# make a general singleton I2C manager, then subscribe each peripheral to that
# might be able to just declare a bus, then pass the bus into all I2C peripherals
class StrainAmp:
    """
    A class to directly manage the 6ch strain gauge amplifier over I2C
    """
    # register numbers for the "ezi2c" interface on the strainamp
    # found in source code here: https://github.com/JFDuval/flexsea-strain/tree/dev
    MEM_R_CH1_H=8
    MEM_R_CH1_L=9
    MEM_R_CH2_H=10
    MEM_R_CH2_L=11
    MEM_R_CH3_H=12
    MEM_R_CH3_L=13
    MEM_R_CH4_H=14
    MEM_R_CH4_L=15
    MEM_R_CH5_H=16
    MEM_R_CH5_L=17
    MEM_R_CH6_H=18
    MEM_R_CH6_L=19

    def __init__(self, bus, I2C_addr=0x66) -> None:
        """Create a strainamp object, to talk over I2C"""
        self._I2CMan = I2CManager(bus)
        self.bus = bus
        self.addr = I2C_addr
        self.genvars = np.zeros(6)
        self.is_streaming = True

    def read_uncompressed_strain(self):
        """Used for an older version of the strain amp firmware (at least pre-2017)"""
        data = []
        for i in range(self.MEM_R_CH1_H, self.MEM_R_CH6_L+1):
            data.append(self._I2CMan.bus.read_byte_data(self.addr, i))

        return self.unpack_uncompressed_strain(data)

    def read_compressed_strain(self):
        """Used for more recent versions of strain amp firmware"""
        data = []
        # read all 9 data registers of compressed data
        for i in range(self.MEM_R_CH1_H, self.MEM_R_CH1_H+9):
            data.append(self._I2CMan.bus.read_byte_data(self.addr, i))

        # unpack them and return as nparray
        return self.unpack_compressed_strain(data)

    def update(self):
        """Called to update data of strain amp. Also returns data."""
        self.genvars = self.read_compressed_strain()
        return self.genvars
        
    @staticmethod
    def unpack_uncompressed_strain(data):
        """Used for an older version of the strain amp firmware (at least pre-2017)"""
        ch1 = (data[0] << 8) | data[1]
        ch2 = (data[2] << 8) | data[3]
        ch3 = (data[4] << 8) | data[5]
        ch4 = (data[6] << 8) | data[7]
        ch5 = (data[8] << 8) | data[9]
        ch6 = (data[10] << 8) | data[11]
        return np.array([ch1, ch2, ch3, ch4, ch5, ch6])

    @staticmethod
    def unpack_compressed_strain(data):
        """Used for more recent versions of strainamp firmware"""
        ch1 = (data[0] << 4) | ( (data[1] >> 4) & 0x0F)
        ch2 = ( (data[1] << 8) & 0x0F00) | data[2]
        ch3 = (data[3] << 4) | ( (data[4] >> 4) & 0x0F)
        ch4 = ( (data[4] << 8) & 0x0F00) | data[5]
        ch5 = (data[6] << 4) | ( (data[7] >> 4) & 0x0F)
        ch6 = ( (data[7] << 8) & 0x0F00) | data[8]
        return np.array([ch1, ch2, ch3, ch4, ch5, ch6])

    @staticmethod
    def strain_data_to_wrench(unpacked_strain, loadcell_matrix, loadcell_zero, exc=5, gain=125):
        """Converts strain values between 0 and 4095 to a wrench in N and Nm"""
        loadcell_signed = (unpacked_strain - 2048) / 4095 * exc
        loadcell_coupled = loadcell_signed * 1000 / (exc * gain)
        return np.reshape(np.transpose(loadcell_matrix.dot(np.transpose(loadcell_coupled))) - loadcell_zero, (6,))

    @staticmethod
    def wrench_to_strain_data(measurement, loadcell_matrix, exc=5, gain=125):
        """Wrench in N and Nm to the strain values that would give that wrench"""
        loadcell_coupled = (np.linalg.inv(loadcell_matrix)).dot(measurement)
        loadcell_signed = loadcell_coupled * (exc * gain) / 1000
        return ((loadcell_signed/exc)*4095 + 2048).round(0).astype(int)







if __name__ == '__main__':
    with SMBus(1) as bus:
        amp = StrainAmp(bus, 0x66)
        loadcell_matrix = np.array([
                                (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
                                (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
                                (-1047.16800,8.63900,-1047.28200,-20.70000,-1073.08800,-8.92300),
                                (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
                                (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
                                (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
                            ])
        loadcell_zero = np.zeros((1,6), dtype=np.double)
        print("\n")
        while(True):
            amp.update()
            print('\x1B[2A',end='')
            print('F = ({}, {}, {}) M = ({}, {}, {})             '.format(
                                                            amp.data[0],
                                                            amp.data[1],
                                                            amp.data[2],
                                                            amp.data[3],
                                                            amp.data[4],
                                                            amp.data[5]),
                                                            end='\n'
                                                            )
            W = amp.strain_V_to_wrench(amp.data,loadcell_matrix,loadcell_zero)
            print('F = ({:002.2f}, {:002.2f}, {:002.2f}) M = ({:002.2f}, {:002.2f}, {:002.2f})             '.format(
                                                            W[0],
                                                            W[1],
                                                            W[2],
                                                            W[3],
                                                            W[4],
                                                            W[5],
                                                            end=''
                                                            ))
            time.sleep(0.1)
