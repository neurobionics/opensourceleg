try:
    from smbus2 import SMBus
except ImportError:
    print("Failed to import smbus2 ")
import time
import numpy as np

I2C_addr = 0x66

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

def read_uncompressed_strain(bus, I2C_addr):
    data = []
    for i in range(MEM_R_CH1_H, MEM_R_CH6_L+1):
        data.append(bus.read_byte_data(I2C_addr, i))

    return unpack_uncompressed_strain(data)

def unpack_uncompressed_strain(data):
    ch1 = (data[0] << 8) | data[1]
    ch2 = (data[2] << 8) | data[3]
    ch3 = (data[4] << 8) | data[5]
    ch4 = (data[6] << 8) | data[7]
    ch5 = (data[8] << 8) | data[9]
    ch6 = (data[10] << 8) | data[11]
    return np.array([ch1, ch2, ch3, ch4, ch5, ch6])


def unpack_compressed_strain(data):
    # compressed data format (from flexsea-strain):
    #  //combo[0]: 0000 0000 0000 1111
    #  //combo[1]: 1111 1111 2222 2222
    #  //combo[2]: 2222 3333 3333 3333
    #  //combo[3]: 4444 4444 4444 5555
    #  //combo[4]: ____ ____ 5555 5555

    # ezI2Cbuf[0]: 0000 0000
    # ezI2Cbuf[1]: 0000 1111
    # ezI2Cbuf[2]: 1111 1111
    # ezI2Cbuf[3]: 2222 2222
    # ezI2Cbuf[4]: 2222 3333
    # ezI2Cbuf[5]: 3333 3333
    # ezI2Cbuf[6]: 4444 4444
    # ezI2Cbuf[7]: 4444 5555
    # ezI2Cbuf[8]: 5555 5555

    # combo[0] = (tmp[0] << 4) | ((tmp[1] >> 8) & 0xFF);
    # combo[1] = (tmp[1] << 8) | ((tmp[2] >> 4) & 0xFFFF);
    # combo[2] = (tmp[2] << 12) | (tmp[3]);
    # combo[3] = (tmp[4] << 4) | ((tmp[5] >> 8) & 0xFF);
    # combo[4] = (tmp[5] & 0xFF);

    ch1 = (data[0] << 4) | ( (data[1] >> 4) & 0x0F)
    ch2 = ( (data[1] << 8) & 0x0F00) | data[2]
    ch3 = (data[3] << 4) | ( (data[4] >> 4) & 0x0F)
    ch4 = ( (data[4] << 8) & 0x0F00) | data[5]
    ch5 = (data[6] << 4) | ( (data[7] >> 4) & 0x0F)
    ch6 = ( (data[7] << 8) & 0x0F00) | data[8]
    return np.array([ch1, ch2, ch3, ch4, ch5, ch6])

def read_compressed_strain(bus, I2C_addr):
    data = []

    # read all 9 data registers of compressed data
    for i in range(MEM_R_CH1_H, MEM_R_CH1_H+9):
        data.append(bus.read_byte_data(I2C_addr, i))

    # unpack them and return as nparray
    return unpack_compressed_strain(data)

loadcell_matrix = np.array([
                                (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
                                (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
                                (-1047.16800,8.63900,-1047.28200,-20.70000,-1073.08800,-8.92300),
                                (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
                                (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
                                (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
                            ])

loadcell_zero = np.zeros((1,6), dtype=np.double)

def strain_V_to_wrench(unpacked_strain, exc=5, gain=125):
    loadcell_signed = (unpacked_strain - 2048) / 4095 * exc
    loadcell_coupled = loadcell_signed * 1000 / (exc * gain)
    return np.reshape(np.transpose(loadcell_matrix.dot(np.transpose(loadcell_coupled))) - loadcell_zero, (6,))

def wrench_to_strain_V(measurement, exc=5, gain=125):
    loadcell_coupled = (np.linalg.inv(loadcell_matrix)).dot(measurement)
    loadcell_signed = loadcell_coupled * (exc * gain) / 1000
    return ((loadcell_signed/exc)*4095 + 2048).round(0).astype(int)

if __name__ == "__main__":
    with SMBus(1) as bus:
        print("\n")
        while(True):
            # bus.pec = 1 # packet error checking
            # print("\n<><><><><><><><><><><>")
            # a = read_uncompressed_strain(bus,I2C_addr)
            # print(a)
            # print(strain_V_to_wrench(a))
            # print("----------------------")
            b = read_compressed_strain(bus, I2C_addr)
            # print(b)
            print('\rF = ({:3.2f}, {:3.2f}, {:3.2f}) M = ({:3.2f}, {:3.2f}, {:3.2f})'.format(
                strain_V_to_wrench(b)[0],
                strain_V_to_wrench(b)[1],
                strain_V_to_wrench(b)[2],
                strain_V_to_wrench(b)[3],
                strain_V_to_wrench(b)[4],
                strain_V_to_wrench(b)[5]
                ),end='')
            # print("<><><><><><><><><><><>\n")
            time.sleep(0.1)
    


