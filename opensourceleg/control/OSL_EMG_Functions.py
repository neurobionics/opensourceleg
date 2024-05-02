############################### PACKAGE IMPORTS ################################

# Imports for Standard Python
from time import sleep, time
import datetime
import os, sys
import math
import numpy as np
import yaml

# Imports for FlexSEA
from flexsea import flexsea as fx
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

# Imports for OSL
from OSL_Modules.OSL_Calibration_Dual import OSL_Constants as osl
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_Storage as stor
from OSL_Modules.OSL_Calibration_Dual import OSL_CalibrationFunctions_Homing as home
from OSL_Modules.OSL_Calibration import OSL_Constants as osl

# Imports for Force/EMG Sensors
from signal import signal, SIGINT
import spidev

# Create Class Object for SPI Functions Related to Force/EMG Sensors
#sleep(5*osl.dtCenti)
#spi = spidev.SpiDev()
#spi.open(0, 0)
#spi.max_speed_hz=1000000

def initialize_spi():
	sleep(5*osl.dtCenti)
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz=1000000
    return spi

############################# FUNCTION DEFINITIONS #############################

def readadc(adcChan, spi):

    '''
    Function for evaluating force sensor data

    NOTE: Some of the documentation may be slightly misleading. Please refer to Ryan Posh for more information
    NOTE: This Function also appears in OSL_Modules/OSL_EMG/OSL_EMG_Functions.py

    INPUT:
        adcChan - Channel associated with force sensor of interest
    OUTPUT:
        data - Reading output from force sensor of interest
    '''

    # Read SPI Data
    if adcChan > 7 or adcChan < 0:
        return -1

    # Correct SPI Data to Appropriate Data Format
    spiDat = spi.xfer2([1, 8 + adcChan << 4, 0])
    data = ((spiDat[1] & 3) << 8) + spiDat[2]

    # Return Force Sensor Data
    return data

def filter_emg(raw, time_window, delay, empty_vec):
    window = int(time_window/delay)

    empty_vec = np.append(empty_vec, [raw])
    empty_vec = np.delete(empty_vec,0)

    emg_avg = np.sum(empty_vec)/window

    return emg_avg, empty_vec

def rectify_emg(raw, baseline):
    rectified = abs(raw - baseline)
    return rectified

def filter_zero(raw, filtered_prev, time_window, delay, reading):
    if raw != 0:
        emg_avg, reading = filter_emg(raw, time_window, delay, reading)
    else:
        emg_avg, reading = filter_emg(filtered_prev, time_window, delay, reading)
    return emg_avg, reading


def find_slope(gas_0, ta_0, stdev_2, stdev_3, flex_time, filter_time_window, delay, direction='plantarflex', intensity=100):
    directions = ['plantarflex', 'dorsiflex']
    if direction not in directions:
        raise ValueError("Invalid direction. Expected one of: %s" % directions)

    input('Please ' + direction + ' your ankle with an intensity of ' + str(intensity) + '% for ' + str(flex_time) + ' seconds. When ready hit Enter.')
    readyx = 'n'
    while readyx == 'n':
        start_time = time()
        all_ta = []
        all_gas = []
        all_m = []
        reading_4 = np.zeros(int(filter_time_window/delay))
        reading_5 = np.zeros(int(filter_time_window/delay))
        emg_avg_prev_gas = 0
        emg_avg_prev_ta = 0
        while time() < start_time + flex_time:
            emg_raw_gas = readadc(osl.pChan2, spi)
            emg_raw_rect_gas = rectify_emg(emg_raw_gas, gas_0)
            emg_avg_gas, reading_4 = filter_emg(emg_raw_rect_gas, filter_time_window, delay, reading_4)

            emg_raw_ta = readadc(osl.pChan3, spi)
            emg_raw_rect_ta = rectify_emg(emg_raw_ta, ta_0)
            emg_avg_ta, reading_5 = filter_emg(emg_raw_rect_ta, filter_time_window, delay, reading_5)

            all_gas = np.append(all_gas, [emg_avg_gas])
            all_ta = np.append(all_ta, [emg_avg_ta])
            if emg_avg_ta != 0:
                if abs(emg_avg_ta) > 2*abs(stdev_3) or abs(emg_avg_gas) > 2*abs(stdev_2):
                    #if emg_avg_ta < ta_0:
                    #    m = 20
                    #elif emg_avg_gas < gas_0:
                    #    m = 0
                    #else:
                    #    m = (emg_avg_gas - gas_0)/(emg_avg_ta - ta_0)
                    #all_m = np.append(all_m, [m])
                    m = (emg_avg_gas)/(emg_avg_ta)
                    all_m = np.append(all_m, [m])

            print('Calibrating co-contraction profile for ' + direction + 'ion...')
            print(emg_avg_gas, emg_avg_ta)
            sleep(delay)
        m_avg = np.mean(all_m)
        max_gas = np.amax(all_gas) # these are already rectified values
        max_ta = np.amax(all_ta)
        print('Average m: ' + str(m_avg) + ' max_GAS: ' + str(max_gas) + ' max_TA: ' + str(max_ta))
        readyx = input('Are you happy with the calibration results? [y/n] (Enter Stop to Exit Script): ')
        if readyx != 'n' and readyx != 'y' and readyx != 'Stop':
            readyx = input('Please enter either y, n, or Stop: ')
    return float(m_avg), float(max_gas), float(max_ta)


def noise_level(cal_time, filter_time_window, delay):
    ready2 = 'n'
    input('Please rest your muscle and stay inactive. When ready hit Enter.')
    while ready2 == 'n':
        start_time = time()
        cal_values_2 = []
        cal_values_3 = []
        reading_2 = np.ones(int(filter_time_window/delay))*510
        reading_3 = np.ones(int(filter_time_window/delay))*515
        while time() < start_time + cal_time:
            emg_raw_value_2 = readadc(osl.pChan2, spi)
            emg_avg_2_base, reading_2 = filter_emg(emg_raw_value_2, filter_time_window, delay, reading_2)

            emg_raw_value_3 = readadc(osl.pChan3, spi)
            emg_avg_3_base, reading_3 = filter_emg(emg_raw_value_3, filter_time_window, delay, reading_3)

            cal_values_2 = np.append(cal_values_2, [emg_avg_2_base])
            cal_values_3 = np.append(cal_values_3, [emg_avg_3_base])

            #print('Calibrating Baseline!')
            print(emg_avg_2_base, emg_avg_3_base)
            sleep(delay)

        baseline_2 = np.mean(cal_values_2)
        baseline_3 = np.mean(cal_values_3)
        stdev_2 = np.std(cal_values_2)
        stdev_3 = np.std(cal_values_3)
        print('Average ch2: ' + str(baseline_2) + ' Standard Deviation: ' + str(stdev_2))
        print('Average ch3: ' + str(baseline_3) + ' Standard Deviation: ' + str(stdev_3))

        ready2 = input('Are you happy with the baseline calibration value? [y/n] (Enter Stop to Exit Script): ')
        if ready2 != 'n' and ready2 != 'y' and ready2 != 'Stop':
            ready2 = input('Please enter either y, n, or Stop: ')

    return float(baseline_2), float(baseline_3), float(stdev_2), float(stdev_3), ready2



def calEMGDump(calData, filename):

    '''
    Function for dumping calibration data to the proper yaml file for future use
    Inputs:
        calData - Class structure storing the calibration data to save
        dev - Calibration data joint (0 for Knee, 1 for Ankle)
    Outputs:
        None
    '''

    # Open Appropriate File Based On Device ID
    pFile = open(filename,'w')

    # Dump Calibration Data to File and Close
    pDoc = yaml.dump(calData, pFile, Dumper = yaml.Dumper)
    pFile.close()


def calEMGLoad(filename):

    '''
    Function for loading calibration data from the proper yaml file
    Inputs:
        dev - Calibration data joint (0 for Knee, 1 for Ankle)
    Outputs:
        calEMGData - Class structure storing the calibration data
    '''

    # Open Appropriate File Based on Device ID
    pFile = open(filename)

    # Load Data From File and Close
    calEMGData = yaml.load(pFile, Loader = yaml.Loader)
    pFile.close()

    # Return Calibration Data
    return calEMGData


class CalEMGDataSingle:

    '''
    Class used for storing calibration data of a single actuator
    Structure:
        cal - Calibration Being Run (0: IMU only, 1: Angle only, 2+: Both)
        gyro - Z-Axis Gyroscope Bias
        xAccel - X-Axis Accelerometer Bias
        yAccel - Y-Axis Accelerometer Bias
        angExtMot - Motor Encoder Tick Value at Full Extension
        angFlexMot - Motor Encoder Tick Value at Full Flexion
        bpdMot - Ticks Per Degree Conversion Unit for Motor
        angExtJoint - Joint Encoder Tick Value at Full Extension
        angFlexJoint - Joint Encoder Tick Value at Full Flexion
        angVertJoint - Joint Encoder Tick Value at Vertical Orientation
        bpdJoint - Ticks Per Degree Conversion Unit for Joint
    '''

    # Init Method for Knee or Ankle Module
    def __init__(self, baseline_2=0, baseline_3=0, stdev_2=0, stdev_3=0, m_gas=0, m_ta=0, m_0=0, mvc_gasflex_gas=0, mvc_taflex_ta=0, mvc_gasflex_ta=0, mvc_taflex_gas=0, mag_max_gas=0, theta_gas=0, mag_max_ta=0, theta_ta=0):

        self.baseline_2 = baseline_2
        self.baseline_3 = baseline_3
        self.stdev_2 = stdev_2
        self.stdev_3 = stdev_3

        self.m_gas = m_gas
        self.m_ta = m_ta
        self.m_0 = m_0
        self.mvc_gasflex_gas = mvc_gasflex_gas
        self.mvc_taflex_ta = mvc_taflex_ta
        self.mvc_gasflex_ta = mvc_gasflex_ta
        self.mvc_taflex_gas = mvc_taflex_gas

        self.mag_max_gas = mag_max_gas
        self.theta_gas = theta_gas
        self.mag_max_ta = mag_max_ta
        self.theta_ta = theta_ta


def emgCalibration(FX, calEMGData, time_window, delay, filename):

    '''
    Function for calibrating the ankle module and storing the calibration data in a class object. This form uses the internal motor encoder
    Inputs:
        calEMGData - Class object to store calibration data in
    Outputs:
        calEMGData - Class object to store calibration data in
    '''

    # Take a baseline reading of noise level for the emg sensors
    calEMGData.baseline_2, calEMGData.baseline_3, calEMGData.stdev_2, calEMGData.stdev_3, ready2 = noise_level(5, time_window, delay)

    # ready3 = 'y'  # initialize to 'n' to run through

    if ready2 == 'Stop': # or ready3 == 'Stop':
        FX.stop_streaming(devId)
        sleep(0.2)
        FX.close(devId)
        sleep(0.1)

        print('Script Complete')

    # Calibrate the MVC and cocontraction slope data
    elif ready2 == 'y': # and ready3 == 'y':
        sleep(0.5)

        # Ask for MVC for gastroc (first) and TA (second). Calculate the cocontraction slope and MVC value for that respective muscle.
        calEMGData.m_gas, calEMGData.mvc_gasflex_gas, _ = find_slope(calEMGData.baseline_2, calEMGData.baseline_3, calEMGData.stdev_2, calEMGData.stdev_3, 1, time_window, delay, direction='plantarflex', intensity=100)
        calEMGData.m_ta, _, calEMGData.mvc_taflex_ta = find_slope(calEMGData.baseline_2, calEMGData.baseline_3, calEMGData.stdev_2, calEMGData.stdev_3, 1, time_window, delay, direction='dorsiflex', intensity=100)

        # establish a bisecting line between the 2 slopes
        calEMGData.m_0 = float(np.tan((np.arctan(calEMGData.m_gas) + np.arctan(calEMGData.m_ta))/2))

        # establish the MVC value for the undesired muscle by taking the MVC value measured for the desired muscle and placing it in line with the slope.
        calEMGData.mvc_gasflex_ta = calEMGData.mvc_gasflex_gas/calEMGData.m_gas
        calEMGData.mvc_taflex_gas = calEMGData.mvc_taflex_ta*calEMGData.m_ta

        # Find the magnitude and angle of the vector going from the baseline to the MVC value
        calEMGData.mag_max_gas = float(np.sqrt((calEMGData.mvc_gasflex_gas)**2 + (calEMGData.mvc_gasflex_ta)**2))
        calEMGData.theta_gas = float(np.arctan(calEMGData.m_gas))
        calEMGData.mag_max_ta = float(np.sqrt((calEMGData.mvc_taflex_gas)**2 + (calEMGData.mvc_taflex_ta)**2))
        calEMGData.theta_ta = float(np.arctan(calEMGData.m_ta))

    print('Theta',calEMGData.theta_ta,'Data Type:',type(calEMGData.theta_ta))
    print('M_0',calEMGData.m_0,'Data Type:',type(calEMGData.m_0))

    # Let User Specify if Calibration Data Should Be Saved
    storeCheck = input('Store calibration data in .yaml file as well? [y/n]: ')

    # If Save Flag Set, Store Calibration Data
    if storeCheck in 'yes':

        # Store Calibration Date for Future Use
        calEMGDump(calEMGData, filename)
        print('Data stored in ' + filename + '...')

    else:

        print('Data not stored in ' + filename + '...')

    # Return Calibration Data
    return calEMGData
