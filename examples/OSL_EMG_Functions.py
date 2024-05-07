############################### PACKAGE IMPORTS ################################

# Imports for Standard Python
from time import sleep, time
import datetime
import os, sys
import math
import numpy as np
import yaml

from dataclasses import dataclass

# Imports for Force/EMG Sensors
from signal import signal, SIGINT
import spidev


class EMG:
	def __init__(self, a_channel: int = 2,
		b_channel: int = 3,
		time_window: float = 0.1,
		time_step: float = 0.001,
		ADC_offset_a: int = 510,
		ADC_offset_b: int = 515):

		self.a_channel = a_channel
		self.b_channel = b_channel
		self.ADC_offset_a = ADC_offset_a
		self.ADC_offset_b = ADC_offset_b
		self.time_window = time_window
		self.time_step = time_step

		self.a_filter = moving_average(time_window/time_step, 0)
		self.b_filter = moving_average(time_window/time_step, 0)

		self.calEMGData = None

		self.spi = spidev.SpiDev()
		self.spi.open(0, 0)
		self.spi.max_speed_hz=1000000


		


############################# FUNCTION DEFINITIONS #############################

	def readadc(self, adcChan):

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
		spiDat = self.spi.xfer2([1, 8 + adcChan << 4, 0])
		data = ((spiDat[1] & 3) << 8) + spiDat[2]

		# Return Force Sensor Data
		return data



	def rectify_emg(self, raw, baseline):
		rectified = abs(raw - baseline)
		return rectified


	def find_slope(self, stdev_1, stdev_2, flex_time, direction='plantarflex', intensity=100):
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
			# reading_4 = np.zeros(int(self.filter_time_window/self.time_step))
			# reading_5 = np.zeros(int(self.filter_time_window/self.time_step))
			emg_avg_prev_gas = 0
			emg_avg_prev_ta = 0
			while time() < start_time + flex_time:
				emg_avg_gas = self.update('GAS')
				# emg_avg_gas, reading_4 = self.filter_emg(emg_raw_rect_gas, filter_time_window, self.time_step, reading_4)
				emg_avg_ta = self.update('TA')
				# emg_avg_ta, reading_5 = self.filter_emg(emg_raw_rect_ta, filter_time_window, self.time_step, reading_5)

				all_gas = np.append(all_gas, [emg_avg_gas])
				all_ta = np.append(all_ta, [emg_avg_ta])
				if emg_avg_ta != 0:
					if abs(emg_avg_ta) > 2*abs(stdev_2) or abs(emg_avg_gas) > 2*abs(stdev_1):
						m = (emg_avg_gas)/(emg_avg_ta)
						all_m = np.append(all_m, [m])

				print('Calibrating co-contraction profile for ' + direction + 'ion...')
				print(emg_avg_gas, emg_avg_ta)
				sleep(self.time_step)
			m_avg = np.mean(all_m)
			max_gas = np.amax(all_gas) # these are already rectified values
			max_ta = np.amax(all_ta)
			print('Average m: ' + str(m_avg) + ' max_GAS: ' + str(max_gas) + ' max_TA: ' + str(max_ta))
			readyx = input('Are you happy with the calibration results? [y/n] (Enter Stop to Exit Script): ')
			if readyx != 'n' and readyx != 'y' and readyx != 'Stop':
				readyx = input('Please enter either y, n, or Stop: ')
		return float(m_avg), float(max_gas), float(max_ta)


	def noise_level(self, cal_time):
		ready2 = 'n'
		input('Please rest your muscle and stay inactive. When ready hit Enter.')
		while ready2 == 'n':
			start_time = time()
			cal_values_2 = []
			cal_values_3 = []
			self.a_filter.reset(self.ADC_offset_a)
			self.b_filter.reset(self.ADC_offset_b)
			while time() < start_time + cal_time:
				emg_raw_value_2 = self.readadc(self.a_channel)
				emg_avg_2_base = self.a_filter.filter(emg_raw_value_2)

				emg_raw_value_3 = self.readadc(self.b_channel)
				emg_avg_3_base = self.b_filter.filter(emg_raw_value_3)

				cal_values_2 = np.append(cal_values_2, [emg_avg_2_base])
				cal_values_3 = np.append(cal_values_3, [emg_avg_3_base])

				#print('Calibrating Baseline!')
				print(emg_avg_2_base, emg_avg_3_base)
				sleep(self.time_step)

			baseline_1 = np.mean(cal_values_2)
			baseline_2 = np.mean(cal_values_3)
			stdev_1 = np.std(cal_values_2)
			stdev_2 = np.std(cal_values_3)
			print('Average ch2: ' + str(baseline_1) + ' Standard Deviation: ' + str(stdev_1))
			print('Average ch3: ' + str(baseline_2) + ' Standard Deviation: ' + str(stdev_2))

			ready2 = input('Are you happy with the baseline calibration value? [y/n] (Enter Stop to Exit Script): ')
			if ready2 != 'n' and ready2 != 'y':
				ready2 = input('Please enter either y, or n: ')

		calEMGData = CalEMGDataSingle()
		calEMGData.baseline_1 = float(baseline_1)
		calEMGData.baseline_2 = float(baseline_2)
		calEMGData.stdev_1 = float(stdev_1)
		calEMGData.stdev_2 = float(stdev_2)

		return calEMGData, ready2

	def update(self, muscle):
		if muscle == 'GAS':
			channel = self.a_channel
			baseline = self.calEMGData.baseline_1
			mov_avg_filter = self.a_filter
		elif muscle == 'TA':
			channel = self.b_channel
			baseline = self.calEMGData.baseline_2
			mov_avg_filter = self.b_filter
		else:
			raise ValueError("Unknown Muscle")
		raw_EMG = self.readadc(channel) # Read in Raw EMG for muscle 1
		# rectify and smooth the EMG:
		emg_raw_rect = self.rectify_emg(raw_EMG, baseline)
		emg_avg = mov_avg_filter.filter(emg_raw_rect)
		return emg_avg



	def calEMGDump(self, calData, filename):

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


	def calEMGLoad(self, filename):

		'''
		Function for loading calibration data from the proper yaml file
		Inputs:
			dev - Calibration data joint (0 for Knee, 1 for Ankle)
		Outputs:
			calEMGData - Class structure storing the calibration data
		'''

		# Open Appropriate File Based on Device ID
		try:
			pFile = open(filename)

			# Load Data From File and Close
			calEMGData = yaml.load(pFile, Loader = yaml.Loader)
			pFile.close()
		except:
			print("No calibration file found at " + filename)
			print("Creating new file")
			calEMGData = self.emgCalibration(filename=filename)

		# Return Calibration Data
		self.calEMGData = calEMGData

	def emgCalibration(self, rest_time = 5, flex_time = 1, filename = './cal.yaml'):

		'''
		Function for calibrating the ankle module and storing the calibration data in a class object. This form uses the internal motor encoder
		Inputs:
			calEMGData - Class object to store calibration data in
		Outputs:
			calEMGData - Class object to store calibration data in
		'''

		# Take a baseline reading of noise level for the emg sensors
		calEMGdata_temp, ready2 = self.noise_level(rest_time)
		self.calEMGData = calEMGdata_temp

		# ready3 = 'y'  # initialize to 'n' to run through

		# Calibrate the MVC and cocontraction slope data
		if ready2 == 'y': # and ready3 == 'y':
			sleep(0.5)

			# Ask for MVC for gastroc (first) and TA (second). Calculate the cocontraction slope and MVC value for that respective muscle.
			self.calEMGData.m_gas, self.calEMGData.MVA_GAS, _ = self.find_slope(self.calEMGData.stdev_1, self.calEMGData.stdev_2, flex_time, direction='plantarflex', intensity=100)
			self.calEMGData.m_ta, _, self.calEMGData.MVA_TA = self.find_slope(self.calEMGData.stdev_1, self.calEMGData.stdev_2, flex_time, direction='dorsiflex', intensity=100)

			# establish a bisecting line between the 2 slopes
			self.calEMGData.m_0 = float(np.tan((np.arctan(self.calEMGData.m_gas) + np.arctan(self.calEMGData.m_ta))/2))

			# establish the MVC value for the undesired muscle by taking the MVC value measured for the desired muscle and placing it in line with the slope.
			# self.calEMGData.MVA_TA = self.calEMGData.MVA_GAS/self.calEMGData.m_gas
			# self.calEMGData.mvc_taflex_gas = self.calEMGData.MVA_TA*self.calEMGData.m_ta

			# Find the magnitude and angle of the vector going from the baseline to the MVC value
			# self.calEMGData.mag_max_gas = float(np.sqrt((calEMGData.MVA_GAS)**2 + (calEMGData.MVA_TA)**2))
			# self.calEMGData.theta_gas = float(np.arctan(calEMGData.m_gas))
			# self.calEMGData.mag_max_ta = float(np.sqrt((calEMGData.mvc_taflex_gas)**2 + (calEMGData.mvc_taflex_ta)**2))
			# self.calEMGData.theta_ta = float(np.arctan(calEMGData.m_ta))

		# print('Theta',self.calEMGData.theta_ta,'Data Type:',type(self.calEMGData.theta_ta))
		print('M_0',self.calEMGData.m_0,'Data Type:',type(self.calEMGData.m_0))

		# Let User Specify if Calibration Data Should Be Saved
		storeCheck = input('Store calibration data in .yaml file as well? [y/n]: ')

		# If Save Flag Set, Store Calibration Data
		if storeCheck in 'yes':

			# Store Calibration Date for Future Use
			self.calEMGDump(self.calEMGData, filename)
			print('Data stored in ' + filename + '...')

		else:

			print('Data not stored in ' + filename + '...')

@dataclass
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

	baseline_1: float = 0.0
	baseline_2: float = 0.0
	stdev_1: float = 0.0
	stdev_2: float = 0.0
	m_gas: float = 0.0
	m_ta: float = 0.0
	m_0: float = 0.0
	MVA_GAS: float = 0.0
	MVA_TA: float = 0.0

	# self.mag_max_gas = mag_max_gas
	# self.theta_gas = theta_gas
	# self.mag_max_ta = mag_max_ta
	# self.theta_ta = theta_ta
class moving_average:
	def __init__(
		self, 
		window: int = 10,
		initial_value: float = 0
	):
		self.window = int(window)
		self.reset(initial_value)
	def reset(self, value):
		self.vec = np.ones(self.window) * value
	def filter(self, raw):

		self.vec = np.append(self.vec, [raw])
		self.vec = np.delete(self.vec,0)

		emg_avg = np.sum(self.vec)/self.window

		return emg_avg