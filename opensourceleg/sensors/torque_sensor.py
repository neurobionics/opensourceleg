import busio
import board
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
# import csv
# import traceback
# import sys
import numpy as np
from time import time, sleep

from opensourceleg.sensors.base import LoadcellBase

# class AdcManager(object):
#     def __init__(self, csv_file_name=None):
#         self.i2c = busio.I2C(board.SCL, board.SDA)
#         self.ads = ADS.ADS1115(self.i2c, data_rate=860)
#         self.chan = AnalogIn(self.ads, ADS.P0)
#         self.save_csv = not (csv_file_name is None)
#         self.csv_file_name = csv_file_name
#         self.csv_file = None
#         self.csv_writer = None
#         self.volts = -42.0 # error code


#     def __enter__(self):
#         self.start()
#         if self.save_csv:
#             with open(self.csv_file_name,'w') as fd:
#                 writer = csv.writer(fd)
#                 writer.writerow(["pi_time", "voltage", "test_duration"])
#             self.csv_file = open(self.csv_file_name,'a').__enter__()
#             self.csv_writer = csv.writer(self.csv_file)
#         return self

#     def __exit__(self, etype, value, tb):
#         """ Closes the file properly """
#         self.stop()
#         if self.save_csv:
#             self.csv_file.__exit__(etype, value, tb)
#         if not (etype is None):
#             traceback.print_exception(etype, value, tb)


#     def update_adc(self):
#         t0=time()
#         self.volts = self.chan.voltage
#         dur = time()-t0
#         if self.save_csv:
#             self.csv_writer.writerow([time(),self.volts, dur])
#         return self.volts

class Futek(LoadcellBase):
    def __init__(self, torque=-42.0, bias=0.0, torque_rating=100.0, wait_time=5.0, voltage=5.0, **kwargs):
        self.torque = torque
        self.bias = bias
        self.torque_rating = torque_rating
        self.wait_time = wait_time
        self.voltage = voltage
        self.volts = -42.0 # error code
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(self.i2c, data_rate=860)
        self.chan = AnalogIn(self.ads, ADS.P0)
        self._is_calibrated = False
        self._is_streaming = False

    def start(self):
        self._is_streaming = True

    def stop(self):
        self._is_streaming = False

    def update(self):
        self.volts = self.chan.voltage
        torque_rating = self.torque_rating # 100 Nm = 5V
        self.torque = (self.volts-(self.voltage/2)-self.bias)/(self.voltage/2)*torque_rating

    def reset(self):
        self._is_calibrated = False


    def calibrate(self,Calibrate=True,SaveCal=False):
        if Calibrate:
            self.update()
            voltage_cal = []
            print("calibrating loadcell")
            start_time = time()
            while time() - start_time < self.wait_time:
                self.update()
                voltage_cal.append(self.volts)

            avg_volt = np.mean(np.array(voltage_cal))
            bias = avg_volt - self.voltage/2
            print("Bias: {} V".format(bias))
            self.update()
            self.bias = bias
            if SaveCal:
                np.save(file=f"./futek_offset.npy", arr=bias)
        else:
            bias = np.load('futek_offset.npy')
            print("Setting Bias: {} V".format(bias))
            self.bias = bias

        self._is_calibrated = True
        return bias

    def set_bias(self,bias):
        self.bias = bias
        print("Bias set to: {} V".format(bias))

    @property
    def mx(self):
        return self.torque

    @property
    def my(self):
        print("Method not implemented")

    @property
    def mz(self):
        print("Method not implemented")

    @property
    def fx(self):
        print("Method not implemented")

    @property
    def fy(self):
        print("Method not implemented")

    @property
    def fz(self):
        print("Method not implemented")

    @property
    def data(self):
        print("Method not implemented")

    @property
    def is_streaming(self) -> bool:
        return self._is_streaming

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated
