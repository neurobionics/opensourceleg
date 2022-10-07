from mscl import mscl as ms
from time import sleep
import numpy as np
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# based on Kevin Best's LordIMU communication class
@dataclass
class IMUdataClass:
    angleX: float = 0
    angleY: float = 0
    angleZ: float = 0
    velocityX:  float = 0  
    velocityY: float = 0
    velocityZ: float = 0
    accelX: float = 0
    accelY: float = 0
    accelZ: float = 0
    IMUTimeSta: float = 0
    IMUFilterGPSTimeWeekNum: float = 0
    thighAngle_Sagi: float = 0
    thighAngle_Coro: float = 0
    thighAngle_Trans: float = 0

class IMU_LordMicrostrain:
    def __init__(self, IMU_PORT = r'/dev/ttyUSB0', BAUD = 921600, TIMEOUT = 500, sampleRate = 100):
        self.PORT = IMU_PORT
        self.BAUD = BAUD
        self.connection = ms.Connection.Serial(os.path.realpath(self.PORT), self.BAUD)
        self.IMU = ms.InertialNode(self.connection)
        self.TIMEOUT  = TIMEOUT                  #Timeout in (ms) to read the IMU
        sleep(0.5)

        # Configure data channels
        channels = ms.MipChannels()
        channels.append(ms.MipChannel(ms.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER, ms.SampleRate.Hertz(sampleRate)))
        channels.append(ms.MipChannel(ms.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE, ms.SampleRate.Hertz(sampleRate)))
        channels.append(ms.MipChannel(ms.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL, ms.SampleRate.Hertz(sampleRate)))
        channels.append(ms.MipChannel(ms.MipTypes.CH_FIELD_ESTFILTER_GPS_TIMESTAMP, ms.SampleRate.Hertz(sampleRate)))
        self.IMU.setActiveChannelFields(ms.MipTypes.CLASS_ESTFILTER, channels)
        self.IMU.enableDataStream(ms.MipTypes.CLASS_ESTFILTER)
        self.IMU.setToIdle()
        packets = self.IMU.getDataPackets(TIMEOUT)       # Clean the internal circular buffer.
        self.IMUdata = IMUdataClass()

    def startDataStream(self):
        self.IMU.resume()

    def stopDataStream(self):
        self.IMU.setToIdle()

    def getData(self):
        IMUPackets = self.IMU.getDataPackets(self.TIMEOUT)
        if(len(IMUPackets)):
            # Read all the information from the first packet as float.
            rawIMUData = {dataPoint.channelName(): dataPoint.as_float() for dataPoint in IMUPackets[-1].data()}
            self.IMUdata.angleX = rawIMUData['estRoll']
            self.IMUdata.angleY = rawIMUData['estPitch']
            self.IMUdata.angleZ = rawIMUData['estYaw']
            self.IMUdata.velocityX = rawIMUData['estAngularRateX']
            self.IMUdata.velocityY = rawIMUData['estAngularRateY']
            self.IMUdata.velocityZ = rawIMUData['estAngularRateZ']
            self.IMUdata.accelX = rawIMUData['estLinearAccelX']
            self.IMUdata.accelY = rawIMUData['estLinearAccelY']
            self.IMUdata.accelZ = rawIMUData['estLinearAccelZ']
            self.IMUdata.IMUTimeSta = rawIMUData['estFilterGpsTimeTow']
            self.IMUdata.IMUFilterGPSTimeWeekNum = rawIMUData['estFilterGpsTimeWeekNum']
            self.IMUdata.thighAngle_Sagi = self.IMUdata.angleX + np.deg2rad( 39.38 )
            self.IMUdata.thighAngle_Coro = self.IMUdata.angleY
            self.IMUdata.thighAngle_Trans = self.IMUdata.angleZ
        
        return self.IMUdata






