import numpy as np
from adc import ADS131M0x
from base import LoadcellBase


class SRILoadcell(LoadcellBase):

    def __init__(self, matrix=None, **kwargs):
        self._adc = ADS131M0x(**kwargs)
        self.matrix = matrix
        self.zero_offset = [0] * self.adc._num_channels

    @property
    def adc(self):
        return self._adc

    def __repr__(self) -> str:
        return f"Loadcell"

    @property
    def is_streaming(self) -> bool:
        return self.adc._streaming

    def start(self):
        self.adc.start()

    def stop(self):
        self.adc.stop()

    def update(self):
        self.adc.update()
        # to mV/V
        coupled = [0] * self.adc._num_channels
        for i in range(0, self.adc._num_channels):
            coupled[i] = self.adc.data[i] * 1000 / (1.2 * self.adc.gain[i])
        self.data = np.transpose(a=self.matrix.dot(b=np.transpose(a=coupled)))

    def reset(self):
        self.adc.reset()

    @property
    def fx(self):
        return self.data[0]

    @property
    def fy(self):
        return self.data[1]

    @property
    def fz(self):
        return self.data[2]

    @property
    def mx(self):
        return self.data[3]

    @property
    def my(self):
        return self.data[4]

    @property
    def mz(self):
        return self.data[5]

    @property
    def ext1(self):
        return self.data[6]

    @property
    def ext2(self):
        return self.data[7]
