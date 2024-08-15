import numpy as np
from adc import ADS131M0x
from base import LoadcellBase, ADCBase


class SRILoadcell(LoadcellBase):

    def __init__(
        self,
        matrix=None,
        excitation_voltage=5,
        adc: ADCBase = None,
        adc_board="neurobionics",
        **kwargs,
    ):
        self.matrix = matrix
        self._excitation_voltage = excitation_voltage

        if adc is not None:
            self._adc = adc
        elif adc_board == "neurobionics":
            self._adc = ADS131M0x(**kwargs)
        elif adc_board == "dephy":
            raise NotImplementedError("Dephy board class is not supported currently")

    def __repr__(self) -> str:
        return f"Loadcell"

    """Reset ADC register values to defaults"""

    def reset(self):
        self.adc.reset()

    """Start streaming ADC data"""

    def start(self):
        self.adc.start()
        # self.adc.calibrate()

    """Stop streaming ADC data"""

    def stop(self):
        self.adc.stop()

    """Read most recent ADC data"""

    def update(self):
        self.adc.update()
        coupled = np.asarray(self.adc.data) / (
            self._excitation_voltage * np.asarray(self.adc.gains)
        )
        self.data = np.asarray(self.matrix) @ coupled

    @property
    def adc(self):
        return self._adc

    @property
    def is_streaming(self) -> bool:
        return self.adc._streaming

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
