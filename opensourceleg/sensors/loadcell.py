from typing import Any, Callable, Union

import time
from dataclasses import dataclass
from enum import Enum

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
        """Initializes Sunrise Instruments Loadcell"""
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

    def reset(self) -> None:
        """Reset ADC register values to defaults"""
        self.adc.reset()

    def start(self) -> None:
        """Start streaming ADC data"""
        self.adc.start()
        self.adc.calibrate()

    def stop(self) -> None:
        """Stop streaming ADC data"""
        self.adc.stop()

    def update(self) -> None:
        """Read most recent ADC data"""
        self.adc.update()
        coupled = np.asarray(self.adc.data) / (
            self._excitation_voltage * np.asarray(self.adc.gains)
        )
        self.data = np.asarray(self.matrix) @ coupled

    @property
    def adc(self) -> ADCBase:
        return self._adc

    @property
    def is_streaming(self) -> bool:
        return self.adc._streaming

    @property
    def fx(self) -> float:
        return self.data[0]

    @property
    def fy(self) -> float:
        return self.data[1]

    @property
    def fz(self) -> float:
        return self.data[2]

    @property
    def mx(self) -> float:
        return self.data[3]

    @property
    def my(self) -> float:
        return self.data[4]

    @property
    def mz(self) -> float:
        return self.data[5]
