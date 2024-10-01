from typing import Any, Callable, Union

import time
from dataclasses import dataclass
from enum import Enum

import numpy as np
from adc import ADS131M0x
from base import ADCBase, LoadcellBase


class SRILoadcell_ADC(LoadcellBase):

    def __init__(
        self,
        matrix=None,
        excitation_voltage=5,
        **kwargs,
    ):
        """Initializes Sunrise Instruments Loadcell using ADC"""
        self.matrix = matrix
        self._excitation_voltage = excitation_voltage

        self._adc = ADS131M0x(**kwargs)

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
        self._data = np.asarray(self.matrix) @ coupled

    @property
    def adc(self) -> ADCBase:
        """Returns the ADS131M0x object of the Loadcell"""
        return self._adc

    @property
    def is_streaming(self) -> bool:
        """Returns whether the ADC of the Loadcell is streaming data or not"""
        return (bool)(self.adc.is_streaming)

    @property
    def data(self):
        return self._data

    @property
    def fx(self) -> float:
        """Returns the force in the x direction of the loadcell"""
        return (float)(self.data[0])

    @property
    def fy(self) -> float:
        """Returns the force in the y direction of the loadcell"""
        return (float)(self.data[1])

    @property
    def fz(self) -> float:
        """Returns the force in the z direction of the loadcell"""
        return (float)(self.data[2])

    @property
    def mx(self) -> float:
        """Returns the moment in the x direction of the loadcell"""
        return (float)(self.data[3])

    @property
    def my(self) -> float:
        """Returns the moment in the y direction of the loadcell"""
        return (float)(self.data[4])

    @property
    def mz(self) -> float:
        """Returns the moment in the z direction of the loadcell"""
        return (float)(self.data[5])
