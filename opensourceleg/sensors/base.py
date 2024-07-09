"""Sensor Interface generalized
06/2024
"""

from abc import ABC, abstractmethod

import numpy as np


class StrainAmp(ABC):
    @abstractmethod
    def __init__(self, bus, addr) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

    def __repr__(self) -> str:
        return f"StrainAmp"


class Loadcell(ABC):
    @abstractmethod
    def __init__(self) -> None:
        pass

    # @abstractmethod
    # def reset(self):
    #     pass

    @abstractmethod
    def update(self, loadcell_zero=None) -> None:
        pass

    # @abstractmethod
    # def initialize(self, number_of_iterations) -> None:
    #     pass

    def __repr__(self) -> str:
        return f"Loadcell"


class SensorIMU(ABC):
    @abstractmethod
    def __init__(
        self,
        port,
        baud_rate,
        timeout,
        sample_rate,
    ):
        pass

    @abstractmethod
    def start_streaming(self):
        pass

    @abstractmethod
    def stop_streaming(self):
        pass

    @abstractmethod
    def get_data(self):
        """Should be erased and merged with update method"""
        pass

    @property
    @abstractmethod
    def is_streaming(self):
        pass


class Encoder(ABC):
    @abstractmethod
    def __init__(self) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass
