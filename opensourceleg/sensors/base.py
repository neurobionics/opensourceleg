from abc import ABC, abstractmethod

import numpy as np


class SensorBase(ABC):
    def __init__(self) -> None:
        pass

    def __repr__(self) -> str:
        return f"SensorBase"

    @abstractmethod
    def start(self) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

    @property
    @abstractmethod
    def is_streaming(self) -> bool:
        pass


class EncoderBase(SensorBase, ABC):
    def __init__(self) -> None:
        super().__init__()

    def __repr__(self) -> str:
        return f"EncoderBase"

    @property
    @abstractmethod
    def position(self) -> float:
        pass

    @property
    @abstractmethod
    def velocity(self) -> float:
        pass


class LoadcellBase(SensorBase, ABC):
    def __init__(self) -> None:
        pass

    def __repr__(self) -> str:
        return f"LoadcellBase"

    @abstractmethod
    def calibrate(self) -> None:
        pass

    @abstractmethod
    def reset(self) -> None:
        pass

    @property
    @abstractmethod
    def fx(self) -> float:
        pass

    @property
    @abstractmethod
    def fy(self) -> float:
        pass

    @property
    @abstractmethod
    def fz(self) -> float:
        pass

    @property
    @abstractmethod
    def mx(self) -> float:
        pass

    @property
    @abstractmethod
    def my(self) -> float:
        pass

    @property
    @abstractmethod
    def mz(self) -> float:
        pass

    @property
    @abstractmethod
    def is_calibrated(self) -> bool:
        pass


class IMU(SensorBase, ABC):
    def __init__(self) -> None:
        pass

    def __repr__(self) -> str:
        return f"IMU"

    @property
    @abstractmethod
    def acc_x(self) -> float:
        pass

    @property
    @abstractmethod
    def acc_y(self) -> float:
        pass

    @property
    @abstractmethod
    def acc_z(self) -> float:
        pass

    @property
    @abstractmethod
    def gyro_x(self) -> float:
        pass

    @property
    @abstractmethod
    def gyro_y(self) -> float:
        pass

    @property
    @abstractmethod
    def gyro_z(self) -> float:
        pass


if __name__ == "__main__":
    pass
