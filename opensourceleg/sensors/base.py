from abc import ABC, abstractmethod

import numpy as np


class Encoder(ABC):
    def __init__(self) -> None:
        pass

    def __repr__(self) -> str:
        return f"Encoder"

    @abstractmethod
    def start(self) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass

    @abstractmethod
    def calibrate(self) -> None:
        pass

    @abstractmethod
    def reset(self) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

    @abstractmethod
    @property
    def position(self) -> float:
        pass

    @abstractmethod
    @property
    def velocity(self) -> float:
        pass

    @abstractmethod
    @property
    def is_streaming(self) -> bool:
        pass


class Loadcell(ABC):
    def __init__(self) -> None:
        pass

    def __repr__(self) -> str:
        return f"Loadcell"

    @abstractmethod
    def start(self) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass

    @abstractmethod
    def calibrate(self) -> None:
        pass

    @abstractmethod
    def reset(self) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()


class IMU(ABC):
    def __init__(self) -> None:
        pass

    def __repr__(self) -> str:
        return f"Inertial Measurement Unit"

    @abstractmethod
    def start(self) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass

    @abstractmethod
    def calibrate(self) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

    @abstractmethod
    @property
    def acc_x(self) -> float:
        pass

    @abstractmethod
    @property
    def acc_y(self) -> float:
        pass

    @abstractmethod
    @property
    def acc_z(self) -> float:
        pass

    @abstractmethod
    @property
    def gyro_x(self) -> float:
        pass

    @abstractmethod
    @property
    def gyro_y(self) -> float:
        pass

    @abstractmethod
    @property
    def gyro_z(self) -> float:
        pass
