from abc import ABC, abstractmethod
from functools import wraps
from typing import Any, Callable, ClassVar

from opensourceleg.common.offline import OfflineMixin


class SensorNotStreamingException(Exception):
    """
    Exception raised when an operation is attempted on a sensor that is not streaming.

    This exception indicates that the sensor is not actively streaming data.
    """

    def __init__(self, sensor_name: str = "Sensor") -> None:
        """
        Initialize the SensorNotStreamingException.

        Args:
            sensor_name (str, optional): The name or identifier of the sensor. Defaults to "Sensor".
        """
        super().__init__(
            f"{sensor_name} is not streaming, please ensure that the connections are intact, "
            f"power is on, and the start method is called."
        )


def check_sensor_stream(func: Callable) -> Callable:
    """
    Decorator to ensure that a sensor is streaming before executing the decorated method.

    If the sensor is not streaming, a SensorNotStreamingException is raised.

    Args:
        func (Callable): The sensor method to be wrapped.

    Returns:
        Callable: The wrapped method that checks streaming status before execution.
    """

    @wraps(func)
    def wrapper(self: Any, *args: Any, **kwargs: Any) -> Any:
        # TODO: This could be a generic type that points to actuator, sensor, etc.
        if not self.is_streaming:
            raise SensorNotStreamingException(sensor_name=self.__repr__())
        return func(self, *args, **kwargs)

    return wrapper


class SensorBase(OfflineMixin, ABC):
    """
    Abstract base class for sensors.

    Defines the common interface for sensors including starting, stopping,
    updating, and streaming status.
    """

    # Base sensor offline configuration
    _OFFLINE_METHODS: ClassVar[list[str]] = ["start", "stop", "update"]
    _OFFLINE_PROPERTIES: ClassVar[list[str]] = ["data", "is_streaming"]
    _OFFLINE_PROPERTY_DEFAULTS: ClassVar[dict[str, Any]] = {
        "data": None,
        "is_streaming": True,
    }

    def __init__(
        self,
        tag: str,
        offline: bool = False,
        **kwargs: Any,
    ) -> None:
        self._tag = tag
        # Initialize OfflineMixin first
        super().__init__(offline=offline, **kwargs)

    def __repr__(self) -> str:
        """
        Return a string representation of the sensor.

        Returns:
            str: A string identifying the sensor class.
        """
        return f"{self.tag}[{self.__class__.__name__}]"

    @property
    @abstractmethod
    def data(self) -> Any:
        """
        Get the sensor data.

        Returns:
            Any: The current data from the sensor.
        """
        pass

    @abstractmethod
    def start(self) -> None:
        """
        Start the sensor streaming.

        Implementations should handle initializing the sensor and beginning data acquisition.
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """
        Stop the sensor streaming.

        Implementations should handle gracefully shutting down the sensor.
        """
        pass

    @abstractmethod
    def update(self) -> None:
        """
        Update the sensor state or data.

        Implementations should refresh or poll the sensor data as needed.
        """
        pass

    def __enter__(self) -> "SensorBase":
        """
        Enter the runtime context for the sensor.

        This method calls start() and returns the sensor instance.

        Returns:
            SensorBase: The sensor instance.
        """
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> None:
        """
        Exit the runtime context for the sensor.

        This method calls stop() to shut down the sensor.

        Args:
            exc_type (Any): Exception type if raised.
            exc_value (Any): Exception value if raised.
            traceback (Any): Traceback if an exception occurred.
        """
        self.stop()

    @property
    @abstractmethod
    def is_streaming(self) -> bool:
        """
        Check if the sensor is currently streaming.

        Returns:
            bool: True if the sensor is streaming, False otherwise.
        """
        pass

    @property
    def tag(self) -> str:
        """
        Get the sensor tag.

        Returns:
            str: The unique identifier for the sensor.

        Examples:
            >>> sensor.tag
            "sensor1"
        """
        return self._tag


class ADCBase(SensorBase, ABC):
    """
    Abstract base class for ADC (Analog-to-Digital Converter) sensors.

    ADC sensors are used to convert analog signals into digital data.
    """

    # ADC-specific offline configuration
    _OFFLINE_METHODS: ClassVar[list[str]] = [*SensorBase._OFFLINE_METHODS, "reset", "calibrate"]

    def __init__(self, tag: str, offline: bool = False, **kwargs: Any) -> None:
        """
        Initialize the ADC sensor.
        """
        super().__init__(tag=tag, offline=offline, **kwargs)

    def __repr__(self) -> str:
        """
        Return a string representation of the ADC sensor.

        Returns:
            str: "ADCBase"
        """
        return "ADCBase"

    def reset(self) -> None:
        """
        Reset the ADC sensor.

        Implementations should clear any stored state or calibration.
        """
        pass

    def calibrate(self) -> None:
        """
        Calibrate the ADC sensor.

        Implementations should perform necessary calibration procedures.
        """
        pass


class EncoderBase(SensorBase, ABC):
    """
    Abstract base class for encoder sensors.

    Encoders are used to measure position and velocity.
    """

    # Encoder-specific offline configuration
    _OFFLINE_PROPERTIES: ClassVar[list[str]] = [*SensorBase._OFFLINE_PROPERTIES, "position", "velocity"]
    _OFFLINE_PROPERTY_DEFAULTS: ClassVar[dict[str, Any]] = {
        **SensorBase._OFFLINE_PROPERTY_DEFAULTS,
        "position": 0.0,
        "velocity": 0.0,
    }

    def __init__(
        self,
        tag: str,
        offline: bool = False,
        **kwargs: Any,
    ) -> None:
        """
        Initialize the encoder sensor.
        """
        super().__init__(tag=tag, offline=offline, **kwargs)

    def __repr__(self) -> str:
        """
        Return a string representation of the encoder sensor.

        Returns:
            str: "EncoderBase"
        """
        return "EncoderBase"

    @property
    @abstractmethod
    def position(self) -> float:
        """
        Get the current encoder position.

        Returns:
            float: The current position value.
        """
        pass

    @property
    @abstractmethod
    def velocity(self) -> float:
        """
        Get the current encoder velocity.

        Returns:
            float: The current velocity value.
        """
        pass


class LoadcellBase(SensorBase, ABC):
    """
    Abstract base class for load cell sensors.

    Load cells are used to measure forces and moments.
    """

    # Load cell-specific offline configuration
    _OFFLINE_METHODS: ClassVar[list[str]] = [*SensorBase._OFFLINE_METHODS, "calibrate", "reset"]
    _OFFLINE_PROPERTIES: ClassVar[list[str]] = [
        *SensorBase._OFFLINE_PROPERTIES,
        "fx",
        "fy",
        "fz",
        "mx",
        "my",
        "mz",
        "is_calibrated",
    ]
    _OFFLINE_PROPERTY_DEFAULTS: ClassVar[dict[str, Any]] = {
        **SensorBase._OFFLINE_PROPERTY_DEFAULTS,
        "fx": 0.0,
        "fy": 0.0,
        "fz": 0.0,  # Could be -9.81 * mass for gravity simulation
        "mx": 0.0,
        "my": 0.0,
        "mz": 0.0,
        "is_calibrated": True,
    }

    def __init__(self, tag: str, offline: bool = False, **kwargs: Any) -> None:
        """
        Initialize the load cell sensor.
        """
        super().__init__(tag=tag, offline=offline, **kwargs)

    def __repr__(self) -> str:
        """
        Return a string representation of the load cell sensor.

        Returns:
            str: "LoadcellBase"
        """
        return "LoadcellBase"

    @abstractmethod
    def calibrate(self) -> None:
        """
        Calibrate the load cell sensor.

        Implementations should perform the calibration procedure to ensure accurate readings.
        """
        pass

    @abstractmethod
    def reset(self) -> None:
        """
        Reset the load cell sensor.

        Implementations should reset the sensor state and any calibration data.
        """
        pass

    @property
    @abstractmethod
    def fx(self) -> float:
        """
        Get the force along the x-axis.

        Returns:
            float: The force measured along the x-axis.
        """
        pass

    @property
    @abstractmethod
    def fy(self) -> float:
        """
        Get the force along the y-axis.

        Returns:
            float: The force measured along the y-axis.
        """
        pass

    @property
    @abstractmethod
    def fz(self) -> float:
        """
        Get the force along the z-axis.

        Returns:
            float: The force measured along the z-axis.
        """
        pass

    @property
    @abstractmethod
    def mx(self) -> float:
        """
        Get the moment about the x-axis.

        Returns:
            float: The moment measured about the x-axis.
        """
        pass

    @property
    @abstractmethod
    def my(self) -> float:
        """
        Get the moment about the y-axis.

        Returns:
            float: The moment measured about the y-axis.
        """
        pass

    @property
    @abstractmethod
    def mz(self) -> float:
        """
        Get the moment about the z-axis.

        Returns:
            float: The moment measured about the z-axis.
        """
        pass

    @property
    @abstractmethod
    def is_calibrated(self) -> bool:
        """
        Check if the load cell sensor is calibrated.

        Returns:
            bool: True if calibrated, False otherwise.
        """
        pass


class IMUBase(SensorBase, ABC):
    """
    Abstract base class for Inertial Measurement Unit (IMU) sensors.

    IMUs typically provide acceleration and gyroscopic data.
    """

    # IMU-specific offline configuration
    _OFFLINE_PROPERTIES: ClassVar[list[str]] = [
        *SensorBase._OFFLINE_PROPERTIES,
        "acc_x",
        "acc_y",
        "acc_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
    ]
    _OFFLINE_PROPERTY_DEFAULTS: ClassVar[dict[str, Any]] = {
        **SensorBase._OFFLINE_PROPERTY_DEFAULTS,
        "acc_x": 0.0,
        "acc_y": 0.0,
        "acc_z": 0.0,  # Gravity in z-axis when stationary
        "gyro_x": 0.0,
        "gyro_y": 0.0,
        "gyro_z": 0.0,
    }

    def __init__(self, tag: str, offline: bool = False, **kwargs: Any) -> None:
        """
        Initialize the IMU sensor.
        """
        super().__init__(tag=tag, offline=offline, **kwargs)

    @property
    @abstractmethod
    def acc_x(self) -> float:
        """
        Get the estimated linear acceleration along the x-axis.

        Returns:
            float: Acceleration in m/s^2 along the x-axis.
        """
        pass

    @property
    @abstractmethod
    def acc_y(self) -> float:
        """
        Get the estimated linear acceleration along the y-axis.

        Returns:
            float: Acceleration in m/s^2 along the y-axis.
        """
        pass

    @property
    @abstractmethod
    def acc_z(self) -> float:
        """
        Get the estimated linear acceleration along the z-axis.

        Returns:
            float: Acceleration in m/s^2 along the z-axis.
        """
        pass

    @property
    @abstractmethod
    def gyro_x(self) -> float:
        """
        Get the gyroscopic measurement along the x-axis.

        Returns:
            float: Angular velocity in rad/s along the x-axis.
        """
        pass

    @property
    @abstractmethod
    def gyro_y(self) -> float:
        """
        Get the gyroscopic measurement along the y-axis.

        Returns:
            float: Angular velocity in rad/s along the y-axis.
        """
        pass

    @property
    @abstractmethod
    def gyro_z(self) -> float:
        """
        Get the gyroscopic measurement along the z-axis.

        Returns:
            float: Angular velocity in rad/s along the z-axis.
        """
        pass


if __name__ == "__main__":
    pass
