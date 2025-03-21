"""
Module for the RobotBase abstract class.

This module defines an abstract base class, `RobotBase`, that provides a template for a robot
system integrating actuators and sensors. The class supports context management and defines
abstract methods for starting, stopping, and updating the robot's components.

Type Parameters:
    TActuator: A type that must be a subclass of ActuatorBase.
    TSensor: A type that must be a subclass of SensorBase.
"""

from abc import ABC, abstractmethod
from typing import Any, Generic, TypeVar

from opensourceleg.actuators.base import ActuatorBase
from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import SensorBase

TActuator = TypeVar("TActuator", bound=ActuatorBase)
TSensor = TypeVar("TSensor", bound=SensorBase)


class RobotBase(ABC, Generic[TActuator, TSensor]):
    """
    Abstract base class representing a robot composed of actuators and sensors.

    This class provides the basic structure for a robot, including methods to start, stop,
    and update its components. It also supports context management so that the robot
    can be used within a with-statement to automatically start and stop its components.

    Attributes:
        actuators (dict[str, TActuator]): A dictionary mapping actuator names to actuator instances.
        sensors (dict[str, TSensor]): A dictionary mapping sensor names to sensor instances.
    """

    def __init__(
        self,
        tag: str,
        actuators: dict[str, TActuator],
        sensors: dict[str, TSensor],
    ) -> None:
        """
        Initialize the RobotBase instance.

        Args:
            tag (str): A unique identifier for the robot.
            actuators (dict[str, TActuator]): A dictionary of actuators keyed by their names.
            sensors (dict[str, TSensor]): A dictionary of sensors keyed by their names.
        """
        self._tag = tag
        self.actuators: dict[str, TActuator] = actuators
        self.sensors: dict[str, TSensor] = sensors

    def __enter__(self) -> "RobotBase":
        """
        Enter the runtime context for the robot.

        This method starts all actuators and sensors and returns the robot instance.

        Returns:
            RobotBase: The current robot instance.

        Example:
            >>> with MyRobot() as robot:
            ...     robot.update()
        """
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """
        Exit the runtime context for the robot.

        This method stops all actuators and sensors.

        Args:
            exc_type (Any): The exception type, if an exception occurred.
            exc_val (Any): The exception value, if an exception occurred.
            exc_tb (Any): The traceback, if an exception occurred.
        """
        self.stop()

    @abstractmethod
    def start(self) -> None:
        """
        Start all actuators and sensors.

        For each actuator in the actuators dictionary, a debug message is logged and its start method is called.
        Similarly, for each sensor in the sensors dictionary, a debug message is logged and its start method is called.

        Returns:
            None

        Example:
            >>> robot = MyRobot()
            >>> robot.start()
        """
        for actuator in self.actuators.values():
            LOGGER.debug(f"Calling start method of {actuator.tag}")
            actuator.start()

        for sensor in self.sensors.values():
            LOGGER.debug(f"Calling start method of {sensor}")
            sensor.start()

    @abstractmethod
    def stop(self) -> None:
        """
        Stop all actuators and sensors.

        For each actuator in the actuators dictionary, a debug message is logged and its stop method is called.
        Similarly, for each sensor in the sensors dictionary, a debug message is logged and its stop method is called.

        Returns:
            None

        Example:
            >>> robot = MyRobot()
            >>> robot.start()
            ... # Do something with the robot
            >>> robot.stop()
        """
        for actuator in self.actuators.values():
            LOGGER.debug(f"Calling stop method of {actuator.tag}")
            actuator.stop()

        for sensor in self.sensors.values():
            LOGGER.debug(f"Calling stop method of {sensor}")
            sensor.stop()

    @abstractmethod
    def update(self) -> None:
        """
        Update all actuators and sensors.

        This method calls the update method for each actuator and sensor to refresh their state.

        Returns:
            None

        Example:
            >>> robot = MyRobot()
            >>> robot.start()
            >>> robot.update()
        """
        for actuator in self.actuators.values():
            actuator.update()

        for sensor in self.sensors.values():
            sensor.update()

    @property
    def tag(self) -> str:
        """
        Get the unique identifier (tag) of the robot.

        Returns:
            str: The robot's tag.

        Example:
            >>> robot = MyRobot()
            >>> robot.tag
            "my_robot"
        """
        return self._tag

    # Example of piping values from the actuators to custom @property methods:
    # @property
    # def knee(self) -> TActuator:
    #     return self.actuators["knee"]


if __name__ == "__main__":
    pass
