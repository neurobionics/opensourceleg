import time

from opensourceleg.logging import Logger


class Sensor:
    """Example sensor class for demonstration."""

    def __init__(self):
        self.temperature = 20.0
        self.humidity = 0.5

    def update(self):
        """Simulate sensor updates."""
        self.temperature += 0.1
        self.humidity += 0.01


class Experiment:
    """Example experiment class demonstrating data collection."""

    def __init__(self):
        self.logger = Logger(
            log_path="./experiments", file_name=f"trial_{time.strftime('%Y%m%d_%H%M%S')}", buffer_size=5000
        )

        self.input = 0.0
        self.output = 0.0
        self.error = 0.0
        self.steps = 0
        self.max_steps = 5  # For demonstration

        # Track experimental variables
        self.logger.track_function(lambda: self.input, "Input")
        self.logger.track_function(lambda: self.output, "Output")
        self.logger.track_function(lambda: self.error, "Error")

    def is_complete(self):
        """Check if experiment is complete."""
        return self.steps >= self.max_steps

    def step(self):
        """Simulate one step of the experiment."""
        self.input += 0.1
        self.output = self.input * 2
        self.error = abs(self.output - self.input)
        self.steps += 1

    def run(self):
        """Run the experiment."""
        while not self.is_complete():
            self.step()
            self.logger.update()


def basic_variable_tracking():
    """Shows basic variable tracking functionality."""
    print("\n=== Basic Variable Tracking Example ===")

    logger = Logger(log_path="./logs", file_name="basic_variable_tracking")

    # Track a simple variable
    x = 42
    logger.track_function(lambda: x, "x_value")

    # Track class attributes
    class Robot:
        def __init__(self):
            self.position = 0.0
            self.velocity = 0.0

    robot = Robot()
    logger.track_attributes(robot, ["position", "velocity"])

    # Update logged values
    logger.update()  # Records current values to buffer
    logger.info("Basic variables tracked and updated")

    # We reset the logger just to start the next example, you don't need to reset the logger in your code
    logger.reset()


def sensor_logging():
    """Shows how to log sensor data."""
    print("\n=== Sensor Logging Example ===")

    sensor = Sensor()
    logger = Logger(log_path="./logs", file_name="sensor_data")

    # Track sensor values
    logger.track_function(lambda: sensor.temperature, "Temperature (C)")
    logger.track_function(lambda: sensor.humidity, "Humidity (%)")

    # Simulate a few updates
    for _ in range(3):
        sensor.update()
        logger.update()
        time.sleep(0.1)

    logger.info("Sensor data logged")

    # We reset the logger just to start the next example, you don't need to reset the logger in your code
    logger.reset()


def experiment():
    """Shows how to use the Experiment class."""
    print("\n=== Experiment Data Collection Example ===")
    experiment = Experiment()
    experiment.run()
    print("Experiment completed and data saved")


if __name__ == "__main__":
    basic_variable_tracking()
    sensor_logging()
    experiment()
