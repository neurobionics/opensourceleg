import time

from observable import Logger


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
        Logger.init(log_directory="./experiments", log_name="trial.log")

        self.input = 0.0
        self.output = 0.0
        self.error = 0.0
        self.steps = 0
        self.max_steps = 5  # For demonstration

        # Track experimental variables
        Logger.track_functions({
            "Input": lambda: self.input,
            "Output": lambda: self.output,
            "Error": lambda: self.error,
        })

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
            Logger.record()


def basic_variable_tracking():
    """Shows basic variable tracking functionality."""
    print("\n=== Basic Variable Tracking Example ===")

    Logger.init(log_directory="./logs", log_name="basic_variable_tracking.log")

    # Track a simple variable
    x = 42
    Logger.track_functions({"x_value": lambda: x})

    # Track class attributes
    class Robot:
        def __init__(self):
            self.position = 0.0
            self.velocity = 0.0

    robot = Robot()
    Logger.trace_variables({"position": robot.position, "velocity": robot.velocity})

    # Update logged values
    Logger.record()  # Records current values to buffer
    Logger.info("Basic variables tracked and updated")


def sensor_logging():
    """Shows how to log sensor data."""
    print("\n=== Sensor Logging Example ===")

    sensor = Sensor()
    Logger.init(log_directory="./logs", log_name="sensor_data.log")

    # Track sensor values
    Logger.track_functions({"Temperature": lambda: sensor.temperature, "Humidity": lambda: sensor.humidity})

    # Simulate a few updates
    for _ in range(3):
        sensor.update()
        Logger.record()
        time.sleep(0.1)

    Logger.info("Sensor data logged")


def experiment():
    """Shows how to use the Experiment class."""
    print("\n=== Experiment Data Collection Example ===")
    experiment = Experiment()
    experiment.run()
    print("Experiment completed and data saved")


if __name__ == "__main__":
    # You can run one test per run

    # basic_variable_tracking()
    # sensor_logging()
    experiment()
