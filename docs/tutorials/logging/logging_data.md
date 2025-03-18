# Logging Data

This guide explains how to use the Logger for data collection and variable tracking.

## Variable Tracking

### Basic Variable Tracking

```python
from opensourceleg.logging import LOGGER

# Track a simple variable
x = 42
LOGGER.track_variable(lambda: x, "x_value")

# Track class attributes
class Robot:
    def __init__(self):
        self.position = 0.0
        self.velocity = 0.0

robot = Robot()
LOGGER.track_variable(lambda: robot.position, "position")
LOGGER.track_variable(lambda: robot.velocity, "velocity")

# Update logged values
LOGGER.update()  # Records current values to buffer
```

### Complex Variable Tracking

```python
# Track computed values
LOGGER.track_variable(
    lambda: robot.position * 2,
    "doubled_position"
)

# Track multiple related values
def get_state():
    return [robot.position, robot.velocity]

LOGGER.track_variable(lambda: get_state()[0], "position")
LOGGER.track_variable(lambda: get_state()[1], "velocity")
```

## CSV Output

### Basic CSV Logging

```python
# Initialize with CSV logging enabled
logger = Logger(
    log_path="./data",
    file_name="experiment_data",
    enable_csv_logging=True
)

# CSV file will contain columns for each tracked variable
# Each update() call creates a new row
```

### Managing CSV Output

```python
# Manual buffer flush
LOGGER.flush_buffer()  # Write current buffer to CSV

# Automatic buffer management
logger = Logger(
    buffer_size=1000,  # Flush every 1000 samples
    enable_csv_logging=True
)

# Disable CSV logging temporarily
LOGGER.set_csv_logging(False)
# ... do something ...
LOGGER.set_csv_logging(True)
```

## Real-World Examples

### 1. Recording Sensor Data

```python
class Sensor:
    def __init__(self):
        self.temperature = 20.0
        self.humidity = 0.5

sensor = Sensor()
logger = Logger(file_name="sensor_data")

# Track sensor values
logger.track_variable(lambda: sensor.temperature, "Temperature (C)")
logger.track_variable(lambda: sensor.humidity, "Humidity (%)")

# Main loop
while True:
    # Update sensor readings
    sensor.update()

    # Log data
    logger.update()

    # Sleep for sampling period
    time.sleep(0.1)
```

### 2. Experiment Data Collection

```python
class Experiment:
    def __init__(self):
        self.logger = Logger(
            log_path="./experiments",
            file_name=f"trial_{time.strftime('%Y%m%d_%H%M%S')}",
            buffer_size=5000
        )

        # Track experimental variables
        self.logger.track_variable(lambda: self.input, "Input")
        self.logger.track_variable(lambda: self.output, "Output")
        self.logger.track_variable(lambda: self.error, "Error")

    def run(self):
        try:
            while not self.is_complete():
                self.step()
                self.logger.update()
        finally:
            self.logger.close()  # Ensure data is saved
```

## Best Practices for Data Logging

1. **Buffer Size Selection**

   - Smaller buffers: More frequent writes, less data loss risk
   - Larger buffers: Better performance, more data at risk

   ```python
   # High-frequency data
   logger = Logger(buffer_size=100)  # Frequent writes

   # Low-frequency data
   logger = Logger(buffer_size=5000)  # Better performance
   ```

2. **Error Handling**

   ```python
   # Set maximum errors before auto-untracking
   logger.set_max_errors_before_untrack(5)

   # Check tracked variables
   tracked_vars = logger.get_tracked_variables()
   for name, value in tracked_vars:
       print(f"{name}: {value}")
   ```

3. **Resource Management**

   ```python
   # Use context manager
   with Logger(file_name="experiment") as logger:
       # Track variables
       logger.track_variable(lambda: sensor.value, "sensor")

       # Run experiment
       while running:
           logger.update()
   # Logger automatically closes and flushes data
   ```
