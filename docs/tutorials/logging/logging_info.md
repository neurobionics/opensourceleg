# Debug Logging

This guide covers using the Logger for debugging and troubleshooting.

## Log Levels

### Understanding Log Levels

```python
from opensourceleg.logging import LOGGER, LogLevel

# Levels in order of severity:
LOGGER.debug("Detailed debugging information")    # Most verbose
LOGGER.info("General information")
LOGGER.warning("Warning messages")
LOGGER.error("Error messages")
LOGGER.critical("Critical errors")                # Most severe
```

### Configuring Log Levels

```python
# Set different levels for file and console
LOGGER.set_file_level(LogLevel.DEBUG)     # Full detail in files
LOGGER.set_stream_level(LogLevel.INFO)    # Less console output

# Common configurations
# Development
LOGGER.set_file_level(LogLevel.DEBUG)
LOGGER.set_stream_level(LogLevel.DEBUG)

# Production
LOGGER.set_file_level(LogLevel.INFO)
LOGGER.set_stream_level(LogLevel.WARNING)
```

## Debug Strategies

### 1. Function Entry/Exit Logging

```python
def complex_function(param):
    LOGGER.debug(f"Entering complex_function with param={param}")
    try:
        result = perform_calculation(param)
        LOGGER.debug(f"Calculation result: {result}")
        return result
    except Exception as e:
        LOGGER.error(f"Error in complex_function: {e}")
        raise
    finally:
        LOGGER.debug("Exiting complex_function")
```

### 2. State Tracking

```python
class StateManager:
    def __init__(self):
        self.state = "INIT"

    def transition(self, new_state):
        LOGGER.debug(f"State transition: {self.state} -> {new_state}")
        self.state = new_state

    def process(self):
        LOGGER.debug(f"Processing in state: {self.state}")
```

### 3. Performance Monitoring

```python
import time

def measure_performance():
    start_time = time.time()
    LOGGER.debug("Starting performance measurement")

    # Your code here

    elapsed = time.time() - start_time
    LOGGER.debug(f"Operation took {elapsed:.3f} seconds")
```

## Error Tracking

### 1. Exception Handling

```python
try:
    # Risky operation
    result = perform_operation()
except ValueError as e:
    LOGGER.error(f"Invalid value: {e}")
except RuntimeError as e:
    LOGGER.critical(f"Critical error: {e}")
    raise  # Re-raise critical errors
except Exception as e:
    LOGGER.error(f"Unexpected error: {type(e).__name__}: {e}")
```

### 2. Error Context

```python
def process_data(data):
    LOGGER.debug(f"Processing data of type: {type(data)}")
    try:
        result = transform_data(data)
    except Exception as e:
        LOGGER.error(f"Error processing data: {e}")
        LOGGER.debug(f"Data dump: {data}")  # Include context
        raise
```

## Advanced Debugging

### 1. Custom Formatting

```python
# Include file and line information
LOGGER.set_format(
    "[%(asctime)s][%(levelname)s] %(filename)s:%(lineno)d - %(message)s"
)

# Include function names
LOGGER.set_format(
    "[%(asctime)s][%(levelname)s] %(funcName)s - %(message)s"
)
```

### 2. Conditional Debugging

```python
DEBUG_MODE = True

def debug_print(*args):
    if DEBUG_MODE:
        LOGGER.debug(" ".join(str(arg) for arg in args))

# Usage
debug_print("Value:", x, "State:", state)
```

### 3. Debug Sessions

```python
class DebugSession:
    def __init__(self, name):
        self.name = name

    def __enter__(self):
        LOGGER.debug(f"=== Starting {self.name} ===")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type:
            LOGGER.error(f"Error in {self.name}: {exc_val}")
        LOGGER.debug(f"=== Ending {self.name} ===")

# Usage
with DebugSession("Data Processing"):
    process_data()
```

```

```
