# Getting Started with Logging

The `opensourceleg` library provides a powerful logging system through the `Logger` class, which is built on top of the native `logging` module. This guide will introduce you to the basics of logging and help you get started.

## Overview

The Logger class offers:

- Standard logging levels (`DEBUG`, `INFO`, `WARNING`, `ERROR`, `CRITICAL`)
- CSV data logging for variables
- File and console output
- Thread-safe operation
- Singleton pattern (one logger instance across your application)

## Basic Usage

### 1. Importing the Logger

```python
from opensourceleg.logging import LOGGER, LogLevel
```

### 2. Simple Logging

```python
# Basic logging at different levels
LOGGER.debug("Detailed information for debugging")
LOGGER.info("General information about program execution")
LOGGER.warning("Warning messages for potentially problematic situations")
LOGGER.error("Error messages for serious problems")
LOGGER.critical("Critical messages for fatal errors")
```

### 3. Basic Configuration

```python
# Initialize logger with custom settings
logger = Logger(
    log_path="./logs",          # Where to save log files
    file_name="my_experiment",  # Base name for log files
    buffer_size=1000           # How many entries to buffer before writing
)
```

## Core Concepts

- Logger uses a singleton pattern
- All imports reference the same logger instance
- Configuration changes affect all users of the logger

- Console output for immediate feedback
- File output for permanent record
- Different log levels can be set for each

- Automatically log variable values over time
- CSV output for data analysis
- Buffer system for efficient writing

## Next Steps

Choose a tutorial to get started:

1. [Configuring the Logger](configuring_logger.md)
2. [Logging Data](logging_data.md)
3. [Read the API Reference](../../api/logging.md)

Each tutorial walks you through specific use cases and best practices.
