# Configuring the Logger

This guide covers the various ways to configure the Logger for your needs.

## Initial Setup

### Basic Configuration

```python
logger = Logger(
    log_path="./logs",              # Log file directory
    file_name="experiment_1",       # Base name for log files
    log_format="[%(asctime)s] %(levelname)s: %(message)s",
    file_level=LogLevel.DEBUG,      # Level for file logging
    stream_level=LogLevel.INFO,     # Level for console output
    buffer_size=1000,              # Entries before auto-flush
    enable_csv_logging=True        # Enable/disable CSV logging
)
```

### Log Levels

```python
# Changing log levels after initialization
logger.set_file_level(LogLevel.DEBUG)    # More detailed file logging
logger.set_stream_level(LogLevel.INFO)   # Less console spam

# Available log levels (in order of severity):
# - LogLevel.DEBUG    - Detailed debugging information
# - LogLevel.INFO     - General information
# - LogLevel.WARNING  - Warning messages
# - LogLevel.ERROR    - Error messages
# - LogLevel.CRITICAL - Critical errors
```

### File Management

```python
# Change log file name
logger.set_file_name("new_experiment")

# Configure file rotation
logger = Logger(
    file_max_bytes=1024*1024,    # 1MB max file size
    file_backup_count=5          # Keep 5 backup files
)
```

## Advanced Configuration

### Custom Formatting

```python
# Available format variables:
# %(asctime)s     - Timestamp
# %(levelname)s   - Log level
# %(message)s     - Log message
# %(filename)s    - Source filename
# %(lineno)d     - Line number
# %(funcName)s   - Function name

logger.set_format("[%(asctime)s][%(levelname)s] %(filename)s:%(lineno)d - %(message)s")
```

### Buffer Management

```python
# Adjust buffer size
logger.set_buffer_size(2000)

# Manual buffer flush
logger.flush_buffer()

# Auto-flush settings
logger = Logger(
    buffer_size=500,              # Flush every 500 entries
    enable_csv_logging=True       # Enable CSV output
)
```

## Best Practices

1. **Log File Organization**

   ```python
   logger = Logger(
       log_path="./logs/experiment_1/",  # Organize by experiment
       file_name="trial_001"             # Clear naming
   )
   ```

2. **Error Handling**

   ```python
   # Set maximum errors before auto-untracking variables
   logger.set_max_errors_before_untrack(3)
   ```

3. **Resource Management**
   ```python
   # Use context manager for automatic cleanup
   with Logger(log_path="./logs") as logger:
       # Your code here
       pass  # Logger automatically closes
   ```

## Common Configurations

### Debug Configuration

```python
logger = Logger(
    file_level=LogLevel.DEBUG,
    stream_level=LogLevel.DEBUG,
    log_format="[%(asctime)s][%(levelname)s] %(filename)s:%(lineno)d - %(message)s"
)
```

### Production Configuration

```python
logger = Logger(
    file_level=LogLevel.INFO,
    stream_level=LogLevel.WARNING,
    file_max_bytes=1024*1024*10,  # 10MB
    file_backup_count=5
)
```

### Data Collection Configuration

```python
logger = Logger(
    buffer_size=5000,
    enable_csv_logging=True,
    log_format="[%(asctime)s] %(message)s"
)
```

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
