# Configuring the Logger

This guide covers the various ways to configure the Logger for your needs. All examples can be found in the [`configuring_logger.py`](https://github.com/neurobionics/opensourceleg/tree/main/tutorials/logging/configuring_logger.py) script.

## Initial Setup

### Basic Configuration

The Logger can be initialized with several basic parameters that control its behavior. This configuration sets up both file and console logging, with different verbosity levels for each output. The `log_path` specifies where log files are stored, while `file_name` determines the base name for log files. The `buffer_size` parameter helps optimize performance by controlling how frequently logs are written to disk.

```python
--8<-- "tutorials/logging/configuring_logger.py:8:17"
```

### Log Levels

Log levels help you control the verbosity of your logging output. You can set different levels for file logging and console output, allowing you to have detailed logs in your files while keeping console output focused on more important messages.

The levels range from `DEBUG` (most verbose) to `CRITICAL` (most severe), giving you fine-grained control over what gets logged.

```python
--8<-- "tutorials/logging/configuring_logger.py:22:32"
```

### File Management

The Logger includes built-in file management capabilities to help maintain your log files. You can configure file rotation based on size limits and specify how many backup files to keep. This prevents log files from growing too large and consuming excessive disk space while maintaining a history of recent logs.

```python
--8<-- "tutorials/logging/configuring_logger.py:38:43"
```

## Advanced Configuration

### Custom Formatting

The Logger supports customizable log message formatting, allowing you to include various pieces of information in each log entry. You can include timestamps, log levels, file names, line numbers, and function names in your log messages. This flexibility helps you create log outputs that match your debugging and monitoring needs.

```python
--8<-- "tutorials/logging/configuring_logger.py:48:57"
```

### Buffer Management

Buffer management allows you to optimize the Logger's performance by controlling when logs are written to disk. A larger buffer size reduces I/O operations but increases the amount of memory used. You can also manually flush the buffer when needed, ensuring critical logs are written immediately.

```python
--8<-- "tutorials/logging/configuring_logger.py:63:69"
```

## Common Configurations

### Debug Configuration

This configuration is optimized for development and debugging scenarios. It enables detailed logging both to file and console, including source file information and line numbers. This helps developers track down issues and understand the flow of their application.

```python
--8<-- "tutorials/logging/configuring_logger.py:75:80"
```

### Production Configuration

The production configuration is designed for deployed applications, with an emphasis on performance and stability. It uses more conservative log levels, larger file sizes, and includes rotation to manage disk usage. Console output is limited to warnings and above to reduce unnecessary output in production environments.

```python
--8<-- "tutorials/logging/configuring_logger.py:86:92"
```

If you have any questions or need further assistance, please post on the [Open Source Leg community forum](https://opensourceleg.org/community).
