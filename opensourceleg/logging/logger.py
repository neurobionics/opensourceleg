"""
Logging module for opensourceleg library.

Module Overview:

This module defines a custom logger class, `Logger`, designed to log attributes
from class instances to a CSV file. It extends the `logging.Logger` class.

Key Classes:

- `LogLevel`: Enum class that defines the log levels supported by the `Logger` class.
- `Logger`: Logs attributes of class instances to a CSV file. It supports
  setting different logging levels for file and stream handlers.
- `LOGGER`: Global instance of the `Logger` class that can be used throughout.

Usage Guide:

1. Create an instance of the `Logger` class.
2. Optionally, set the logging levels for file and stream handlers using
   `set_file_level` and `set_stream_level` methods.
3. Add class instances and attributes to log using the `track_variable` method.
4. Start logging data using the `update` method.
5. PLEASE call the `close` method before exiting the program to ensure all data is written to the log file.
"""

import csv
import logging
import os
from collections import deque
from datetime import datetime
from enum import Enum
from logging.handlers import RotatingFileHandler
from typing import Any, Callable, Optional, Union

__all__ = ["LOGGER", "LOG_LEVEL", "Logger"]


class LogLevel(Enum):
    """
    Enum for log levels used by the Logger class.

    Attributes:
        DEBUG: Detailed information, typically of interest only when diagnosing problems.
        INFO: Confirmation that things are working as expected.
        WARNING: An indication that something unexpected happened.
        ERROR: A more serious problem, the software has not been able to perform some function.
        CRITICAL: A serious error, indicating that the program itself may be unable to continue running.
    """

    DEBUG = logging.DEBUG
    INFO = logging.INFO
    WARNING = logging.WARNING
    ERROR = logging.ERROR
    CRITICAL = logging.CRITICAL


class Logger(logging.Logger):
    """
    Represents a custom singleton logger class that extends the built-in Python logger. The logger provides additional
    functionality for tracking and logging variables to a CSV file. It supports different log levels and log formatting
    options.

    Args:
        log_path (str): The path to save log files.
        log_format (str): The log message format.
        file_level (LogLevel): The log level for file output.
        stream_level (LogLevel): The log level for console output.
        file_max_bytes (int): The maximum size of the log file in bytes before rotation.
        file_backup_count (int): The number of backup log files to keep.
        file_name (Union[str, None]): The base name for the log file.
        buffer_size (int): The maximum number of log entries to buffer before writing to the CSV file.

    Properties:
        - **file_path**: The path to the log file.
        - **buffer_size**: The maximum number of log entries to buffer.
        - **file_level**: The log level for file output.
        - **stream_level**: The log level for console output.
        - **file_max_bytes**: The maximum size of the log file in bytes before rotation.
        - **file_backup_count**: The number of backup log files to keep.

    Methods:
        - **track_variable**: Track a variable for logging.
        - **untrack_variable**: Stop tracking a variable.
        - **flush_buffer**: Write the buffered log entries to the CSV file.
        - **reset**: Reset the logger state.
        - **close**: Close the logger and flush any remaining log entries.
        - **debug**: Log a debug message.
        - **info**: Log an info message.
        - **warning**: Log a warning message.
        - **error**: Log an error message.
        - **critical**: Log a critical message.
        - **log**: Log a message at a specific log level.

    Examples:
        >>> logger = Logger()
        >>> logger.info("This is an info message")
        [2022-01-01 12:00:00] INFO: This is an info message
        >>> logger.debug("This is a debug message")
        [2022-01-01 12:00:00] DEBUG: This is a debug message

        >>> logger.track_variable(lambda: 42, "answer")
        >>> logger.update()
        >>> logger.flush_buffer()
    """

    _instance = None

    def __new__(cls, *args: Any, **kwargs: Any) -> "Logger":
        """
        Ensure that only one instance of Logger is created (singleton pattern).

        Returns:
            Logger: The singleton Logger instance.
        """
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        else:
            print(f"Reusing existing Logger instance: {id(cls._instance)}")
        return cls._instance

    def __init__(
        self,
        log_path: str = "./",
        log_format: str = "[%(asctime)s] %(levelname)s: %(message)s",
        file_level: LogLevel = LogLevel.DEBUG,
        stream_level: LogLevel = LogLevel.INFO,
        file_max_bytes: int = 0,
        file_backup_count: int = 5,
        file_name: Union[str, None] = None,
        buffer_size: int = 1000,
    ) -> None:
        """
        Initialize the Logger instance.

        Sets up logging paths, format, handler levels, and internal buffers for tracking variables.

        Args:
            log_path (str): Directory path where log files will be stored.
            log_format (str): Format string for log messages.
            file_level (LogLevel): Logging level for file handler.
            stream_level (LogLevel): Logging level for stream (console) handler.
            file_max_bytes (int): Maximum size (in bytes) for log file rotation.
            file_backup_count (int): Number of backup log files to keep.
            file_name (Union[str, None]): Optional user-specified file name prefix.
            buffer_size (int): Maximum number of log records to buffer before writing to CSV.
        """
        if not hasattr(self, "_initialized"):
            super().__init__(__name__)
            self._log_path = log_path
            self._log_format = log_format
            self._file_level = file_level
            self._stream_level = stream_level
            self._file_max_bytes = file_max_bytes
            self._file_backup_count = file_backup_count
            self._user_file_name = file_name

            self._file_path: str = ""
            self._csv_path: str = ""
            self._file: Optional[Any] = None
            self._writer = None
            self._is_logging = False
            self._header_written = False

            self._tracked_vars: dict[int, Callable[[], Any]] = {}
            self._var_names: dict[int, str] = {}
            self._buffer: deque[list[str]] = deque(maxlen=buffer_size)
            self._buffer_size: int = buffer_size

            self._setup_logging()
            self._initialized: bool = True
        else:
            self.set_file_name(file_name)
            self.set_file_level(file_level)
            self.set_stream_level(stream_level)
            self.set_format(log_format)
            self._file_max_bytes = file_max_bytes
            self._file_backup_count = file_backup_count
            self.set_buffer_size(buffer_size)

            self._log_path = log_path

    def _setup_logging(self) -> None:
        """
        Set up the stream logging handler.

        Configures the logger level, formatter, and attaches a stream handler for console output.
        """
        if not hasattr(self, "_stream_handler"):  # Prevent duplicate handlers
            self.setLevel(level=self._file_level.value)
            self._std_formatter = logging.Formatter(self._log_format)
            self._stream_handler = logging.StreamHandler()
            self._stream_handler.setLevel(level=self._stream_level.value)
            self._stream_handler.setFormatter(fmt=self._std_formatter)
            self.addHandler(hdlr=self._stream_handler)

    def _setup_file_handler(self) -> None:
        if not hasattr(self, "_file_handler"):  # Ensure file handler is added only once
            self._generate_file_paths()

            self._file_handler = RotatingFileHandler(
                filename=self._file_path,
                mode="w",
                maxBytes=self._file_max_bytes,
                backupCount=self._file_backup_count,
                encoding="utf-8",
            )
            self._file_handler.setLevel(level=self._file_level.value)
            self._file_handler.setFormatter(fmt=self._std_formatter)
            self.addHandler(hdlr=self._file_handler)

    def _ensure_file_handler(self) -> None:
        if not hasattr(self, "_file_handler"):
            self._setup_file_handler()

    def track_variable(self, var_func: Callable[[], Any], name: str) -> None:
        """
        Record the value of a variable and log it to a CSV file.

        Args:
            var_func: A function that returns the value of the variable.
            name: The name of the variable.

        Examples:
            >>> class MyClass:
            ...     def __init__(self):
            ...         self.value = 42
            >>> obj = MyClass()
            >>> LOGGER.track_variable(lambda: obj.value, "answer")
            >>> LOGGER.update()
            >>> LOGGER.flush_buffer()
        """
        var_id = id(var_func)
        self._tracked_vars[var_id] = var_func
        self._var_names[var_id] = name

    def untrack_variable(self, var_func: Callable[[], Any]) -> None:
        """
        Stop tracking a variable and remove it from the logger buffer.

        Args:
            var_func: The function used to track the variable.

        Examples:
            >>> class MyClass:
            ...     def __init__(self):
            ...         self.value = 42
            >>> obj = MyClass()
            >>> LOGGER.track_variable(lambda: obj.value, "answer")
            >>> LOGGER.update()
            >>> LOGGER.flush_buffer()
            >>> LOGGER.untrack_variable(lambda: obj.value)
        """
        var_id = id(var_func)
        self._tracked_vars.pop(var_id, None)
        self._var_names.pop(var_id, None)

    def __repr__(self) -> str:
        """
        Return a string representation of the Logger instance.

        Returns:
            str: A string representation including the current file path.
        """
        return f"Logger(file_path={self._file_path})"

    def set_file_name(self, file_name: Union[str, None]) -> None:
        """
        Set the base name for the log file.

        Args:
            file_name: The base name for the log file.

        Examples:
            >>> LOGGER.set_file_name("my_log_file")
            >>> LOGGER.file_path
            "./my_log_file.log"
        """
        # if filename has an extension, remove it
        if file_name is not None and "." in file_name:
            file_name = file_name.split(".")[0]

        self._user_file_name = file_name
        self._file_path = os.path.join(self._log_path, f"{file_name}.log")
        self._csv_path = os.path.join(self._log_path, f"{file_name}.csv")

    def set_file_level(self, level: LogLevel) -> None:
        """
        Set the log level for file output.

        Args:
            level: The log level for file output.

        Examples:
            >>> LOGGER.set_file_level(LogLevel.INFO)
            >>> LOGGER.file_level
            LogLevel.INFO
            >>> LOGGER.debug("This is a debug message and will not be logged")
        """
        self._file_level = level
        if hasattr(self, "_file_handler"):
            self._file_handler.setLevel(level=level.value)

    def set_stream_level(self, level: LogLevel) -> None:
        """
        Set the log level for console output.

        Args:
            level: The log level for console output.

        Examples:
            >>> LOGGER.set_stream_level(LogLevel.INFO)
            >>> LOGGER.stream_level
            LogLevel.INFO
            >>> LOGGER.debug("This is a debug message and will not be streamed")
        """
        self._stream_level = level
        self._stream_handler.setLevel(level=level.value)

    def set_format(self, log_format: str) -> None:
        """
        Set the log message format. The format string uses the same syntax as the built-in Python logging module.

        Args:
            log_format: The log message format.

        Examples:
            >>> LOGGER.set_format("[%(asctime)s] %(levelname)s: %(message)s")
            >>> LOGGER.info("This is an info message")
            [2022-01-01 12:00:00] INFO: This is an info message
        """
        self._log_format = log_format
        self._std_formatter = logging.Formatter(log_format)
        if hasattr(self, "_file_handler"):
            self._file_handler.setFormatter(fmt=self._std_formatter)
        self._stream_handler.setFormatter(fmt=self._std_formatter)

    def set_buffer_size(self, buffer_size: int) -> None:
        """
        Set the maximum number of log entries to buffer before writing to the CSV file.

        Args:
            buffer_size: The maximum number of log entries to buffer.
        """
        self._buffer_size = buffer_size
        self._buffer = deque(self._buffer, maxlen=buffer_size)

    def update(self) -> None:
        """
        Update the logger by logging the current values of tracked variables to the buffer.

        Examples:
            >>> class MyClass:
            ...     def __init__(self):
            ...         self.value = 42
            >>> obj = MyClass()
            >>> LOGGER.track_variable(lambda: obj.value, "answer")
            >>> LOGGER.update()
        """
        if not self._tracked_vars:
            return

        data = []
        for _var_id, get_value in self._tracked_vars.items():
            value = get_value()
            data.append(str(value))

        self._buffer.append(data)

        if len(self._buffer) >= self._buffer_size:
            self.flush_buffer()

    def flush_buffer(self) -> None:
        """
        Flush the buffered log data to the CSV file.

        Ensures that the file handler is available, writes the header if not yet written,
        writes all buffered rows to the CSV, clears the buffer, and flushes the file.
        """
        if not self._buffer:
            return

        self._ensure_file_handler()

        if self._file is None:
            self._file = open(self._csv_path, "w", newline="")
            self._writer = csv.writer(self._file)  # type: ignore[assignment]

        if not self._header_written:
            self._write_header()

        self._writer.writerows(self._buffer)  # type: ignore[attr-defined]
        self._buffer.clear()
        self._file.flush()

    def _write_header(self) -> None:
        """
        Write the CSV header based on tracked variable names.
        This header is written only once per log file.
        """
        header = list(self._var_names.values())
        self._writer.writerow(header)  # type: ignore[attr-defined]
        self._header_written = True

    def _generate_file_paths(self) -> None:
        """
        Generate file paths for the log and CSV files based on the current settings.

        Creates the log directory if it does not exist, and uses the current timestamp
        (and optionally a user-specified name) to generate unique file names.
        """
        now = datetime.now()
        timestamp = now.strftime("%Y%m%d_%H%M%S")
        script_name = os.path.basename(__file__).split(".")[0]

        base_name = self._user_file_name if self._user_file_name else f"{script_name}_{timestamp}"

        file_path = os.path.join(self._log_path, base_name)
        self._file_path = file_path + ".log"
        self._csv_path = file_path + ".csv"

    def __enter__(self) -> "Logger":
        """
        Enter the runtime context related to this Logger instance.

        Returns:
            Logger: The current Logger instance.
        """
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """
        Exit the runtime context and close the Logger.

        Args:
            exc_type (Any): The exception type if an exception occurred.
            exc_val (Any): The exception value if an exception occurred.
            exc_tb (Any): The traceback if an exception occurred.
        """
        self.close()

    def reset(self) -> None:
        """
        Reset the Logger.

        Closes the current file, reinitializes the logging handlers, clears tracked variables,
        and resets header status.
        """
        self.close()
        self._setup_logging()

        self._tracked_vars.clear()
        self._var_names.clear()
        self._header_written = False
        if hasattr(self, "_file_handler"):
            self._file_handler.close()
            del self._file_handler

    def close(self) -> None:
        """
        Flush any buffered log data and close the CSV file.

        This method should be called before the program exits to ensure all data is written.
        Close the logger and flush any remaining log entries.

        Examples:
            >>> LOGGER.close()
            >>> LOGGER.info("This message will not be logged")
        """
        self.flush_buffer()
        if self._file:
            self._file.close()
            self._file = None
            self._writer = None

    def debug(self, msg: object, *args: object, **kwargs: Any) -> None:
        """
        Log a debug message.

        Ensures that the file handler is set up before logging.

        Args:
            msg (object): The message to log.
            *args (object): Additional arguments.
            **kwargs (Any): Additional keyword arguments.
        """
        self._ensure_file_handler()
        super().debug(msg, *args, **kwargs)

    def info(self, msg: object, *args: object, **kwargs: Any) -> None:
        """
        Log an info message.

        Ensures that the file handler is set up before logging.

        Args:
            msg (object): The message to log.
            *args (object): Additional arguments.
            **kwargs (Any): Additional keyword arguments.
        """
        self._ensure_file_handler()
        super().info(msg, *args, **kwargs)

    def warning(self, msg: object, *args: object, **kwargs: Any) -> None:
        """
        Log a warning message.

        Ensures that the file handler is set up before logging.

        Args:
            msg (object): The message to log.
            *args (object): Additional arguments.
            **kwargs (Any): Additional keyword arguments.
        """
        self._ensure_file_handler()
        super().warning(msg, *args, **kwargs)

    def error(self, msg: object, *args: object, **kwargs: Any) -> None:
        """
        Log an error message.

        Ensures that the file handler is set up before logging.

        Args:
            msg (object): The message to log.
            *args (object): Additional arguments.
            **kwargs (Any): Additional keyword arguments.
        """
        self._ensure_file_handler()
        super().error(msg, *args, **kwargs)

    def critical(self, msg: object, *args: object, **kwargs: Any) -> None:
        """
        Log a critical message.

        Ensures that the file handler is set up before logging.

        Args:
            msg (object): The message to log.
            *args (object): Additional arguments.
            **kwargs (Any): Additional keyword arguments.
        """
        self._ensure_file_handler()
        super().critical(msg, *args, **kwargs)

    def log(self, level: int, msg: object, *args: object, **kwargs: Any) -> None:
        """
        Log a message with a specific log level.

        Ensures that the file handler is set up before logging.

        Args:
            level (int): The log level.
            msg (object): The message to log.
            *args (object): Additional arguments.
            **kwargs (Any): Additional keyword arguments.
        """
        self._ensure_file_handler()
        super().log(level, msg, *args, **kwargs)

    @property
    def file_path(self) -> Optional[str]:
        """
        Get the current file path for the log file.

        Returns:
            Optional[str]: The file path as a string, or None if not set.
        """
        return self._file_path

    @property
    def csv_path(self) -> Optional[str]:
        """
        Get the current file path for the CSV file.

        Returns:
            Optional[str]: The CSV file path as a string, or None if not set.
        """
        return self._csv_path

    @property
    def log_path(self) -> str:
        """
        Get the log directory path.

        Returns:
            str: The directory path where log files are stored.
        """
        return self._log_path

    @property
    def buffer_size(self) -> int:
        """
        Get the current buffer size.

        Returns:
            int: The maximum number of log records held in the buffer.
        """
        return self._buffer_size

    @property
    def file_level(self) -> LogLevel:
        """
        Get the current file logging level.

        Returns:
            LogLevel: The logging level for the file handler.
        """
        return self._file_level

    @property
    def stream_level(self) -> LogLevel:
        """
        Get the current stream (console) logging level.

        Returns:
            LogLevel: The logging level for the stream handler.
        """
        return self._stream_level

    @property
    def file_max_bytes(self) -> int:
        """
        Get the maximum number of bytes for the log file before rotation.

        Returns:
            int: The maximum file size in bytes.
        """
        return self._file_max_bytes

    @property
    def file_backup_count(self) -> int:
        """
        Get the number of backup log files to keep.

        Returns:
            int: The backup count.
        """
        return self._file_backup_count


# Initialize a global logger instance to be used throughout the library
LOGGER = Logger()
LOG_LEVEL = dict(enumerate(LogLevel.__members__.values()))

if __name__ == "__main__":
    LOGGER.info("This is an info message")

    LOGGER.set_stream_level(LogLevel.CRITICAL)

    LOGGER.info("This is an info message and won't be displayed")
    LOGGER.critical("This is a critical message and will be displayed")
