from enum import Enum
from typing import Any, Dict, Optional

class LogLevel(Enum):
    """Enum for available log levels."""
    TRACE: LogLevel
    DEBUG: LogLevel
    INFO: LogLevel
    WARN: LogLevel
    ERROR: LogLevel


class Logger:
    """
    A class for configuring and using a logger.
    This class is a Python interface to the Rust logging implementation.
    """

    @staticmethod
    def init(
        time_format: Optional[str] = None,
        log_directory: Optional[str] = None,
        log_name: Optional[str] = None,
        print_stdout: bool = False,
        file_max_bytes: int = 0,
        backup_count: int = 0,
    ) -> None:
        """
        Initializes the global logger. This can be called multiple times - but behaviour will not change

        Args:
            time_format (Optional[str]): The format for timestamps in the log, using chrono format specifiers. Defaults to "%Y-%m-%d %H:%M:%S%.3f".
            log_directory (Optional[str]): The directory to save log files. Defaults to "./logs".
            log_name (Optional[str]): The base name for the log file. Defaults to "logfile.log".
            print_stdout (bool): Whether to print log messages to standard output. Defaults to False.
            file_max_bytes (int): The maximum size in bytes for a log file before it is rotated. A value of 0 means no rotation based on size. Defaults to 0.
            backup_count (int): The number of backup log files to keep. A value of 0 means infinite backups. Defaults to 0.
        """

    @staticmethod
    def debug(msg: str) -> None:
        """
        Logs a message with the DEBUG level.

        Args:
            msg (str): The message to log.
        """

    @staticmethod
    def info(msg: str) -> None:
        """
        Logs a message with the INFO level.

        Args:
            msg (str): The message to log.
        """

    @staticmethod
    def trace(msg: str) -> None:
        """
        Logs a message with the TRACE level.

        Args:
            msg (str): The message to log.
        """

    @staticmethod
    def error(msg: str) -> None:
        """
        Logs a message with the ERROR level.

        Args:
            msg (str): The message to log.
        """
    @staticmethod
    def warn(msg: str) -> None:
        """
        Logs a message with the WARN level.

        Args:
            msg (str): The message to log.
        """

    @staticmethod
    def trace_variables(variables: Dict[str, Any]) -> None:
        """
        Records the state of multiple variables from a dictionary.
        These variables are held in memory until `flush_record` is called.

        Args:
            variables (Dict[str, Any]): A dictionary of variable names to their values.
        """

    @staticmethod
    def flush_record() -> None:
        """
        Writes all currently tracked variables to the variables log file and clears the in-memory store.
        """