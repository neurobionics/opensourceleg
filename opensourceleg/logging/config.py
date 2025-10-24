"""
Logging module for opensourceleg library using loguru.

This module provides a clean, simple logging interface based on loguru,
replacing the previous custom Logger implementation. It provides:

- Colored console output with customizable formatting
- File logging with automatic rotation and compression
- Multiple setup modes: default, minimal, quiet
- Thread-safe logging with async file I/O

Usage Guide:

1. Import the logger: `from opensourceleg.logging import logger`
2. Use it directly: `logger.info("message")`, `logger.error("error")`
3. Optionally customize with setup functions
4. Logging is initialized by default on module import

Examples:
    >>> from opensourceleg.logging import logger
    >>> logger.info("Initialization complete")
    >>> logger.warning("Temperature approaching limit")
    >>> logger.error("Failed to connect to actuator")
"""

import sys
from typing import Union

from loguru import logger

__all__ = [
    "logger",
    "setup_console_logging",
    "setup_file_logging",
    "setup_default_logging",
    "setup_minimal_logging",
    "setup_quiet_logging",
    "DEFAULT_CONSOLE_FORMAT",
    "DEFAULT_FILE_FORMAT",
    "MINIMAL_CONSOLE_FORMAT",
]


# ============================================================================
# Format Definitions
# ============================================================================

_LOG_JOINER = " | "

_DEFAULT_COMPONENTS = [
    "{time:HH:mm:SSS}",
    "{level}",
    "{module}:{function}:{line}",
    "{message}",
]

_CONSOLE_STYLED_COMPONENTS = [
    "<green>{time:HH:mm:SSS}</green>",
    "<level>{level}</level>",
    "<cyan>{module}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan>",
    "<level>{message}</level>",
]

_MINIMAL_CONSOLE_STYLED_COMPONENTS = [
    "<level>{level}</level>",
    "<level>{message}</level>",
]

# Default format strings for logging
DEFAULT_CONSOLE_FORMAT = _LOG_JOINER.join(_CONSOLE_STYLED_COMPONENTS)
DEFAULT_FILE_FORMAT = _LOG_JOINER.join(_DEFAULT_COMPONENTS)
MINIMAL_CONSOLE_FORMAT = _LOG_JOINER.join(_MINIMAL_CONSOLE_STYLED_COMPONENTS)


# ============================================================================
# Internal Configuration Tracking
# ============================================================================

_logging_config = {}


def _record_logging_config(**kwargs: Union[str, bool]) -> None:
    """Internal function to track logging configuration."""
    _logging_config.update(kwargs)


# ============================================================================
# Logging Setup Functions
# ============================================================================


def setup_console_logging(
    level: str = "INFO",
    format_string: Union[str, None] = None,
    colorize: bool = True,
) -> int:
    """Add a console (stderr) logging handler.

    Args:
        level: Minimum log level to display (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        format_string: Custom format string. If None, uses DEFAULT_CONSOLE_FORMAT
        colorize: Whether to colorize the output

    Returns:
        Handler ID that can be used with logger.remove() if needed

    Example:
        >>> from opensourceleg.logging import setup_console_logging
        >>> setup_console_logging(level="DEBUG")
    """
    fmt = format_string if format_string is not None else DEFAULT_CONSOLE_FORMAT

    handler_id = logger.add(
        sys.stderr,
        format=fmt,
        level=level,
        colorize=colorize,
    )
    return handler_id


def setup_file_logging(
    file_path: str = "opensourceleg.log",
    level: str = "DEBUG",
    format_string: Union[str, None] = None,
    rotation: str = "10 MB",
    retention: str = "7 days",
    compression: str = "zip",
    enqueue: bool = True,
    delay: bool = False,
) -> int:
    """Add a file logging handler with rotation and compression.

    Args:
        file_path: Path to the log file
        level: Minimum log level to write (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        format_string: Custom format string. If None, uses DEFAULT_FILE_FORMAT
        rotation: When to rotate the log file (e.g., "10 MB", "1 day", "12:00")
        retention: How long to keep old log files (e.g., "7 days", "10 files")
        compression: Compression format for rotated files ("zip", "gz", "bz2", or None)
        enqueue: Whether to use thread-safe logging (recommended)
        delay: Whether to delay file creation until first write

    Returns:
        Handler ID that can be used with logger.remove() if needed

    Example:
        >>> from opensourceleg.logging import setup_file_logging
        >>> setup_file_logging("my_robot.log", level="DEBUG", rotation="50 MB")
    """
    fmt = format_string if format_string is not None else DEFAULT_FILE_FORMAT

    handler_id = logger.add(
        file_path,
        format=fmt,
        level=level,
        rotation=rotation,
        retention=retention,
        compression=compression,
        enqueue=enqueue,
        delay=delay,
    )
    return handler_id


def setup_default_logging(
    console_level: str = "INFO",
    file_level: str = "DEBUG",
    file_path: str = "opensourceleg.log",
    clear_existing_handlers: bool = True,
    delay_file_creation: bool = False,
) -> tuple[int, int]:
    """Configure logging with sensible defaults: console at INFO + file at DEBUG.

    This is the recommended way to set up logging for most users. It provides:
    - Colored console output at INFO level or higher
    - Detailed file logging at DEBUG level or higher
    - Automatic log rotation (10 MB) and retention (7 days)
    - Compressed archives of rotated logs

    Args:
        console_level: Minimum level for console output (default: "INFO")
        file_level: Minimum level for file output (default: "DEBUG")
        file_path: Path to the log file (default: "opensourceleg.log")
        clear_existing_handlers: Whether to remove existing handlers first (default: True)
        delay_file_creation: Whether to delay file creation until first write (default: False)

    Returns:
        Tuple of (console_handler_id, file_handler_id)

    Example:
        >>> from opensourceleg.logging import setup_default_logging
        >>> setup_default_logging()  # Use all defaults
        >>> # Or customize:
        >>> setup_default_logging(console_level="DEBUG", file_path="my_robot.log")
    """
    if clear_existing_handlers:
        logger.remove()

    console_id = setup_console_logging(level=console_level, colorize=True)
    file_id = setup_file_logging(file_path=file_path, level=file_level, delay=delay_file_creation)

    _record_logging_config(
        mode="default",
        console_level=console_level,
        file_level=file_level,
        file_path=file_path,
        clear_existing_handlers=clear_existing_handlers,
        delay_file_creation=delay_file_creation,
    )

    return console_id, file_id


def setup_minimal_logging(level: str = "INFO") -> int:
    """Configure minimal console-only logging without file output.

    Useful for quick scripts or when you don't want log files.

    Args:
        level: Minimum log level to display (default: "INFO")

    Returns:
        Handler ID that can be used with logger.remove() if needed

    Example:
        >>> from opensourceleg.logging import setup_minimal_logging
        >>> setup_minimal_logging(level="WARNING")
    """
    logger.remove()
    handler_id = setup_console_logging(level=level, format_string=MINIMAL_CONSOLE_FORMAT)
    _record_logging_config(mode="minimal", console_level=level)
    return handler_id


def setup_quiet_logging(file_path: str = "opensourceleg.log", level: str = "DEBUG") -> int:
    """Configure file-only logging with no console output.

    Useful for background tasks or automated scripts where console output
    would be distracting.

    Args:
        file_path: Path to the log file (default: "opensourceleg.log")
        level: Minimum log level to write (default: "DEBUG")

    Returns:
        Handler ID that can be used with logger.remove() if needed

    Example:
        >>> from opensourceleg.logging import setup_quiet_logging
        >>> setup_quiet_logging("background_task.log")
    """
    logger.remove()
    handler_id = setup_file_logging(file_path=file_path, level=level)
    _record_logging_config(
        mode="quiet",
        file_path=file_path,
        file_level=level,
        clear_existing_handlers=True,
    )
    return handler_id


# ============================================================================
# Initialize default logging configuration
# ============================================================================
# Configure default logging behavior when the module is imported.
# By default, logs to console at INFO level and to file at DEBUG level.
# Users can override this by calling any of the setup_*_logging functions.

# Remove loguru's default handler (which logs everything to stderr at DEBUG level)
setup_default_logging(
    console_level="INFO",
    file_level="DEBUG",
    file_path="opensourceleg.log",
    clear_existing_handlers=True,
    delay_file_creation=True,
)


if __name__ == "__main__":
    logger.info("hello world")
    logger.debug("debug message")
    logger.warning("warning message")
    logger.error("error message")
