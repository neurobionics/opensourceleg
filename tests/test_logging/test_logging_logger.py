import csv
import logging
import os
from collections import deque
from unittest.mock import Mock

import pytest

from opensourceleg.logging.logger import LOGGER, Logger, LogLevel

CURR_DIR = os.path.dirname(os.path.realpath(__file__))


# Test LogLevel class
def test_log_level_default():
    assert {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"} <= {e.name for e in LogLevel}


def test_log_level_value_logging():
    assert all([
        LogLevel.DEBUG.value == logging.DEBUG,
        LogLevel.INFO.value == logging.INFO,
        LogLevel.WARNING.value == logging.WARNING,
        LogLevel.ERROR.value == logging.ERROR,
        LogLevel.CRITICAL.value == logging.CRITICAL,
    ])


def test_log_level_len():
    assert len(LogLevel) == 5


@pytest.fixture(scope="function")
def test_logger():
    """
    Create a new singleton instance of the Logger class.

    Yields:
        Logger: A new singleton instance of the Logger class.
    """
    # Create a new singleton instance
    log = Logger(
        log_path=CURR_DIR,
        file_name="test_logging",
    )
    log.reset()

    yield log
    log.reset()

    if hasattr(log, "_file") and log._file:
        log._file.close()
        log._file = None

    for ext in [".log", ".csv"]:
        file_path = os.path.join(CURR_DIR, f"test_logging{ext}")
        if os.path.exists(file_path):
            os.remove(file_path)


@pytest.fixture(scope="function")
def isolated_logger():
    """
    Creates a non-singleton instance of the Logger class by resetting the singleton state.

    Yields:
        Logger: A non-singleton instance of the Logger class.
    """
    original_instance = Logger._instance
    Logger._instance = None

    # Create a test instance
    log = Logger(
        log_path=CURR_DIR,
        file_name="test_logging",
    )

    yield log

    # Clean up
    log.reset()
    if hasattr(log, "_file") and log._file:
        log._file.close()

    # Restore original singleton
    Logger._instance = original_instance


# check if the fixture is being reset after each test
def test_fixture_reset(isolated_logger: Logger):
    assert not hasattr(isolated_logger, "_file_handler")


# Test new
def test_logger_new(isolated_logger: Logger):
    new_logger = Logger.__new__(Logger)
    assert new_logger is isolated_logger
    new_logger.reset()


# Test init
def test_logger_init_default(isolated_logger: Logger):
    assert all([
        isinstance(isolated_logger.file_level, LogLevel),
        isinstance(isolated_logger.stream_level, LogLevel),
        isolated_logger.file_max_bytes == 0,
        isolated_logger._file_backup_count == 5,
        isolated_logger.buffer_size == 1000,
    ])


def test_logger_init_set(isolated_logger: Logger):
    isolated_logger = Logger(buffer_size=10, file_level=LogLevel.CRITICAL)
    assert all([
        isolated_logger._log_format == "[%(asctime)s] %(levelname)s: %(message)s",
        isolated_logger._buffer_size == 10,
        isolated_logger._file_level == LogLevel.CRITICAL,
    ])


# Test setup logging
def test_setup_logging(isolated_logger: Logger):
    isolated_logger = Logger(
        log_format="[%(levelname)s]",
        file_level=LogLevel.DEBUG,
        stream_level=LogLevel.INFO,
    )
    isolated_logger._setup_logging()
    assert all([
        isolated_logger.level == LogLevel.DEBUG.value,
        isolated_logger._stream_handler.level == LogLevel.INFO.value,
        isolated_logger._stream_handler.formatter._fmt == "[%(levelname)s]",
    ])
    isolated_logger.reset()


# Test setup file handler
def test_setup_file_handler(isolated_logger: Logger):
    assert not hasattr(isolated_logger, "_file_handler")

    isolated_logger._setup_file_handler()

    assert all([
        isolated_logger._file_handler.maxBytes == 0,
        isolated_logger._file_handler.backupCount == 5,
        isolated_logger._file_handler.level == LogLevel.DEBUG.value,
        isolated_logger._file_handler.mode == "w",
        isolated_logger._file_handler.formatter._fmt == "[%(asctime)s] %(levelname)s: %(message)s",
        hasattr(isolated_logger, "_file_handler"),
    ])


# Test ensure file handler
def test_ensure_file_handler_called_once(isolated_logger: Logger):
    original_setup = isolated_logger._setup_file_handler

    isolated_logger._setup_file_handler = Mock()
    isolated_logger._ensure_file_handler()
    isolated_logger._setup_file_handler.assert_called_once()

    isolated_logger._setup_file_handler = original_setup


def test_ensure_file_handler(isolated_logger: Logger):
    assert not hasattr(isolated_logger, "_file_handler")
    isolated_logger._ensure_file_handler()
    assert hasattr(isolated_logger, "_file_handler")

    isolated_logger.reset()


# Test track variable
def test_track_variable(isolated_logger: Logger):
    def test_func() -> list:
        return [1, 2, 3]

    assert test_func() == [1, 2, 3]

    isolated_logger.track_variable(test_func, "Testing")
    assert all([
        test_func in list(isolated_logger._tracked_vars.values()),
        "Testing" in list(isolated_logger._var_names.values()),
    ])


# Test repr
# def test_repr(test_logger: Logger):
#     test_logger._file_path = "newpath"
#     assert test_logger.__repr__() == "Logger(file_path=newpath)"


# Test set file name
def test_set_file_name_str(isolated_logger: Logger):
    isolated_logger.set_file_name("test_file")
    assert all([
        isolated_logger._user_file_name == "test_file",
        isolated_logger._file_path == f"{CURR_DIR}/test_file.log",
        isolated_logger._csv_path == f"{CURR_DIR}/test_file.csv",
    ])


def test_set_file_name_none(isolated_logger: Logger):
    isolated_logger.set_file_name(None)
    assert all([
        isolated_logger._user_file_name is not None,
    ])


# Test set file level
def test_set_file_level(isolated_logger: Logger):
    isolated_logger.set_file_level(LogLevel.CRITICAL)
    assert all([
        isolated_logger._file_level == LogLevel.CRITICAL,
        not hasattr(isolated_logger, "_file_handler"),
    ])


def test_set_file_level_has_attr(isolated_logger: Logger):
    isolated_logger._setup_file_handler()
    assert all([hasattr(isolated_logger, "_file_handler"), isolated_logger._file_handler.mode == "w"])

    isolated_logger.set_file_level(LogLevel.DEBUG)
    assert all([
        isolated_logger._file_level == LogLevel.DEBUG,
        isolated_logger._file_handler.level == LogLevel.DEBUG.value,
    ])


# Test set stream level
def test_set_stream_level(isolated_logger: Logger):
    isolated_logger.set_stream_level(LogLevel.ERROR)
    assert all([
        hasattr(isolated_logger, "_stream_handler"),
        isolated_logger._stream_level == LogLevel.ERROR,
        isolated_logger._stream_handler.level == LogLevel.ERROR.value,
    ])


# Test set format
def test_set_format(isolated_logger: Logger):
    isolated_logger.set_format("[%(levelname)s]")
    assert all([
        isolated_logger._log_format == "[%(levelname)s]",
        isinstance(isolated_logger._std_formatter, logging.Formatter),
        isolated_logger._std_formatter._fmt == "[%(levelname)s]",
        not hasattr(isolated_logger, "_file_handler"),
        isolated_logger._stream_handler.formatter._fmt == "[%(levelname)s]",
    ])


def test_set_format_has_attr(isolated_logger: Logger):
    isolated_logger._setup_file_handler()
    assert all([hasattr(isolated_logger, "_file_handler"), isolated_logger._file_handler.mode == "w"])

    isolated_logger.set_format("[%(test)s]")
    assert isolated_logger._file_handler.formatter._fmt == "[%(test)s]"


# Test set buffer size
def test_set_buffer_size(isolated_logger: Logger):
    isolated_logger.set_buffer_size(5)
    assert all([
        isolated_logger._buffer_size == 5,
        isinstance(isolated_logger._buffer, deque),
        isolated_logger._buffer.maxlen == 5,
    ])


# Test update
def test_update(isolated_logger: Logger):
    def test_func() -> int:
        return 18

    def test_func2() -> int:
        return 8

    assert not isolated_logger._tracked_vars
    isolated_logger.track_variable(test_func, "first")
    isolated_logger.update()
    isolated_logger.track_variable(test_func2, "second")
    isolated_logger.update()
    assert all([
        isolated_logger._buffer[0] == ["18"],
        isolated_logger._buffer[1] == ["18", "8"],
        len(isolated_logger._buffer) == 2,
    ])


# Test update size exceeded
def test_update_size_exceeded(isolated_logger: Logger):
    def test_func() -> int:
        return -2

    isolated_logger.set_buffer_size(2)
    isolated_logger.track_variable(test_func, "test")
    isolated_logger.update()
    assert len(isolated_logger._buffer) == 1

    isolated_logger.track_variable(test_func, "test2")
    isolated_logger.update()
    assert len(isolated_logger._buffer) == 0


# Test flush buffer
def test_flush_buffer(isolated_logger: Logger):
    def test_func() -> int:
        return -2

    isolated_logger.track_variable(test_func, "test")
    isolated_logger.update()
    assert len(isolated_logger._buffer) == 1

    isolated_logger._ensure_file_handler()

    isolated_logger.flush_buffer()
    assert len(isolated_logger._buffer) == 0

    # Ensure expected output was written
    file = open(isolated_logger._csv_path)
    expected = "test\n-2\n"
    assert expected == file.read()
    file.close()


# Test write header
def test_write_header(isolated_logger: Logger):
    isolated_logger.track_variable(lambda: 2, "first")
    isolated_logger.track_variable(lambda: 4, "second")

    isolated_logger._ensure_file_handler()
    isolated_logger._file = open(isolated_logger._csv_path, "w", newline="")
    isolated_logger._writer = csv.writer(isolated_logger._file)
    isolated_logger._write_header()
    isolated_logger.close()
    assert isolated_logger._header_written is True

    # Ensure expected output was written
    file = open(isolated_logger._csv_path)
    header_contents = file.read()
    expected = "first,second\n"
    assert expected == header_contents
    file.close()


# Test generate file paths
def test_generate_file_paths_no_input_filename(isolated_logger: Logger):
    isolated_logger._user_file_name = None
    isolated_logger._generate_file_paths()
    assert all([".log" in isolated_logger._file_path, ".csv" in isolated_logger._csv_path])
    # For timestamp-based file names, portion of string minus script name will be 20 chars
    assert len(isolated_logger._csv_path) >= 20


def test_generate_file_paths_with_input_filename(isolated_logger: Logger):
    isolated_logger._user_file_name = "test_file"
    isolated_logger._generate_file_paths()
    assert isolated_logger._csv_path == f"{isolated_logger.log_path}/test_file.csv"


# Test enter
def test_enter(isolated_logger: Logger):
    assert isolated_logger.__enter__() is isolated_logger


# Test exit
def test_exit(isolated_logger: Logger):
    with isolated_logger:
        # Perform some logging operations
        isolated_logger.track_variable(lambda: 1, "test_var")
        isolated_logger.update()

    # After exiting the context, the logger should be closed
    assert len(isolated_logger._buffer) == 0
    assert isolated_logger._file is None
    assert isolated_logger._writer is None


# Test reset
def test_reset(isolated_logger: Logger):
    isolated_logger.track_variable(lambda: 2, "test")
    isolated_logger.update()
    isolated_logger._setup_file_handler()
    assert all([
        len(isolated_logger._buffer) == 1,
        len(isolated_logger._tracked_vars) == 1,
        len(isolated_logger._var_names) == 1,
        hasattr(isolated_logger, "_file_handler"),
    ])

    isolated_logger.reset()
    assert all([
        len(isolated_logger._buffer) == 0,
        len(isolated_logger._tracked_vars) == 0,
        len(isolated_logger._var_names) == 0,
        hasattr(isolated_logger, "_file_handler"),
    ])


def test_reset_header(isolated_logger: Logger):
    isolated_logger.track_variable(lambda: 2, "test")
    isolated_logger.update()
    isolated_logger.flush_buffer()
    assert isolated_logger._header_written is True

    isolated_logger.reset()
    assert isolated_logger._header_written is False


# Test close
def test_close(isolated_logger: Logger):
    isolated_logger.track_variable(lambda: 2, "first")
    isolated_logger.update()
    isolated_logger.flush_buffer()

    assert isolated_logger._file
    isolated_logger.close()
    assert not isolated_logger._file
    assert not isolated_logger._writer


# Test debug
def test_debug(isolated_logger: Logger):
    original_ensure = isolated_logger._ensure_file_handler
    original_debug = logging.Logger.debug

    isolated_logger._ensure_file_handler = Mock()
    logging.Logger.debug = Mock()
    isolated_logger.debug("debug_test")
    isolated_logger._ensure_file_handler.assert_called_once()
    logging.Logger.debug.assert_called_once_with("debug_test")

    isolated_logger._ensure_file_handler = original_ensure
    logging.Logger.debug = original_debug


# Test info
def test_info(isolated_logger: Logger):
    original_ensure = isolated_logger._ensure_file_handler
    original_info = logging.Logger.info

    isolated_logger._ensure_file_handler = Mock()
    logging.Logger.info = Mock()
    isolated_logger.info("info_test")
    isolated_logger._ensure_file_handler.assert_called_once()
    logging.Logger.info.assert_called_once_with("info_test")

    isolated_logger._ensure_file_handler = original_ensure
    logging.Logger.info = original_info


# Test warning
def test_warning(isolated_logger: Logger):
    original_ensure = isolated_logger._ensure_file_handler
    original_warning = logging.Logger.warning

    isolated_logger._ensure_file_handler = Mock()
    logging.Logger.warning = Mock()
    isolated_logger.warning("warning_test")
    isolated_logger._ensure_file_handler.assert_called_once()
    logging.Logger.warning.assert_called_once_with("warning_test")

    isolated_logger._ensure_file_handler = original_ensure
    logging.Logger.warning = original_warning


# Test error
def test_error(isolated_logger: Logger):
    original_ensure = isolated_logger._ensure_file_handler
    original_error = logging.Logger.error

    isolated_logger._ensure_file_handler = Mock()
    logging.Logger.error = Mock()
    isolated_logger.error("error_test")
    isolated_logger._ensure_file_handler.assert_called_once()
    logging.Logger.error.assert_called_once_with("error_test")

    isolated_logger._ensure_file_handler = original_ensure
    logging.Logger.error = original_error


# Test critical
def test_critical(isolated_logger: Logger):
    original_ensure = isolated_logger._ensure_file_handler
    original_critical = logging.Logger.critical

    isolated_logger._ensure_file_handler = Mock()
    logging.Logger.critical = Mock()
    isolated_logger.critical("critical_test")
    isolated_logger._ensure_file_handler.assert_called_once()
    logging.Logger.critical.assert_called_once_with("critical_test")

    isolated_logger._ensure_file_handler = original_ensure
    logging.Logger.critical = original_critical


# Test log
def test_log(isolated_logger: Logger):
    original_ensure = isolated_logger._ensure_file_handler
    original_log = logging.Logger.log

    isolated_logger._ensure_file_handler = Mock()
    logging.Logger.log = Mock()
    isolated_logger.log(LogLevel.DEBUG, "log_test")
    isolated_logger._ensure_file_handler.assert_called_once()
    logging.Logger.log.assert_called_once_with(LogLevel.DEBUG, "log_test")

    isolated_logger._ensure_file_handler = original_ensure
    logging.Logger.log = original_log


# # Test file path
# def test_file_path(test_logger: Logger):
#     original_generate = test_logger._generate_file_paths

#     test_logger._generate_file_paths = Mock()
#     test_logger._file_path = ""
#     test_logger._generate_file_paths.assert_called_once()

#     test_logger._generate_file_paths = original_generate


# Test buffer size
def test_buffer_size(isolated_logger: Logger):
    isolated_logger.set_buffer_size(5)
    assert isolated_logger.buffer_size == 5


# Test file level
def test_file_level(isolated_logger: Logger):
    isolated_logger.set_file_level(LogLevel.WARNING)
    assert isolated_logger.file_level == LogLevel.WARNING


# Test stream level
def test_stream_level(isolated_logger: Logger):
    assert isolated_logger._file is None
    isolated_logger.set_stream_level(LogLevel.INFO)
    assert isolated_logger.stream_level == LogLevel.INFO


# Test file max bytes
def test_max_bytes():
    isolated_logger = Logger(file_max_bytes=200)
    assert isolated_logger.file_max_bytes == 200
    isolated_logger.reset()


# Test file backup count
def test_file_backup_count():
    isolated_logger = Logger(file_backup_count=10)
    assert isolated_logger.file_backup_count == 10
    isolated_logger.reset()


# Test initialized global logger
def test_global():
    assert isinstance(LOGGER, Logger)
