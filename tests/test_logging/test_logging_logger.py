from unittest.mock import Mock

import pytest

from opensourceleg.logging.logger import *


# Test LogLevel class
def test_log_level_default():
    {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"} <= {e.name for e in LogLevel}


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


# Test Logger class
@pytest.fixture(scope="function")
def test_logger():
    log = Logger()
    log.reset()
    yield log


# Test new
def test_logger_new():
    logger = Logger()
    logger2 = Logger()
    assert logger is logger2
    logger.reset()
    logger2.reset()


# Test init
def test_logger_init_default():
    logger = Logger()
    assert all([
        logger._log_path == "./",
        isinstance(logger.file_level, LogLevel),
        isinstance(logger.stream_level, LogLevel),
        logger.file_max_bytes == 0,
        logger._file_backup_count == 5,
        logger.buffer_size == 1000,
    ])
    logger.reset()


def test_logger_init_set():
    logger = Logger(buffer_size=10, file_level=LogLevel.CRITICAL)
    assert all([
        logger._log_format == "[%(asctime)s] %(levelname)s: %(message)s",
        logger._buffer_size == 10,
        logger._file_level == LogLevel.CRITICAL,
    ])
    logger.reset()


# Test setup logging
def test_setup_logging():
    test_logger = Logger(
        log_format="[%(levelname)s]",
        file_level=LogLevel.DEBUG,
        stream_level=LogLevel.INFO,
    )
    test_logger._setup_logging()
    assert all([
        test_logger.level == LogLevel.DEBUG.value,
        test_logger._stream_handler.level == LogLevel.INFO.value,
        test_logger._stream_handler.formatter._fmt == "[%(levelname)s]",
    ])
    test_logger.reset()


# Test setup file handler
def test_setup_file_handler():
    test_logger = Logger(
        file_max_bytes=0,
        file_backup_count=20,
        file_level=LogLevel.WARNING,
        log_format="[%(levelname)s]",
    )
    assert not hasattr(test_logger, "_file_handler")

    test_logger._setup_file_handler()

    assert all([
        test_logger._file_handler.maxBytes == 0,
        test_logger._file_handler.backupCount == 20,
        test_logger._file_handler.level == LogLevel.WARNING.value,
        test_logger._file_handler.mode == "a",
        test_logger._file_handler.formatter._fmt == "[%(levelname)s]",
        hasattr(test_logger, "_file_handler"),
    ])

    test_logger.reset()


# Test ensure file handler
def test_ensure_file_handler_called_once(test_logger: Logger):
    original_setup = test_logger._setup_file_handler

    test_logger._setup_file_handler = Mock()
    test_logger._ensure_file_handler()
    test_logger._setup_file_handler.assert_called_once()

    test_logger._setup_file_handler = original_setup


def test_ensure_file_handler(test_logger: Logger):
    assert not hasattr(test_logger, "_file_handler")
    test_logger._ensure_file_handler()
    assert hasattr(test_logger, "_file_handler")

    test_logger.reset()


# Test track & untrack variable
def test_track_and_untrack_variable(test_logger: Logger):
    def test_func() -> list:
        return [1, 2, 3]

    assert test_func() == [1, 2, 3]

    test_logger.track_variable(test_func, "Testing")
    assert all([
        test_func in list(test_logger._tracked_vars.values()),
        "Testing" in list(test_logger._var_names.values()),
    ])

    test_logger.untrack_variable(test_func)
    assert all([
        test_func not in list(test_logger._tracked_vars.values()),
        "Testing" not in list(test_logger._var_names.values()),
    ])


# Test repr
def test_repr(test_logger: Logger):
    test_logger._file_path = "newpath"
    assert test_logger.__repr__() == "Logger(file_path=newpath)"


# Test set file name
def test_set_file_name_str(test_logger: Logger):
    test_logger.set_file_name("test_file")
    assert all([
        test_logger._user_file_name == "test_file",
        test_logger._file_path == "",
        test_logger._csv_path == "",
    ])


def test_set_file_name_none(test_logger: Logger):
    test_logger.set_file_name(None)
    assert all([
        test_logger._user_file_name == None,
        test_logger._file_path == "",
        test_logger._csv_path == "",
    ])


# Test set file level
def test_set_file_level(test_logger: Logger):
    test_logger.set_file_level(LogLevel.CRITICAL)
    assert all([
        test_logger._file_level == LogLevel.CRITICAL,
        not hasattr(test_logger, "_file_handler"),
    ])


def test_set_file_level_has_attr(test_logger: Logger):
    test_logger._setup_file_handler()
    assert all([hasattr(test_logger, "_file_handler"), test_logger._file_handler.mode == "a"])

    test_logger.set_file_level(LogLevel.DEBUG)
    assert all([
        test_logger._file_level == LogLevel.DEBUG,
        test_logger._file_handler.level == LogLevel.DEBUG.value,
    ])


# Test set stream level
def test_set_stream_level(test_logger: Logger):
    test_logger.set_stream_level(LogLevel.ERROR)
    assert all([
        hasattr(test_logger, "_stream_handler"),
        test_logger._stream_level == LogLevel.ERROR,
        test_logger._stream_handler.level == LogLevel.ERROR.value,
    ])


# Test set format
def test_set_format(test_logger: Logger):
    test_logger.set_format("[%(levelname)s]")
    assert all([
        test_logger._log_format == "[%(levelname)s]",
        isinstance(test_logger._std_formatter, logging.Formatter),
        test_logger._std_formatter._fmt == "[%(levelname)s]",
        not hasattr(test_logger, "_file_handler"),
        test_logger._stream_handler.formatter._fmt == "[%(levelname)s]",
    ])


def test_set_format_has_attr(test_logger: Logger):
    test_logger._setup_file_handler()
    assert all([hasattr(test_logger, "_file_handler"), test_logger._file_handler.mode == "a"])

    test_logger.set_format("[%(test)s]")
    assert test_logger._file_handler.formatter._fmt == "[%(test)s]"


# Test set buffer size
def test_set_buffer_size(test_logger: Logger):
    test_logger.set_buffer_size(5)
    assert all([
        test_logger._buffer_size == 5,
        isinstance(test_logger._buffer, deque),
        test_logger._buffer.maxlen == 5,
    ])


# Test update
def test_update(test_logger: Logger):
    def test_func() -> int:
        return 18

    def test_func2() -> int:
        return 8

    assert not test_logger._tracked_vars
    test_logger.track_variable(test_func, "first")
    test_logger.update()
    test_logger.track_variable(test_func2, "second")
    test_logger.update()
    assert all([
        test_logger._buffer[0] == ["18"],
        test_logger._buffer[1] == ["18", "8"],
        len(test_logger._buffer) == 2,
    ])


# Test update size exceeded
def test_update_size_exceeded(test_logger: Logger):
    def test_func() -> int:
        return -2

    test_logger.set_buffer_size(2)
    test_logger.track_variable(test_func, "test")
    test_logger.update()
    assert len(test_logger._buffer) == 1

    test_logger.track_variable(test_func, "test2")
    test_logger.update()
    assert len(test_logger._buffer) == 0


# Test flush buffer
def test_flush_buffer(test_logger: Logger):
    def test_func() -> int:
        return -2

    test_logger.track_variable(test_func, "test")
    test_logger.update()
    assert len(test_logger._buffer) == 1

    test_logger._ensure_file_handler()

    # Clear the file to start since it is being used by other tests
    test_logger._file = open(test_logger._csv_path, "w", newline="")
    test_logger.close()

    test_logger.flush_buffer()
    assert len(test_logger._buffer) == 0

    # Ensure expected output was written
    file = open(test_logger._csv_path)
    expected = "test\n-2\n"
    assert expected == file.read()
    file.close()


# Test write header
def test_write_header(test_logger: Logger):
    test_logger.track_variable(lambda: 2, "first")
    test_logger.track_variable(lambda: 4, "second")

    test_logger._ensure_file_handler()
    test_logger._file = open(test_logger._csv_path, "w", newline="")
    test_logger._writer = csv.writer(test_logger._file)
    test_logger._write_header()
    test_logger.close()
    assert test_logger._header_written == True

    # Ensure expected output was written
    file = open(test_logger._csv_path)
    header_contents = file.read()
    expected = "first,second\n"
    assert expected == header_contents
    file.close()


# Test generate file paths
def test_generate_file_paths_no_input_filename(test_logger: Logger):
    test_logger._user_file_name = None
    test_logger._generate_file_paths()
    assert all([".log" in test_logger._file_path, ".csv" in test_logger._csv_path])
    # For timestamp-based file names, portion of string minus script name will be 20 chars
    assert len(test_logger._csv_path) >= 20


def test_generate_file_paths_with_input_filename(test_logger: Logger):
    test_logger._user_file_name = "test_file"
    test_logger._generate_file_paths()
    assert test_logger._csv_path == "./test_file.csv"


# Test enter
def test_enter(test_logger: Logger):
    assert isinstance(test_logger.__enter__(), Logger)
    assert test_logger.__enter__() is test_logger


# Test exit
def test_exit(test_logger: Logger):
    test_logger.track_variable(lambda: 2, "first")
    test_logger.update()

    original_flush = test_logger.flush_buffer
    original_close = test_logger.close
    test_logger.flush_buffer = Mock()
    test_logger.close = Mock()

    test_logger.__exit__(1, 1, 1)
    test_logger.flush_buffer.assert_called_once()
    test_logger.close.assert_called_once()

    test_logger.flush_buffer = original_flush
    test_logger.close = original_close


# Test reset
def test_reset(test_logger: Logger):
    test_logger.track_variable(lambda: 2, "test")
    test_logger.update()
    test_logger._setup_file_handler()
    assert all([
        len(test_logger._buffer) == 1,
        len(test_logger._tracked_vars) == 1,
        len(test_logger._var_names) == 1,
        hasattr(test_logger, "_file_handler"),
    ])

    test_logger.reset()
    assert all([
        len(test_logger._buffer) == 0,
        len(test_logger._tracked_vars) == 0,
        len(test_logger._var_names) == 0,
        not hasattr(test_logger, "_file_handler"),
    ])


def test_reset_header(test_logger: Logger):
    test_logger.track_variable(lambda: 2, "test")
    test_logger.update()
    test_logger.flush_buffer()
    assert test_logger._header_written == True

    test_logger.reset()
    assert test_logger._header_written == False


# Test close
def test_close(test_logger: Logger):
    test_logger.track_variable(lambda: 2, "first")
    test_logger.update()
    test_logger.flush_buffer()

    assert test_logger._file
    test_logger.close()
    assert not test_logger._file
    assert not test_logger._writer


# Test debug
def test_debug(test_logger: Logger):
    original_ensure = test_logger._ensure_file_handler
    original_debug = logging.Logger.debug

    test_logger._ensure_file_handler = Mock()
    logging.Logger.debug = Mock()
    test_logger.debug("debug_test")
    test_logger._ensure_file_handler.assert_called_once()
    logging.Logger.debug.assert_called_once_with("debug_test")

    test_logger._ensure_file_handler = original_ensure
    logging.Logger.debug = original_debug


# Test info
def test_info(test_logger: Logger):
    original_ensure = test_logger._ensure_file_handler
    original_info = logging.Logger.info

    test_logger._ensure_file_handler = Mock()
    logging.Logger.info = Mock()
    test_logger.info("info_test")
    test_logger._ensure_file_handler.assert_called_once()
    logging.Logger.info.assert_called_once_with("info_test")

    test_logger._ensure_file_handler = original_ensure
    logging.Logger.info = original_info


# Test warning
def test_warning(test_logger: Logger):
    original_ensure = test_logger._ensure_file_handler
    original_warning = logging.Logger.warning

    test_logger._ensure_file_handler = Mock()
    logging.Logger.warning = Mock()
    test_logger.warning("warning_test")
    test_logger._ensure_file_handler.assert_called_once()
    logging.Logger.warning.assert_called_once_with("warning_test")

    test_logger._ensure_file_handler = original_ensure
    logging.Logger.warning = original_warning


# Test error
def test_error(test_logger: Logger):
    original_ensure = test_logger._ensure_file_handler
    original_error = logging.Logger.error

    test_logger._ensure_file_handler = Mock()
    logging.Logger.error = Mock()
    test_logger.error("error_test")
    test_logger._ensure_file_handler.assert_called_once()
    logging.Logger.error.assert_called_once_with("error_test")

    test_logger._ensure_file_handler = original_ensure
    logging.Logger.error = original_error


# Test critical
def test_critical(test_logger: Logger):
    original_ensure = test_logger._ensure_file_handler
    original_critical = logging.Logger.critical

    test_logger._ensure_file_handler = Mock()
    logging.Logger.critical = Mock()
    test_logger.critical("critical_test")
    test_logger._ensure_file_handler.assert_called_once()
    logging.Logger.critical.assert_called_once_with("critical_test")

    test_logger._ensure_file_handler = original_ensure
    logging.Logger.critical = original_critical


# Test log
def test_log(test_logger: Logger):
    original_ensure = test_logger._ensure_file_handler
    original_log = logging.Logger.log

    test_logger._ensure_file_handler = Mock()
    logging.Logger.log = Mock()
    test_logger.log(LogLevel.DEBUG, "log_test")
    test_logger._ensure_file_handler.assert_called_once()
    logging.Logger.log.assert_called_once_with(LogLevel.DEBUG, "log_test")

    test_logger._ensure_file_handler = original_ensure
    logging.Logger.log = original_log


# Test file path
def test_file_path(test_logger: Logger):
    original_generate = test_logger._generate_file_paths

    test_logger._generate_file_paths = Mock()
    test_logger._file_path = ""
    filename = test_logger.file_path
    test_logger._generate_file_paths.assert_called_once()

    test_logger._generate_file_paths = original_generate


# Test buffer size
def test_buffer_size(test_logger: Logger):
    test_logger.set_buffer_size(5)
    assert test_logger.buffer_size == 5


# Test file level
def test_file_level(test_logger: Logger):
    test_logger.set_file_level(LogLevel.WARNING)
    assert test_logger.file_level == LogLevel.WARNING


# Test stream level
def test_stream_level(test_logger: Logger):
    test_logger.set_stream_level(LogLevel.INFO)
    assert test_logger.stream_level == LogLevel.INFO


# Test file max bytes
def test_max_bytes():
    test_logger = Logger(file_max_bytes=200)
    assert test_logger.file_max_bytes == 200
    test_logger.reset()


# Test file backup count
def test_file_backup_count():
    test_logger = Logger(file_backup_count=10)
    assert test_logger.file_backup_count == 10
    test_logger.reset()


# Test initialized global logger
def test_global():
    assert isinstance(LOGGER, Logger)
