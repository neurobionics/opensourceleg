import pytest

from opensourceleg.logger import Logger


class Simple_Class:
    def __init__(self):
        self.a = 1
        self.b = 2
        self.c = 3


def test_init():
    test_Logger_default = Logger(file_path="tests/test_logger/test_log_default")
    assert test_Logger_default._class_instances == []
    assert test_Logger_default._attributes == []


def test_set_file_level():
    test_Logger_file_level = Logger(file_path="tests/test_logger/test_log_file_level")
    test_Logger_file_level.set_file_level(level="DEBUG")
    assert test_Logger_file_level._file_handler.level == 10
    test_Logger_file_level.set_file_level(level="INFO")
    assert test_Logger_file_level._file_handler.level == 20
    test_Logger_file_level.set_file_level(level="WARNING")
    assert test_Logger_file_level._file_handler.level == 30
    test_Logger_file_level.set_file_level(level="ERROR")
    assert test_Logger_file_level._file_handler.level == 40
    test_Logger_file_level.set_file_level(level="CRITICAL")
    assert test_Logger_file_level._file_handler.level == 50
    with pytest.raises(KeyError):
        test_Logger_file_level.set_file_level(level="debug")


def test_set_stream_level():
    test_Logger_stream_level = Logger(
        file_path="tests/test_logger/test_log_stream_level"
    )
    test_Logger_stream_level.set_stream_level(level="DEBUG")
    assert test_Logger_stream_level._stream_handler.level == 10
    test_Logger_stream_level.set_stream_level(level="INFO")
    assert test_Logger_stream_level._stream_handler.level == 20
    test_Logger_stream_level.set_stream_level(level="WARNING")
    assert test_Logger_stream_level._stream_handler.level == 30
    test_Logger_stream_level.set_stream_level(level="ERROR")
    assert test_Logger_stream_level._stream_handler.level == 40
    test_Logger_stream_level.set_stream_level(level="CRITICAL")
    assert test_Logger_stream_level._stream_handler.level == 50
    with pytest.raises(KeyError):
        test_Logger_stream_level.set_stream_level(level="BAD LEVEL")


def test_add_attributes():
    test_class_instance = Simple_Class()
    test_class_instance2 = Simple_Class()

    test_Logger1 = Logger(file_path="tests/test_logger/test_log1")
    test_Logger1.add_attributes(
        class_instance=test_class_instance, attributes_str=["a", "b", "c"]
    )

    assert test_Logger1._class_instances == [test_class_instance]
    assert test_Logger1._attributes == [["a", "b", "c"]]
    test_Logger1.add_attributes(
        class_instance=test_class_instance2, attributes_str=["a", "b", "c"]
    )
    assert test_Logger1._class_instances == [test_class_instance, test_class_instance2]
    assert test_Logger1._attributes == [["a", "b", "c"], ["a", "b", "c"]]


def test_close():
    test_Logger3 = Logger(file_path="tests/test_logger/test_log3")
    test_Logger3.close()
    assert test_Logger3._file.closed == True
