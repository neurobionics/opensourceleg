import csv

import pytest

from opensourceleg.logger import Logger


# Defining a simple class for testing
class Simple_Class:
    def __init__(self):
        self.a = 1
        self.b = 2
        self.c = 3


# Testing the Logger constructor
def test_init():
    test_Logger_default = Logger(file_path="tests/test_logger/test_log_default")
    assert test_Logger_default._class_instances == []
    assert test_Logger_default._attributes == []


# Testing the Logger set_file_level method
def test_set_file_level():
    # Asserts the proper level is set when a valid level is passed
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
    # Asserts the proper error is raised when an invalid level is passed
    with pytest.raises(KeyError):
        test_Logger_file_level.set_file_level(level="debug")


# Testing the Logger set_stream_level method
def test_set_stream_level():
    # Asserts the proper level is set when a valid level is passed
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
    # Asserts the proper error is raised when an invalid level is passed
    with pytest.raises(KeyError):
        test_Logger_stream_level.set_stream_level(level="BAD LEVEL")


# Testing the Logger add_attributes method
def test_add_attributes():
    # Initializes SimpleClass instances to use in the following tests
    test_class_instance = Simple_Class()
    test_class_instance2 = Simple_Class()

    # Asserts the add_attributes method works properly when passed a single class instance
    test_Logger1 = Logger(file_path="tests/test_logger/test_log1")
    test_Logger1.add_attributes(
        class_instance=test_class_instance, attributes_str=["a", "b", "c"]
    )

    # Asserts the add_attributes method works properly when a class instance is passed multiple times
    assert test_Logger1._class_instances == [test_class_instance]
    assert test_Logger1._attributes == [["a", "b", "c"]]
    test_Logger1.add_attributes(
        class_instance=test_class_instance2, attributes_str=["a", "b", "c"]
    )
    assert test_Logger1._class_instances == [test_class_instance, test_class_instance2]
    assert test_Logger1._attributes == [["a", "b", "c"], ["a", "b", "c"]]


# Testing the Logger close method
def test_close():
    test_Logger3 = Logger(file_path="tests/test_logger/test_log3")
    test_Logger3.close()
    assert test_Logger3._file.closed == True


# Testing the Logger data method
def test_data():
    # Initializes SimpleClass instances to use in the following tests
    test_class_instance = Simple_Class()
    test_class_instance2 = Simple_Class()
    test_Logger_data = Logger(file_path="tests/test_logger/test_log_data")
    test_Logger_data.add_attributes(
        class_instance=test_class_instance, attributes_str=["a", "b", "c"]
    )
    # Asserts the data method works properly when passed a single class instance
    test_Logger_data.data()
    expected_rows = [["a", "b", "c"], ["1", "2", "3"]]
    with open("tests/test_logger/test_log_data.csv", "r", newline="") as f:
        reader = csv.reader(f)
        rows = list(reader)
        assert rows == expected_rows

    test_Logger_data.add_attributes(
        class_instance=test_class_instance2, attributes_str=["a", "b", "c"]
    )
    # Asserts the data method works properly when passed multiple class instances
    test_Logger_data.data()
    expected_rows2 = [["a", "b", "c"], ["1", "2", "3"], ["1", "2", "3", "1", "2", "3"]]
    with open("tests/test_logger/test_log_data.csv", "r", newline="") as f:
        reader = csv.reader(f)
        rows2 = list(reader)
    assert rows2 == expected_rows2
