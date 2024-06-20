import csv

import pytest

from opensourceleg.tools.logger import Logger


class Simple_Class:
    """
    Simple class to use for testing the Logger class
    """

    def __init__(self):
        self.a = 1
        self.b = 2
        self.c = 3


def test_init():
    """
    Tests the Logger constrctor\n
    Asserts the constructor works properly when passed a valid file_path
    """

    test_Logger_default = Logger(file_path="tests/test_logger/test_log_default")
    assert test_Logger_default._containers == []
    assert test_Logger_default._attributes == []


def test_set_file_level():
    """
    Tests the Logger set_file_level method\n
    Asserts the proper level is set when a valid level is passed and the proper
    error is raised when an invalid level is passed.
    """

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


def test_set_stream_level():
    """
    Tests the Logger set_stream_level method\n
    Asserts the proper level is set when a valid level is passed and the proper
    error is raised when an invalid level is passed.
    """

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


def test_add_attributes():
    """
    Tests the Logger add_attributes method\n
    Asserts the method works properly when passed a single class instance and
    when passed multiple class instances.
    """

    # Initializes SimpleClass instances to use in the following tests
    test_container = Simple_Class()
    test_container2 = Simple_Class()

    # Asserts the add_attributes method works properly when passed a single class instance
    test_Logger1 = Logger(file_path="tests/test_logger/test_log1")
    test_Logger1.add_attributes(container=test_container, attributes=["a", "b", "c"])

    # Asserts the add_attributes method works properly when a class instance is passed multiple times
    assert test_Logger1._containers == [test_container]
    assert test_Logger1._attributes == [["a", "b", "c"]]
    test_Logger1.add_attributes(container=test_container2, attributes=["a", "b", "c"])
    assert test_Logger1._containers == [test_container, test_container2]
    assert test_Logger1._attributes == [["a", "b", "c"], ["a", "b", "c"]]


def test_data():
    """
    Tests the Logger data method\n
    This test initializes a Logger instance and then calls the add_attributes
    method to add attributes to the Logger instance. Then the data method is
    called and the data is asserted to be correct. This is then repeated to ensure
    the method works properly when passed multiple class instances.
    """

    # Initializes SimpleClass instances to use in the following tests
    test_container = Simple_Class()
    test_container2 = Simple_Class()
    test_Logger_data = Logger(file_path="tests/test_logger/test_log_data")
    test_Logger_data.add_attributes(
        container=test_container, attributes=["a", "b", "c"]
    )
    # Asserts the data method works properly when passed a single class instance
    test_Logger_data.update()
    expected_rows = [["a", "b", "c"], ["1", "2", "3"]]
    with open("tests/test_logger/test_log_data.csv", newline="") as f:
        reader = csv.reader(f)
        rows = list(reader)
        assert rows == expected_rows

    # Asserts the data method works properly when passed a custom name
    test_Logger_data = Logger(file_path="tests/test_logger/test_log_data")
    test_Logger_data.add_attributes(
        container=test_container2,
        attributes=["a", "b", "c"],
        container_name="custom_name",
    )
    test_Logger_data.update()
    expected_rows2 = [
        ["custom_name:a", "custom_name:b", "custom_name:c"],
        ["1", "2", "3"],
    ]
    with open("tests/test_logger/test_log_data.csv", newline="") as f:
        reader = csv.reader(f)
        rows2 = list(reader)
    assert rows2 == expected_rows2

    # Asserts the data method works properly when passed multiple instances, including locals
    test_Logger_data = Logger(file_path="tests/test_logger/test_log_data")
    test_Logger_data.add_attributes(
        container=test_container, attributes=["a", "b", "c"]
    )
    test_Logger_data.add_attributes(
        container=test_container2,
        attributes=["a", "b", "c"],
        container_name="custom_name",
    )
    pi = 3.14159
    test_Logger_data.add_attributes(locals(), ["pi"], "local")
    test_Logger_data.update()
    expected_rows2 = [
        ["a", "b", "c", "custom_name:a", "custom_name:b", "custom_name:c", "local:pi"],
        ["1", "2", "3", "1", "2", "3", "3.14159"],
    ]
    with open("tests/test_logger/test_log_data.csv", newline="") as f:
        reader = csv.reader(f)
        rows2 = list(reader)
    assert rows2 == expected_rows2
