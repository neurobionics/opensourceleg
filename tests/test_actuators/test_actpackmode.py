import pytest

from opensourceleg.hardware.actuators import dephy as act


class MockDephyActuator:
    """
    Mocked DephyActuator class for testing the ActpackMode class.
    """

    def __init__(self):
        pass


@pytest.fixture
def mock_DephyActuator() -> MockDephyActuator:
    return MockDephyActuator()


def test_ActpackMode(mock_DephyActuator):
    """
    Test function which initializes a MockDephyActuator object and tests the
    ActpackMode constructor, equality, string representation, and properties.
    It also overwrites the entry and exit callbacks and tests the enter, exit,
    and transition methods.
    """
    mock_DephyActuator = MockDephyActuator()
    control_mode1 = 1
    control_mode2 = 2

    test_mode1 = act.ActpackMode(control_mode1, mock_DephyActuator)
    test_mode2 = act.ActpackMode(control_mode2, mock_DephyActuator)
    test_mode2._has_gains = True
    new_instance_test_mode1 = act.ActpackMode(control_mode1, None)

    # Testing the ActpackMode constructor
    assert test_mode1._control_mode == control_mode1
    assert test_mode1._device == mock_DephyActuator
    assert test_mode1._has_gains == False
    assert test_mode2.has_gains == True

    # Testing the ActpackMode equality
    assert (test_mode1 == test_mode2) == False
    assert test_mode1 == new_instance_test_mode1
    assert (test_mode1 == "ActpackMode") == False

    # Testing the ActpackMode string representation
    assert str(test_mode1) == "1"
    assert str(test_mode2) == "2"
    assert str(new_instance_test_mode1) == "1"

    # Testing the ActpackMode mode property
    assert test_mode1.mode == 1
    assert test_mode2.mode == 2
    assert new_instance_test_mode1.mode == 1

    # Testing the ActpackMode has_gains property
    assert test_mode1.has_gains == False
    assert test_mode2.has_gains == True
    assert new_instance_test_mode1.has_gains == False

    # Testing the ActpackMode enter method
    entry_callback_counter = 0

    def my_entry_callback():
        nonlocal entry_callback_counter
        entry_callback_counter += 1

    test_mode1._entry_callback = my_entry_callback
    test_mode1.enter()
    assert entry_callback_counter == 1

    # Testing the ActpackMode exit method
    exit_callback_counter = 0

    def my_exit_callback():
        nonlocal exit_callback_counter
        exit_callback_counter += 1

    test_mode1._exit_callback = my_exit_callback
    test_mode1.exit()
    assert exit_callback_counter == 1

    # Testing the ActpackMode transition method
    entry_callback_counter = 0

    def my_entry_callback():
        nonlocal entry_callback_counter
        entry_callback_counter += 1

    test_mode2._entry_callback = my_entry_callback

    exit_callback_counter = 0

    def my_exit_callback():
        nonlocal exit_callback_counter
        exit_callback_counter += 1

    test_mode1._exit_callback = my_exit_callback

    test_mode1.transition(test_mode2)

    assert entry_callback_counter == 1
    assert exit_callback_counter == 1
