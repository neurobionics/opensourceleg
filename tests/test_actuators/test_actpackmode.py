from typing import Callable

from ctypes import c_int

import pytest

from opensourceleg import actuators as act


class MockDephyActpack:
    def __init__(self):
        pass

    def set_mode(self, mode):
        pass

    def set_motor_zero_position(self):
        pass


@pytest.fixture
def mock_dephyactpack():
    return MockDephyActpack()


def test_actpackmode(mock_dephyactpack):
    control_mode1 = 1
    control_mode2 = 2

    test_mode1 = act.ActpackMode(control_mode1, mock_dephyactpack)
    test_mode2 = act.ActpackMode(control_mode2, mock_dephyactpack)
    test_mode2._has_gains = True
    new_instance_test_mode1 = act.ActpackMode(control_mode1, None)

    # Testing the ActpackMode constructor
    assert test_mode1._control_mode == control_mode1
    assert test_mode1._device == mock_dephyactpack
    assert test_mode1._has_gains == False
    assert test_mode2.has_gains == True

    # Testing the ActpackMode equality
    assert (test_mode1 == test_mode2) == False
    assert test_mode1 == new_instance_test_mode1

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

    # assert test_mode1.enter() == "entry"
    # assert test_mode1.exit() == "exit"
    # assert test_mode2.enter() == "entry"
    # assert test_mode2.exit() == "exit"
