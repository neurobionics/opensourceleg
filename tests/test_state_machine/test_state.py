from typing import Callable

import time
from dataclasses import field

import pytest

from opensourceleg.logger import Logger
from opensourceleg.state_machine import (
    Event,
    FromToTransition,
    Idle,
    State,
    StateMachine,
    Transition,
)


# Test the State constructor
def test_state_init():
    # Initialize the State object and assert the attributes are properly set
    test_state = State()
    assert test_state._name == "state"
    assert test_state._is_knee_active == False
    assert test_state._knee_stiffness == 0.0
    assert test_state._knee_damping == 0.0
    assert test_state._knee_theta == 0.0
    assert test_state._is_ankle_active == False
    assert test_state._ankle_stiffness == 0.0
    assert test_state._ankle_damping == 0.0
    assert test_state._ankle_theta == 0.0
    assert test_state._time_entered == 0.0
    assert test_state._time_exited == 0.0
    assert test_state._min_time_in_state == 2.0


# Test the State __eq__ method
def test_state_eq():
    # Test the if statement
    test_state1 = State()
    test_state2 = State()
    assert test_state1 == test_state2
    # Test the else statement
    test_state1._name = "test"
    assert (test_state1 == test_state2) == False


# Test the State __ne__ method
def test_state_ne():
    test_state3 = State()
    test_state4 = State()
    assert (test_state3 != test_state4) == False
    test_state3._name = "test"
    assert test_state3 != test_state4


# Test the State set_minimum_time_spent_in_state method
def test_state_set_minimum_time_spent_in_state():
    test_state_smtsis = State()
    # Assert the default value is 2.0
    assert test_state_smtsis._min_time_in_state == 2.0
    # Assert the value is set properly
    test_state_smtsis.set_minimum_time_spent_in_state(1.0)
    assert test_state_smtsis._min_time_in_state == 1.0


# Test the State set_knee_impedance_paramters method
def test_state_set_knee_impedance_parameters():
    test_state_skip = State()
    # Assert the default values are set properly
    assert test_state_skip._knee_theta == 0.0
    assert test_state_skip._knee_stiffness == 0.0
    assert test_state_skip._knee_damping == 0.0
    # Assert the values are set properly
    test_state_skip.set_knee_impedance_paramters(1.0, 2.0, 3.0)
    assert test_state_skip._knee_theta == 1.0
    assert test_state_skip._knee_stiffness == 2.0
    assert test_state_skip._knee_damping == 3.0


# Test the State set_ankle_impedance_paramters method
def test_state_set_ankle_impedance_parameters():
    test_state_skip = State()
    # Assert the default values are set properly
    assert test_state_skip._ankle_theta == 0.0
    assert test_state_skip._ankle_stiffness == 0.0
    assert test_state_skip._ankle_damping == 0.0
    # Assert the values are set properly
    test_state_skip.set_ankle_impedance_paramters(1.0, 2.0, 3.0)
    assert test_state_skip._ankle_theta == 1.0
    assert test_state_skip._ankle_stiffness == 2.0
    assert test_state_skip._ankle_damping == 3.0


# Test the State set_custom_data and get_custom_data methods
def test_state_set_and_get_custom_data():
    test_state_scd = State()
    test_state_scd._custom_data = {}
    # Assert the custom data is set properly
    test_state_scd.set_custom_data("name", "test")
    assert test_state_scd._custom_data["name"] == "test"
    # Assert the custom data is retrieved properly
    assert test_state_scd.get_custom_data("name") == "test"
    # Assert the overwrite works properly
    test_state_scd.set_custom_data("name", "test2")
    assert test_state_scd._custom_data["name"] == "test2"
    # Assert the custom data is retrieved properly
    assert test_state_scd.get_custom_data("name") == "test2"


# Test the State on_entry method
def test_state_on_entry():
    test_state_soe = State()
    # Create a callback function to be used in the on_entry method
    def callback(data):
        print("Called", data)

    # Assert the on_entry method works properly
    test_state_soe.on_entry(callback)
    assert callback in test_state_soe._entry_callbacks
    # Create another callback function to be used in the on_entry method
    def callback2(data):
        print("Called2", data)

    # Assert the on_entry method works properly with multiple callbacks
    test_state_soe.on_entry(callback2)
    assert callback in test_state_soe._entry_callbacks
    assert callback2 in test_state_soe._entry_callbacks


# Test the State on_exit method
def test_state_on_exit():
    test_state_soe = State()
    # Create a callback function to be used in the on_exit method
    def callback(data):
        print("Called", data)

    # Assert the on_exit method works properly
    test_state_soe.on_exit(callback)
    assert callback in test_state_soe._exit_callbacks
    # Create another callback function to be used in the on_exit method
    def callback2(data):
        print("Called2", data)

    # Assert the on_exit method works properly with multiple callbacks
    test_state_soe.on_exit(callback2)
    assert callback in test_state_soe._exit_callbacks
    assert callback2 in test_state_soe._exit_callbacks


# Patch the time.time method to return 1.0
@pytest.fixture
def mock_time(monkeypatch):
    monkeypatch.setattr(time, "time", lambda: 1.0)


# Create a test function that logs it was called to a log
def fake_callback(name: str):
    callback_log = Logger(file_path="tests/test_state_machine/{}".format(name))
    callback_log.set_stream_level(level="DEBUG")
    callback_log.debug(f"{name} was called")


# Unfinished
# Test the State start method
def test_state_start():
    pass
    # test_state_start = State()
    # # Add the fake_callback using the on_entry method
    # test_state_start._entry_callbacks.append(fake_callback(name="test_state_start"))
    # # Assert the time_entered is set properly
    # test_state_start.start(data="test_data")
    # assert test_state_start._time_entered == 1.0
    # assert fake_callback in test_state_start._entry_callbacks
    # with open("tests/test_state_machine/test_state_start.log", "r") as f:
    #     contents = f.read()
    #     assert "DEBUG: test_state_start was called" in contents


# Unfinished
# Test the State stop method
def test_state_stop():
    pass


# Test the State make_knee_active method
def test_state_make_knee_active():
    test_state_mka = State()
    # Assert the knee is not active by default
    assert test_state_mka._is_knee_active == False
    # Assert the knee is active after calling the method
    test_state_mka.make_knee_active()
    assert test_state_mka._is_knee_active == True


# Test the State make_ankle_active method
def test_state_make_ankle_active():
    test_state_maa = State()
    # Assert the ankle is not active by default
    assert test_state_maa._is_ankle_active == False
    # Assert the ankle is active after calling the method
    test_state_maa.make_ankle_active()
    assert test_state_maa._is_ankle_active == True


# Test the State default properties
def test_state_default_properties():
    test_state = State()

    assert test_state.name == "state"
    assert test_state.knee_stiffness == 0.0
    assert test_state.knee_damping == 0.0
    assert test_state.knee_theta == 0.0
    assert test_state.ankle_stiffness == 0.0
    assert test_state.ankle_damping == 0.0
    assert test_state.ankle_theta == 0.0
    assert test_state.is_knee_active == False
    assert test_state.is_ankle_active == False
    assert test_state.minimum_time_spent_in_state == 2.0


# Test the Idle constructor
def test_idle_init():
    # Initialize the Idle object and assert the attributes are properly set
    test_idle = Idle()
    assert test_idle._name == "idle"
    assert test_idle._is_knee_active == False
    assert test_idle._knee_stiffness == 0.0
    assert test_idle._knee_damping == 0.0
    assert test_idle._knee_theta == 0.0
    assert test_idle._is_ankle_active == False
    assert test_idle._ankle_stiffness == 0.0
    assert test_idle._ankle_damping == 0.0
    assert test_idle._ankle_theta == 0.0
    assert test_idle._time_entered == 0.0
    assert test_idle._time_exited == 0.0
    assert test_idle._min_time_in_state == 2.0


# Test the Idle properties
def test_idle_properties():
    # Initialize the Idle object and assert the properties are properly set
    test_idle = Idle()
    assert test_idle.name == "idle"
    assert test_idle.knee_stiffness == 0.0
    assert test_idle.knee_damping == 0.0
    assert test_idle.knee_theta == 0.0
    assert test_idle.ankle_stiffness == 0.0
    assert test_idle.ankle_damping == 0.0
    assert test_idle.ankle_theta == 0.0
    assert test_idle.is_knee_active == False
    assert test_idle.is_ankle_active == False
    assert test_idle.minimum_time_spent_in_state == 2.0
    assert test_idle.status == "idle"


# Test the Event constructor
def test_event_init():
    # Initialize the Event object and assert the attributes are properly set
    test_event = Event(name="test_event")
    assert test_event._name == "test_event"
    test_event2 = Event(name="test_event2")
    assert test_event2._name == "test_event2"


# Test the Event __eq__ method
def test_event_eq():
    # Test the if statement
    test_event1 = Event(name="test_event")
    test_event2 = Event(name="test_event")
    assert test_event1 == test_event2
    # Test the else statement
    test_event1._name = "test"
    assert (test_event1 == test_event2) == False


# Test the Event __ne__ method
def test_event_ne():
    test_event3 = Event(name="test_event")
    test_event4 = Event(name="test_event")
    assert (test_event3 != test_event4) == False


# Create a test function to be used in the subsequent tests
def test_criteria() -> bool:
    return True


# Create another test function to be used in the subsequent tests
def test_action() -> None:
    print("data")


# Test the Transition constructor
def test_transition_init():
    # Initialize the Transition object and assert the attributes are properly set
    test_transition = Transition(
        event=Event(name="test_event"),
        source=State(name="state1"),
        destination=State(name="state2"),
    )
    assert test_transition._event == Event(name="test_event")
    assert test_transition._source_state == State(name="state1")
    assert test_transition._destination_state == State(name="state2")
    assert test_transition._criteria == None
    assert test_transition._action == None
    # Initialize another Transition object and assert the attributes are properly set
    test_transition2 = Transition(
        event=Event(name="test_event2"),
        source=State(name="state3"),
        destination=State(name="state4"),
        callback=test_criteria,
    )
    assert test_transition2._event == Event(name="test_event2")
    assert test_transition2._source_state == State(name="state3")
    assert test_transition2._destination_state == State(name="state4")
    assert test_transition2._criteria == test_criteria
    assert test_transition2._action == None


# Test the Transition __call__ method
def test_transition_call():
    transition_call = Transition(
        event=Event(name="test_event"),
        source=State(name="state1"),
        destination=State(name="state2"),
    )
    # Assert the call method works properly
    test_data = "test_data"
    try:
        transition_call(test_data)
    except NotImplementedError:
        pass
    else:
        assert False


# Test the add_criteria and add_action methods
def test_transition_add_criteria_and_add_action():
    test_transition_acaa = Transition(
        event=Event(name="test_event"),
        source=State(name="state1"),
        destination=State(name="state2"),
    )
    # Assert the add_criteria method works properly
    test_transition_acaa.add_criteria(callback=test_criteria)
    assert test_transition_acaa._criteria == test_criteria
    # Assert the add_action method works properly
    test_transition_acaa.add_action(callback=test_action)
    assert test_transition_acaa._action == test_action


# Test the Transition properties
def test_transition_properties():
    test_transition = Transition(
        event=Event(name="test_event"),
        source=State(name="state1"),
        destination=State(name="state2"),
    )
    assert test_transition.event == Event(name="test_event")
    assert test_transition.source_state == State(name="state1")
    assert test_transition.destination_state == State(name="state2")


# Test the FromToTransition constructor
def test_from_to_transition_init():
    # Initialize the FromToTransition object and assert the attributes are properly set
    test_from_to_transition = FromToTransition(
        event=Event(name="test_event"),
        source=State(name="state1"),
        destination=State(name="state2"),
    )
    assert test_from_to_transition._event == Event(name="test_event")
    assert test_from_to_transition._source_state == State(name="state1")
    assert test_from_to_transition._destination_state == State(name="state2")
    assert test_from_to_transition._criteria == None
    assert test_from_to_transition._action == None
    assert test_from_to_transition._from == State(name="state1")
    assert test_from_to_transition._to == State(name="state2")
    # Initialize another FromToTransition object and assert the attributes are properly set
    test_from_to_transition2 = FromToTransition(
        event=Event(name="test_event2"),
        source=State(name="state3"),
        destination=State(name="state4"),
        callback=test_criteria,
    )
    assert test_from_to_transition2._event == Event(name="test_event2")
    assert test_from_to_transition2._source_state == State(name="state3")
    assert test_from_to_transition2._destination_state == State(name="state4")
    assert test_from_to_transition2._criteria == test_criteria
    assert test_from_to_transition2._action == None
    assert test_from_to_transition2._from == State(name="state3")
    assert test_from_to_transition2._to == State(name="state4")


# Unfinished
# Test the FromToTransition __call__ method
def test_from_to_transition_call():
    from_to_transition_call = FromToTransition(
        event=Event(name="test_event"),
        source=State(name="state1"),
        destination=State(name="state2"),
    )
    # Assert the call method works properly
    from_to_transition_call(test_criteria)


# Test the StateMachine constructor
def test_state_machine_init():
    # Initialize the StateMachine object and assert the attributes are properly set
    test_state_machine = StateMachine()
    # assert test_state_machine._states == [State()]
    assert test_state_machine._events == []
    assert test_state_machine._transitions == []
    assert test_state_machine._current_state == None
    assert test_state_machine._initial_state == Idle()
    assert test_state_machine._current_state == None
    assert test_state_machine._exit_callback == None
    assert test_state_machine._exit_state == Idle()
    assert test_state_machine._initial_state == Idle()
    assert test_state_machine._exited == True
    assert test_state_machine._osl == None
    assert test_state_machine._spoof == False
