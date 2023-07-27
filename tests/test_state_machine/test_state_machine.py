from typing import Callable

import time
from dataclasses import field

import pytest

from opensourceleg.logger import Logger
from opensourceleg.osl import OpenSourceLeg
from opensourceleg.state_machine import (
    Event,
    FromToTransition,
    Idle,
    State,
    StateMachine,
    Transition,
)


def test_state_init():

    """
    Tests the State constructor\n
    Initializes a default state object and asserts the attributes are properly set.
    """

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


def test_state_eq():

    """
    Tests the State __eq__ method\n
    Initializes two default state objects and asserts the if and else statements correctly
    compare the name attributes.
    """

    # Test the if statement
    test_state1 = State()
    test_state2 = State()
    assert test_state1 == test_state2
    # Test the else statement
    test_state1._name = "test"
    assert (test_state1 == test_state2) == False


def test_state_ne():

    """
    Tests the State __ne__ method\n
    Initializes two default state objects and asserts the if and else statements correctly
    compare the name attributes.
    """

    test_state3 = State()
    test_state4 = State()
    assert (test_state3 != test_state4) == False
    test_state3._name = "test"
    assert test_state3 != test_state4


def test_state_call():

    """
    Tests the State __call__ method\n
    Initializes a default state object and asserts the call method correctly
    returns None.
    """

    test_state_call = State()
    # Assert the call method works properly
    test_data = "test_data"
    result = test_state_call(test_data)
    assert result == None


def test_state_set_minimum_time_spent_in_state():

    """
    Tests the State set_minimum_time_spent_in_state method\n
    Initializes a default state object and asserts the default value for
    _min_time_in_state is 2.0. Then, the method is called and asserts the
    value is set properly.
    """

    test_state_smtsis = State()
    # Assert the default value is 2.0
    assert test_state_smtsis._min_time_in_state == 2.0
    # Assert the value is set properly
    test_state_smtsis.set_minimum_time_spent_in_state(1.0)
    assert test_state_smtsis._min_time_in_state == 1.0


def test_state_set_knee_impedance_parameters():

    """
    Tests the State set_knee_impedance_paramters method\n
    Initializes a default state object and asserts the default values for
    _knee_theta, _knee_stiffness, and _knee_damping are 0.0. Then, the method
    is called and asserts the values are set properly.
    """

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


def test_state_set_ankle_impedance_parameters():

    """
    Tests the State set_ankle_impedance_paramters method\n
    Initializes a default state object and asserts the default values for
    _ankle_theta, _ankle_stiffness, and _ankle_damping are 0.0. Then, the method
    is called and asserts the values are set properly.
    """

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


def test_state_set_and_get_custom_data():

    """
    Tests the State set_custom_data and get_custom_data methods\n
    Initializes a default state object and asserts the default value for
    _custom_data is an empty dictionary. Then, the set_custom_data method is
    called and asserts the value is set properly. Then, the get_custom_data
    method is called and asserts the value is retrieved properly. This is then
    repeated to assert the overwrite works properly.
    """

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


def callback1(data) -> bool:

    """
    Callback function to be used in the on_entry test and the on_exit test
    """

    print("Called", data)
    return True


def test_callback1():

    """
    Tests the callback1 function\n
    Asserts the function returns True.
    """

    assert callback1("test") == True


def callback2(data) -> bool:

    """
    Callback function to be used in the on_entry test and the on_exit test
    """

    print("Called", data)
    return True


def test_callback2():

    """
    Tests the callback2 function\n
    Asserts the function returns True.
    """

    assert callback2("test") == True


def test_state_on_entry():

    """
    Tests the State on_entry method\n
    Initializes a default state object. Then, the on_entry method is called and
    asserts the callback is added to the _entry_callbacks list. This is then
    repeated to assert the multiple callbacks are added to the list.
    """
    test_state_soe = State()

    # Assert the on_entry method works properly
    test_state_soe.on_entry(callback1)
    assert callback1 in test_state_soe._entry_callbacks
    # Create another callback function to be used in the on_entry method

    # Assert the on_entry method works properly with multiple callbacks
    test_state_soe.on_entry(callback2)
    assert callback1 in test_state_soe._entry_callbacks
    assert callback2 in test_state_soe._entry_callbacks


def test_state_on_exit():

    """
    Tests the State on_exit method\n
    Initializes a default state object. Then, the on_exit method is called and
    asserts the callback is added to the _exit_callbacks list. This is then
    repeated to assert the multiple callbacks are added to the list.
    """

    test_state_soe = State()

    # Assert the on_exit method works properly
    test_state_soe.on_exit(callback1)
    assert callback1 in test_state_soe._exit_callbacks

    # Assert the on_exit method works properly with multiple callbacks
    test_state_soe.on_exit(callback2)
    assert callback1 in test_state_soe._exit_callbacks
    assert callback2 in test_state_soe._exit_callbacks


@pytest.fixture
def mock_time(monkeypatch):

    """
    Fixture to mock the time.time method to return 1.0
    """

    monkeypatch.setattr(time, "time", lambda: 1.0)


def criteria_test(data="test_data") -> bool:

    """
    Test function that initializes a logger of the lowest stream level
    and asserts the callback is called.
    """

    callback_log = Logger(file_path="tests/test_state_machine/{}".format(data))
    callback_log.set_stream_level(level="DEBUG")
    callback_log.debug("Criteria was called")
    return True


def criteria_test2(data="log_name") -> bool:

    """
    Test function that initializes a logger of the lowest stream level
    and asserts the callback is called.
    """

    callback_log = Logger(file_path="tests/test_state_machine/{}".format(data))
    callback_log.set_stream_level(level="DEBUG")
    callback_log.debug("Criteria2 was called")
    return False


def test_action(data="log_name") -> bool:

    """
    Test function that initializes a logger of the lowest stream level
    and asserts the callback is called.
    """

    callback_log = Logger(file_path="tests/test_state_machine/{}".format(data))
    callback_log.set_stream_level(level="DEBUG")
    callback_log.debug("Action was called")
    return True


def test_state_start(mock_time):

    """
    Tests the State start method\n
    Initializes a default state object with a callback added to the _entry_callbacks
    list. Then, the start method is called and asserts the time_entered is set
    properly, and the callback is called.
    """

    test_state_start = State()
    # Add the criteria_test using the on_entry method
    test_state_start._entry_callbacks.append(criteria_test)
    # Assert the time_entered is set properly
    test_state_start.start(data="test_state_start")
    assert test_state_start._time_entered == 1.0
    assert criteria_test in test_state_start._entry_callbacks
    with open("tests/test_state_machine/test_state_start.log", "r") as f:
        contents = f.read()
        assert "DEBUG: Criteria was called" in contents


def test_state_stop(mock_time):

    """
    Tests the State stop method\n
    Initializes a default state object with a callback added to the _exit_callbacks
    list. Then, the stop method is called and asserts the time_exited is set
    properly, and the callback is called.
    """

    test_state_stop = State()
    # Add the criteria_test using the on_exit method
    test_state_stop._exit_callbacks.append(criteria_test2)
    # Assert the time_exited is set properly
    test_state_stop.stop(data="test_state_stop")
    assert test_state_stop._time_exited == 1.0
    assert criteria_test2 in test_state_stop._exit_callbacks
    with open("tests/test_state_machine/test_state_stop.log", "r") as f:
        contents = f.read()
        assert "DEBUG: Criteria2 was called" in contents


def test_state_make_knee_active():

    """
    Tests the State make_knee_active method\n
    Initializes a default state object and asserts the knee is not active by
    default. Then, the method is called and asserts the knee is active.
    """

    test_state_mka = State()
    # Assert the knee is not active by default
    assert test_state_mka._is_knee_active == False
    # Assert the knee is active after calling the method
    test_state_mka.make_knee_active()
    assert test_state_mka._is_knee_active == True


def test_state_make_ankle_active():

    """
    Tests the State make_ankle_active method\n
    Initializes a default state object and asserts the ankle is not active by
    default. Then, the method is called and asserts the ankle is active.
    """

    test_state_maa = State()
    # Assert the ankle is not active by default
    assert test_state_maa._is_ankle_active == False
    # Assert the ankle is active after calling the method
    test_state_maa.make_ankle_active()
    assert test_state_maa._is_ankle_active == True


def test_state_default_properties(mock_time):

    """
    Tests the State properties\n
    Initializes a default state object and asserts the properties are properly set.
    """

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
    assert test_state.current_time_in_state == 1.0
    assert test_state.time_spent_in_state == 0.0


def test_idle_init():

    """
    Tests the Idle constructor\n
    Initializes a default idle object and asserts the attributes are properly set.
    """

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


def test_idle_properties():

    """
    Tests the Idle properties\n
    Initializes a default idle object and asserts the properties are properly set.
    """

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


def test_event_init():

    """
    Tests the Event constructor\n
    Initializes a default event object and asserts the attributes are properly set.
    """

    test_event = Event(name="test_event")
    assert test_event._name == "test_event"
    test_event2 = Event(name="test_event2")
    assert test_event2._name == "test_event2"


def test_event_eq():

    """
    Tests the Event __eq__ method\n
    Initializes two default event objects and asserts the if and else statements correctly
    compare the name attributes.
    """

    # Test the if statement
    test_event1 = Event(name="test_event")
    test_event2 = Event(name="test_event")
    assert test_event1 == test_event2
    # Test the else statement
    test_event1._name = "test"
    assert (test_event1 == test_event2) == False


def test_event_ne():

    """
    Tests the Event __ne__ method\n
    Initializes two default event objects and asserts the if and else statements correctly
    compare the name attributes.
    """

    test_event3 = Event(name="test_event")
    test_event4 = Event(name="test_event")
    assert (test_event3 != test_event4) == False


def test_transition_init():

    """
    Tests the Transition constructor\n
    Initializes a transition object and asserts the attributes are properly set. Then,
    another transition object is initialized with a callback and asserts the attributes
    are properly set.
    """

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
        callback=criteria_test,
    )
    assert test_transition2._event == Event(name="test_event2")
    assert test_transition2._source_state == State(name="state3")
    assert test_transition2._destination_state == State(name="state4")
    assert test_transition2._criteria == criteria_test
    assert test_transition2._action == None


def test_transition_call():

    """
    Tests the Transition __call__ method\n
    Initializes a transition object and asserts the call method correctly
    returns None.
    """

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


def test_transition_add_criteria_and_add_action():

    """
    Tests the Transition add_criteria and add_action methods\n
    Initializes a transition object and calls the add_criteria and add_action methods.
    Then, asserts the attributes are properly set.
    """

    test_transition_acaa = Transition(
        event=Event(name="test_event"),
        source=State(name="state1"),
        destination=State(name="state2"),
    )
    # Assert the add_criteria method works properly
    test_transition_acaa.add_criteria(callback=criteria_test)
    assert test_transition_acaa._criteria == criteria_test
    # Assert the add_action method works properly
    test_transition_acaa.add_action(callback=test_action)
    assert test_transition_acaa._action == test_action


def test_transition_properties():

    """
    Tests the Transition properties\n
    Initializes a transition object and asserts the properties are properly set.
    """

    test_transition = Transition(
        event=Event(name="test_event"),
        source=State(name="state1"),
        destination=State(name="state2"),
    )
    assert test_transition.event == Event(name="test_event")
    assert test_transition.source_state == State(name="state1")
    assert test_transition.destination_state == State(name="state2")


def test_from_to_transition_init():

    """
    Tests the FromToTransition constructor\n
    Initializes a FromToTransition object and asserts the attributes are properly set.
    Then, another FromToTransition object is initialized with a callback and asserts the
    attributes are properly set.
    """

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
        callback=criteria_test,
    )
    assert test_from_to_transition2._event == Event(name="test_event2")
    assert test_from_to_transition2._source_state == State(name="state3")
    assert test_from_to_transition2._destination_state == State(name="state4")
    assert test_from_to_transition2._criteria == criteria_test
    assert test_from_to_transition2._action == None
    assert test_from_to_transition2._from == State(name="state3")
    assert test_from_to_transition2._to == State(name="state4")


# Unfinished
# Test the FromToTransition __call__ method
def test_from_to_transition_call(mock_time):
    # Assert the call method works properly with the if statement inside the if statement
    state1 = State(name="state1")
    state1._time_entered = -100.0
    from_to_transition_call = FromToTransition(
        event=Event(name="test_event"),
        source=state1,
        destination=State(name="state2"),
    )
    from_to_transition_call.add_action(callback=test_action)
    test = from_to_transition_call(data="test_from_to_transition_call", spoof=True)
    with open("tests/test_state_machine/test_from_to_transition_call.log", "r") as f:
        contents = f.read()
        assert "DEBUG: Action was called" in contents
    assert test == State(name="state2")
    # Assert the call method works properly with the else statement inside the if statement
    state2 = State(name="state2")
    state2._time_entered = 100.0
    from_to_transition_call1 = FromToTransition(
        event=Event(name="test_event"),
        source=state2,
        destination=State(name="state3"),
    )
    from_to_transition_call1.add_action(callback=test_action)
    test2 = from_to_transition_call1(data="test_from_to_transition_call1", spoof=True)
    # with open("tests/test_state_machine/test_from_to_transition_call1.log", "r") as f:
    #     contents = f.read()
    #     assert "DEBUG: Action was called" in contents
    assert test2 == State(name="state2")

    # Assert the call method works properly with the elif statement
    from_to_transition_call2 = FromToTransition(
        event=Event(name="test_event"),
        source=State(name="state1"),
        destination=State(name="state2"),
    )
    from_to_transition_call2.add_action(callback=test_action)
    test1 = from_to_transition_call2(data="test_from_to_transition_call2")
    with open("tests/test_state_machine/test_from_to_transition_call2.log", "r") as f:
        contents = f.read()
        assert "DEBUG: Action was called" in contents
    assert test1 == State(name="state2")
    # Assert the call method works properly with the else statement
    from_to_transition_call3 = FromToTransition(
        event=Event(name="test_event"),
        source=State(name="state1"),
        destination=State(name="state2"),
    )
    from_to_transition_call3.add_criteria(callback=criteria_test2)
    test2 = from_to_transition_call3(data="test_from_to_transition_call3")
    with open("tests/test_state_machine/test_from_to_transition_call3.log", "r") as f:
        contents = f.read()
        assert "DEBUG: Criteria2 was called" in contents
    assert test2 == State(name="state1")


def test_state_machine_init():

    """
    Tests the StateMachine constructor\n
    Initializes a StateMachine object and asserts the attributes are properly set.
    """

    # Initialize the StateMachine object and assert the attributes are properly set
    test_state_machine = StateMachine()
    assert test_state_machine._states == [Idle()]
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


def test_state_machine_add_state():

    """
    Tests the StateMachine add_state method\n
    Initializes a StateMachine object and adds a state and asserts it was
    added properly. Then, another state is added and asserts it was added
    properly. Then, a duplicate state is added and asserts the value error
    is raised. Then, another state is added as the initial state and asserts
    it was added properly and the initial state was set properly.
    """

    test_state_machine_asm = StateMachine()
    # Assert the add_state method works properly
    test_state_machine_asm.add_state(state=State(name="test_state"))
    assert test_state_machine_asm._states == [Idle(), State(name="test_state")]
    # Assert the add_state method works properly with multiple states
    test_state_machine_asm.add_state(state=State(name="test_state2"))
    assert test_state_machine_asm._states == [
        Idle(),
        State(name="test_state"),
        State(name="test_state2"),
    ]
    # Assert the add_state method works properly with a duplicate state
    try:
        test_state_machine_asm.add_state(state=State(name="test_state"))
    except ValueError:
        pass
    # else:
    #     assert False
    # Assert the initial state is set properly
    test_state_machine_asm.add_state(
        state=State(name="test_state3"), initial_state=True
    )
    assert test_state_machine_asm._states == [
        Idle(),
        State(name="test_state"),
        State(name="test_state2"),
        State(name="test_state3"),
    ]
    assert test_state_machine_asm._initial_state == State(name="test_state3")


def test_state_machine_add_event():

    """
    Tests the StateMachine add_event method\n
    Initializes a StateMachine object and adds an event and asserts it was
    added properly. Then, another event is added and asserts it was added
    properly.
    """

    test_state_machine_ame = StateMachine()
    # Assert the add_event method works properly
    test_state_machine_ame.add_event(event=Event(name="test_event"))
    assert test_state_machine_ame._events == [Event(name="test_event")]
    # Assert the add_event method works properly with multiple events
    test_state_machine_ame.add_event(event=Event(name="test_event2"))
    assert test_state_machine_ame._events == [
        Event(name="test_event"),
        Event(name="test_event2"),
    ]


def test_state_machine_add_transition():

    """
    Tests the StateMachine add_transition method\n
    Initializes a StateMachine object and adds 3 states and 3 events. Then,
    a transition is added and asserts it was added properly. Then, another
    transition is added and asserts it was added properly.
    """

    test_state_machine_amt = StateMachine()
    # Assert the add_transition method works properly
    test_state_machine_amt.add_state(state=State(name="state1"))
    test_state_machine_amt.add_state(state=State(name="state2"))
    test_state_machine_amt.add_state(state=State(name="state3"))
    test_state_machine_amt.add_event(event=Event(name="test_event1"))
    test_state_machine_amt.add_event(event=Event(name="test_event2"))
    test_state_machine_amt.add_event(event=Event(name="test_event3"))
    transition1 = test_state_machine_amt.add_transition(
        source=State(name="state1"),
        destination=State(name="state2"),
        event=Event(name="test_event1"),
    )
    assert test_state_machine_amt._transitions == [transition1]
    # Assert the add_transition method works properly with an invalid transition
    transition2 = test_state_machine_amt.add_transition(
        source=State(name="state2"),
        destination=State(name="state4"),
        event=Event(name="test_event2"),
    )
    assert test_state_machine_amt._transitions == [transition1]


def exit_callback_test(idle=Idle(), data="test_state_machine_update2"):

    """
    Test callback function that initializes a logger of the lowest stream level
    and asserts the callback is called.
    """

    callback_log = Logger(file_path="tests/test_state_machine/{}".format(data))
    callback_log.set_stream_level(level="DEBUG")
    callback_log.debug("Exit callback was called")
    return True


def test_state_machine_update():

    """
    Tests the StateMachine update method\n
    Initializes a StateMachine object and intializes the initial state and
    current state to none. Then, the update method is called and asserts the
    value error is raised. Then, the initial state is set to an Idle object
    and the current state is set to a State object. The _osl attribute is
    set to an OpenSourceLeg object and a logger of the lowest stream level
    is initialized for the _osl. Then, the update method is called and
    asserts the proper log message is written when the event is not valid.
    Another StateMachine object is initialized with 2 states, 1 event, and
    1 transition, and the _osl attribute is set to a OpenSourceLeg object
    The update method is called and asserts the current state was updated properly.
    Another transition is added and the initial and current states are set to
    State objects. The _exit_callback attribute is set to a callback function
    and the update method is called and asserts the current state was updated
    properly and the exit callback was called.
    """

    # Assert the value error is raised when the initial state is not set
    test_state_machine_update = StateMachine()
    test_state_machine_update._initial_state = None
    test_state_machine_update._current_state = None
    try:
        test_state_machine_update.update(data="test_data")
    except ValueError:
        pass
    # else:
    #     assert False
    # Assert the proper log message is written when validity is false
    test_state_machine_update._initial_state = Idle()
    test_state_machine_update._current_state = State(name="state1")
    test_state_machine_update._osl = OpenSourceLeg()
    test_state_machine_update._osl.log = Logger(
        file_path="tests/test_state_machine/test_state_machine_update"
    )
    test_state_machine_update._osl.log.set_stream_level(level="DEBUG")
    test_state_machine_update.update(data="test_data")
    with open("tests/test_state_machine/test_state_machine_update.log", "r") as f:
        contents = f.read()
        assert "DEBUG: Event isn't valid at state1" in contents
    # Assert the for loop works properly
    test_state_machine_update2 = StateMachine()
    test_state_machine_update2._exited = False
    test_state_machine_update2.add_state(state=State(name="state1"))
    test_state_machine_update2.add_state(state=State(name="state2"))
    test_state_machine_update2.add_event(event=Event(name="test_event1"))
    transition1 = test_state_machine_update2.add_transition(
        source=State(name="state1"),
        destination=State(name="state2"),
        event=Event(name="test_event1"),
    )
    assert test_state_machine_update2._transitions == [transition1]
    test_state_machine_update2._initial_state = State(name="state3")
    test_state_machine_update2._current_state = State(name="state1")
    test_state_machine_update2._osl = OpenSourceLeg()
    test_state_machine_update2.update()
    assert test_state_machine_update2._current_state == State(name="state2")
    # Assert the if isintance statement works properly
    transition2 = test_state_machine_update2.add_transition(
        source=State(name="state2"),
        destination=Idle(),
        event=Event(name="test_event1"),
    )
    assert test_state_machine_update2._transitions == [transition1, transition2]
    test_state_machine_update2._initial_state = State(name="state3")
    test_state_machine_update2._current_state = State(name="state2")
    test_state_machine_update2._exit_callback = exit_callback_test
    test_state_machine_update2.update()
    assert isinstance(test_state_machine_update2._current_state, Idle)
    assert test_state_machine_update2._exited == True
    with open("tests/test_state_machine/test_state_machine_update2.log", "r") as f:
        contents = f.read()
        assert "DEBUG: Exit callback was called" in contents


def test_state_machine_start():

    """
    Tests the StateMachine start method\n
    Initializes a StateMachine object with no initial state, calls the start
    method, and asserts the value error is raised. Then, the initial state is
    set to a State object and the start method is called and asserts the
    current state and _exited attributes were set properly.
    """

    test_state_machine_sta = StateMachine()
    # Assert the value error is raised when the initial state is not set
    test_state_machine_sta._initial_state = None
    try:
        test_state_machine_sta.start(data="test_data")
    except ValueError:
        pass
    # else:
    #     assert False
    # Assert the start method works properly
    test_state_machine_sta._initial_state = State(name="state1")
    test_state_machine_sta._current_state = None

    test_state_machine_sta.start(data="test_state_machine_start")
    assert test_state_machine_sta._current_state == State(name="state1")
    assert test_state_machine_sta._exited == False


def test_state_machine_stop():

    """
    Tests the StateMachine stop method\n
    Initializes a StateMachine object with no initial state, calls the stop
    method, and asserts the value error is raised. Then, the initial state
    and current state are set to State objects and the stop method is called
    and asserts the current state and _exited attributes were set properly.
    """

    test_state_machine_sto = StateMachine()
    # Assert the value error is raised when the initial state is not set
    test_state_machine_sto._initial_state = None
    try:
        test_state_machine_sto.stop(data="test_data")
    except ValueError:
        pass
    # else:
    #     assert False
    # Assert the stop method works properly
    test_state_machine_sto._initial_state = State(name="state2")
    test_state_machine_sto._current_state = State(name="state1")
    test_state_machine_sto.stop(data="test_state_machine_stop")
    assert test_state_machine_sto._current_state == State(name="idle")
    assert test_state_machine_sto._exited == True


def test_state_machine_is_on():

    """
    Tests the StateMachine is_on method\n
    Initializes a StateMachine object and sets the _current_state attribute
    to a State object and asserts the method returns True. Then, the
    _current_state attribute is set to an Idle object and asserts the method
    returns False.
    """

    test_state_machine_io = StateMachine()
    # Assert the is_on method works properly
    assert test_state_machine_io._exit_state == Idle()
    test_state_machine_io._current_state = State(name="state1")
    assert test_state_machine_io.is_on() == True
    # Assert the is_on method works properly
    assert test_state_machine_io._exit_state == Idle()
    test_state_machine_io._current_state = Idle()
    assert test_state_machine_io.is_on() == False


def test_state_machine_spoof():

    """
    Tests the StateMachine spoof method\n
    Initializes a StateMachine object and asserts the spoof attribute is
    False. Then, the spoof method is called with spoof=True and asserts the
    spoof attribute is True. Then, the spoof method is called with spoof=False
    and asserts the spoof attribute is False.
    """

    test_state_machine_spoof = StateMachine()
    # Assert the spoof method works properly
    assert test_state_machine_spoof._spoof == False
    test_state_machine_spoof.spoof(spoof=True)
    assert test_state_machine_spoof._spoof == True
    test_state_machine_spoof.spoof(spoof=False)
    assert test_state_machine_spoof._spoof == False


def test_state_machine_default_properties():

    """
    Tests the StateMachine default properties\n
    Initializes a default StateMachine object and asserts the properties are properly set.
    """

    test_state_machine_dp = StateMachine()
    assert test_state_machine_dp.current_state == Idle()
    assert test_state_machine_dp.states == ["idle"]
    assert test_state_machine_dp.current_state_name == "idle"
    assert test_state_machine_dp.is_spoofing == False


def test_state_machine_non_default_properties():

    """
    Tests the StateMachine non-default properties\n
    Initializes a StateMachine object and adds 3 states, sets the
    initial and current states to State objects, and sets the spoof
    attribute to True. Then, asserts the properties are properly set.
    """

    test_state_machine_ndp = StateMachine()
    test_state_machine_ndp.add_state(state=State(name="state1"))
    test_state_machine_ndp.add_state(state=State(name="state2"))
    test_state_machine_ndp.add_state(state=State(name="state3"))
    test_state_machine_ndp._initial_state = State(name="state1")
    test_state_machine_ndp._current_state = State(name="state2")
    test_state_machine_ndp._spoof = True
    assert test_state_machine_ndp.current_state == State(name="state2")
    assert test_state_machine_ndp.states == ["idle", "state1", "state2", "state3"]
    assert test_state_machine_ndp.current_state_name == "state2"
    assert test_state_machine_ndp.is_spoofing == True
