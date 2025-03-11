import time

import pytest

from opensourceleg.control.state_machine import Event, Idle, State, StateMachine, Transition


# Use a dummy OSL class to simulate the OpenSourceLeg object
class DummyOSL:
    class Logger:
        def debug(self, msg):
            pass

    def __init__(self):
        self.log = self.Logger()


# Mock time.time to control time flow in tests
@pytest.fixture
def mock_time(monkeypatch):
    mock_time = [0]

    def mock_time_func():
        return mock_time[0]

    monkeypatch.setattr(time, "time", mock_time_func)
    return mock_time


def test_state_creation():
    state = State(
        name="test_state",
        is_knee_active=True,
        knee_stiffness=10.0,
        knee_damping=1.0,
        knee_equilibrium_angle=0.5,
        is_ankle_active=True,
        ankle_stiffness=20.0,
        ankle_damping=2.0,
        ankle_equilibrium_angle=1.0,
        minimum_time_in_state=1.5,
    )

    assert state.name == "test_state"
    assert state.is_knee_active
    assert state.knee_stiffness == 10.0
    assert state.knee_damping == 1.0
    assert state.knee_theta == 0.5
    assert state.is_ankle_active
    assert state.ankle_stiffness == 20.0
    assert state.ankle_damping == 2.0
    assert state.ankle_theta == 1.0
    assert state.minimum_time_spent_in_state == 1.5


def test_knee_impedance_parameters():
    state = State()
    state.make_knee_active()
    state.set_knee_impedance_paramters(theta=0.3, k=15.0, b=1.5)
    assert state.is_knee_active
    assert state.knee_theta == 0.3
    assert state.knee_stiffness == 15.0
    assert state.knee_damping == 1.5


def test_ankle_impedance_parameters():
    state = State()
    state.make_ankle_active()
    state.set_ankle_impedance_paramters(theta=0.7, k=25.0, b=2.5)
    assert state.is_ankle_active
    assert state.ankle_theta == 0.7
    assert state.ankle_stiffness == 25.0
    assert state.ankle_damping == 2.5


def test_custom_data():
    state = State()
    state.set_custom_data("test_key", "test_value")
    state.set_custom_data("test_key_2", "test_value_2")
    assert state.get_custom_data("test_key") == "test_value"
    assert state.get_custom_data("test_key_2") == "test_value_2"
    assert list(state._custom_data.keys()) == ["test_key", "test_key_2"]


def test_callbacks():
    state = State()

    entry_called = []
    exit_called = []

    def entry_callback(data):
        entry_called.append(True)

    def exit_callback(data):
        exit_called.append(True)

    state.on_entry(entry_callback)
    state.on_exit(exit_callback)

    state.start(data=None)
    assert entry_called

    state.stop(data=None)
    assert exit_called


def test_time_spent_in_state(mock_time):
    state = State()
    state.start(data=None)
    mock_time[0] += 0.1  # Simulate 0.1 seconds passing
    state.stop(data=None)
    assert state.current_time_in_state == 0.1
    assert state.time_spent_in_state == 0.1


def test_idle_state():
    idle_state = Idle()
    assert idle_state.name == "idle"
    assert idle_state.status == "idle"


def test_event_creation():
    event = Event(name="test_event")
    assert event.name == "test_event"


def test_event_equality():
    event1 = Event(name="event")
    event2 = Event(name="event")
    event3 = Event(name="another_event")
    assert event1 == event2
    assert event1 != event3


def test_transition_creation():
    state1 = State(name="state1")
    state2 = State(name="state2")
    event = Event(name="event")
    transition = Transition(event=event, source=state1, destination=state2)
    assert transition.source_state == state1
    assert transition.destination_state == state2
    assert transition.event == event


def test_transition_call():
    state1 = State(name="state1")
    state2 = State(name="state2")
    event = Event(name="event")
    transition = Transition(event=event, source=state1, destination=state2)
    with pytest.raises(NotImplementedError):
        transition(data=None)


def test_from_to_transition():
    state1 = State(name="state1")
    state2 = State(name="state2")
    event = Event(name="event")
    data = {}

    # Define a simple criteria function that always returns True
    def criteria(data):
        return True

    # Define an action function
    action_called = []

    def action(data):
        action_called.append(True)

    transition = Transition(event=event, source=state1, destination=state2, callback=criteria)
    transition.add_action(action)
    state1.start(data)
    next_state = transition(data)
    assert next_state == state2
    assert action_called


def test_from_to_transition_no_criteria():
    state1 = State(name="state1")
    state2 = State(name="state2")
    event = Event(name="event")
    data = {}

    # Criteria function that returns False
    def criteria(data):
        return False

    transition = Transition(event=event, source=state1, destination=state2, callback=criteria)
    state1.start(data)
    next_state = transition(data)
    assert next_state == state1  # Should remain in state1


def test_state_machine_creation():
    sm = StateMachine()
    assert isinstance(sm, StateMachine)
    assert not sm.is_on()


def test_add_state():
    sm = StateMachine()
    state = State(name="state1")
    sm.add_state(state, initial_state=True)
    assert "state1" in sm.states
    sm.start()  # Start the state machine to set the current state
    assert sm.current_state == state


def test_add_event():
    sm = StateMachine()
    event = Event(name="event1")
    sm.add_event(event)
    assert event in sm._events


def test_add_transition():
    sm = StateMachine()
    state1 = State(name="state1")
    state2 = State(name="state2")
    event = Event(name="event1")
    sm.add_state(state1, initial_state=True)
    sm.add_state(state2)
    sm.add_event(event)

    transition = sm.add_transition(source=state1, destination=state2, event=event)
    assert transition is not None
    assert transition in sm._transitions


def test_state_machine_start_stop():
    sm = StateMachine()
    state = State(name="state1")
    sm.add_state(state, initial_state=True)
    sm.start()
    assert sm.is_on()
    sm.stop()
    assert not sm.is_on()


def test_state_machine_update():
    sm = StateMachine()
    state1 = State(name="state1")
    state2 = State(name="state2")
    event = Event(name="event1")
    sm.add_state(state1, initial_state=True)
    sm.add_state(state2)
    sm.add_event(event)

    # Criteria that always returns True
    def criteria(data):
        return True

    sm.add_transition(source=state1, destination=state2, event=event, callback=criteria)
    sm.start()
    sm._osl = DummyOSL()
    sm.update()
    assert sm.current_state == state2


def test_state_machine_no_valid_transition():
    sm = StateMachine()
    state1 = State(name="state1")
    state2 = State(name="state2")
    event = Event(name="event1")
    sm.add_state(state1, initial_state=True)
    sm.add_state(state2)
    sm.add_event(event)

    # Criteria that always returns False
    def criteria(data):
        return False

    sm.add_transition(source=state1, destination=state2, event=event, callback=criteria)
    sm.start()
    sm._osl = DummyOSL()
    sm.update()
    # Should remain in state1
    assert sm.current_state == state1


def test_state_machine_spoof(mock_time):
    sm = StateMachine(spoof=True)
    # Set minimum_time_in_state to match the simulated time
    state1 = State(name="state1", minimum_time_in_state=0.1)
    state2 = State(name="state2")
    event = Event(name="event1")
    sm.add_state(state1, initial_state=True)
    sm.add_state(state2)
    sm.add_event(event)

    sm.add_transition(source=state1, destination=state2, event=event)
    sm.start()
    sm._osl = DummyOSL()
    # Simulate time passing
    mock_time[0] += 0.2
    sm.update()
    assert sm.current_state == state2
