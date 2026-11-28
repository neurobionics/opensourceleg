import math
import time
from unittest.mock import MagicMock

import pytest

import opensourceleg.control.fsm as fsm_module
from opensourceleg.control.fsm import Event, State, StateMachine, Transition


@pytest.fixture(autouse=True)
def mock_fsm_logger(monkeypatch):
    mock_logger = MagicMock()
    monkeypatch.setattr(fsm_module, "LOGGER", mock_logger)
    return mock_logger


def _prevent_unstarted_del_error(sm: StateMachine) -> None:
    if sm._current_state is None:
        sm._current_state = State("_cleanup")


def test_state_init_equality_hash_and_repr():
    s1 = State(name="idle", minimum_time_in_state=1.5, custom_flag=True)
    s2 = State(name="idle")
    s3 = State(name="walking")

    assert s1.name == "idle"
    assert s1.minimum_time_spent_in_state == 1.5
    assert s1.custom_flag is True
    assert s1 == s2
    assert s1 != s3
    assert hash(s1) == hash(s2)
    assert repr(s1) == "State[idle]"
    assert len({s1, s2, s3}) == 2

    s1.set_minimum_time_spent_in_state(2.5)
    assert s1.minimum_time_spent_in_state == 2.5


def test_state_callbacks_and_timing(monkeypatch):
    t_val = [10.0]
    monkeypatch.setattr(time, "monotonic", lambda: t_val[0])

    entered = []
    exited = []
    state = State(
        name="stance",
        entry_callbacks=[lambda *a, **kw: entered.append((a, kw))],
        exit_callbacks=[lambda *a, **kw: exited.append((a, kw))],
    )
    state.add_entry_callback(lambda x, y=0: entered.append(("second", x, y)))
    state.add_exit_callback(lambda x, y=0: exited.append(("second", x, y)))

    state.enter(1, y=2)
    t_val[0] = 12.5
    assert math.isclose(state.current_time_in_state, 2.5)

    state.exit(3, y=4)
    assert math.isclose(state.time_spent_in_state, 2.5)
    assert entered == [((1,), {"y": 2}), ("second", 1, 2)]
    assert exited == [((3,), {"y": 4}), ("second", 3, 4)]


def test_event_equality_and_repr():
    e1 = Event("walk")
    e2 = Event("walk")
    e3 = Event("run")

    assert e1.name == "walk"
    assert e1 == e2
    assert e1 != e3
    assert repr(e1) == "Event[walk]"


def test_transition_success_and_kwargs_filtering():
    calls = []
    src = State("idle", exit_callbacks=[lambda **kw: calls.append(("exit", kw))])
    dst = State("walking", entry_callbacks=[lambda **kw: calls.append(("enter", kw))])
    event = Event("go")

    transition = Transition(
        event=event,
        source=src,
        destination=dst,
        criteria=lambda speed: speed > 1.0,
        action=lambda speed: calls.append(("action", speed)),
    )

    assert transition.event == event
    assert transition.source_state == src
    assert transition.destination_state == dst
    assert repr(transition) == "Transition[idle -> walking]"

    assert transition(speed=0.5, extra="ignored") == src
    assert calls == []

    assert transition(speed=2.0, extra="ignored") == dst
    assert calls == [
        ("action", 2.0),
        ("exit", {"speed": 2.0, "extra": "ignored"}),
        ("enter", {"speed": 2.0, "extra": "ignored"}),
    ]


def test_transition_without_criteria_and_mutators():
    src = State("a")
    dst = State("b")
    transition = Transition(event=Event("step"), source=src, destination=dst)

    assert transition() == dst

    transition.add_criteria(lambda flag: flag)
    transition.add_action(lambda: None)
    assert transition(flag=False) == src
    assert transition(flag=True) == dst


def test_transition_handles_criteria_and_action_exceptions(mock_fsm_logger):
    src = State("a")
    dst = State("b")

    def bad_criteria():
        raise RuntimeError("criteria failed")

    t_bad_criteria = Transition(event=Event("e1"), source=src, destination=dst, criteria=bad_criteria)
    assert t_bad_criteria() == src
    assert mock_fsm_logger.warning.call_count == 1

    def bad_action():
        raise RuntimeError("action failed")

    t_bad_action = Transition(event=Event("e2"), source=src, destination=dst, action=bad_action)
    assert t_bad_action() == dst
    assert mock_fsm_logger.warning.call_count == 2


def test_state_machine_setup_and_warnings(mock_fsm_logger):
    idle = State("idle")
    walking = State("walking")
    sm = StateMachine(states=[idle, walking], initial_state_name="idle")

    try:
        assert repr(sm) == "StateMachine"
        assert sm.states == ["idle", "walking"]
        assert sm.initial_state == idle
        assert sm.current_state is None
        assert sm.get_state_by_name("walking") == walking
        assert sm.get_state_by_name("missing") is None

        # Duplicate state, overwrite initial state, unknown initial state
        sm.add_states([State("idle"), State("running")], initial_state_name="walking")
        assert sm.initial_state == walking
        assert "running" in sm.states

        sm.add_states([], initial_state_name="nonexistent")
        assert sm.initial_state == walking

        # Duplicate events
        e1 = sm.create_event("go")
        assert e1.name == "go"
        sm.add_events([Event("go")])

        # Invalid transition returns None
        outside = State("outside")
        assert sm.add_transition(outside, walking, "bad") is None
        assert sm.add_transition(walking, outside, "bad") is None
        assert mock_fsm_logger.warning.call_count >= 3
    finally:
        _prevent_unstarted_del_error(sm)


def test_state_machine_start_stop_and_context_manager():
    events_log = []
    idle = State(
        "idle",
        entry_callbacks=[lambda *a, **kw: events_log.append(("enter", a, kw))],
        exit_callbacks=[lambda *a, **kw: events_log.append(("exit", a, kw))],
    )

    sm_empty = StateMachine()
    try:
        with pytest.raises(ValueError, match="No initial state set"):
            sm_empty.start()
        with pytest.raises(ValueError, match="State machine isn't active"):
            sm_empty.stop()
        with pytest.raises(ValueError, match="State machine isn't active"):
            sm_empty.update()
    finally:
        _prevent_unstarted_del_error(sm_empty)

    sm = StateMachine(states=[idle], initial_state_name="idle")
    sm.start("arg1", key="val")
    assert sm.current_state == idle
    sm.stop("arg2", key="val2")

    with sm as active_sm:
        assert active_sm is sm
        assert active_sm.current_state == idle

    assert events_log == [
        ("enter", ("arg1",), {"key": "val"}),
        ("exit", ("arg2",), {"key": "val2"}),
        ("enter", (), {}),
        ("exit", (), {}),
    ]


def test_state_machine_update_min_time_and_precedence(monkeypatch):
    t_val = [0.0]
    monkeypatch.setattr(time, "monotonic", lambda: t_val[0])

    sm = StateMachine()
    s_idle = sm.create_state("idle", minimum_time_in_state=1.0)
    s_walk = sm.create_state("walking")
    s_run = sm.create_state("running")
    sm.add_states([], initial_state_name="idle")

    # Add two transitions from idle; walking has precedence
    sm.add_transition(s_idle, s_walk, "to_walk", criteria=lambda go: go)
    sm.add_transition(s_idle, s_run, "to_run", criteria=lambda go: go)

    with sm:
        # Before minimum_time_in_state elapses, no transition occurs
        t_val[0] = 0.5
        sm.update(go=True)
        assert sm.current_state == s_idle

        # After minimum time elapses, criteria=False stays in idle
        t_val[0] = 1.5
        sm.update(go=False)
        assert sm.current_state == s_idle

        # First matching transition wins
        sm.update(go=True)
        assert sm.current_state == s_walk

        # State with no transitions stays put
        sm.update(go=True)
        assert sm.current_state == s_walk


def test_state_machine_iteration():
    sm_empty = StateMachine()
    try:
        assert list(sm_empty) == []
        with pytest.raises(StopIteration, match="No states in state machine"):
            next(sm_empty)
    finally:
        _prevent_unstarted_del_error(sm_empty)

    s1 = State("one")
    s2 = State("two")
    sm = StateMachine(states=[s1, s2], initial_state_name="one")
    try:
        assert list(sm) == [s1, s2]
        assert next(sm) == s1
    finally:
        _prevent_unstarted_del_error(sm)
