#!/usr/bin/python3
# A simple and scalable Finite State Machine module

import time
from collections.abc import Iterator
from typing import Any, Callable, Optional

from opensourceleg.logging.logger import LOGGER

"""
The state_machine module provides classes for implementing a finite state machine (FSM).
It includes the State, Event, Transition, and StateMachine classes.

Usage:
1. Use the `State` class to represent a state in the FSM.
2. Utilize the `Event` class to define events that trigger state transitions.
3. Create transitions between states using the `Transition` class.
   Add criteria and actions as needed.
4. Instantiate the `StateMachine` class, add states, events, and transitions, and start the FSM.
"""


class State:
    """
    A class to represent a state in a finite state machine.

    Args:
        name (str): Name of the state
        minimum_time_in_state (float): Minimum time spent in the state in seconds. Default: 2.0
        entry_callbacks (list[Callable]): Functions to call when entering the state
        exit_callbacks (list[Callable]): Functions to call when exiting the state
        **kwargs: Additional attributes to set on the state
    """

    def __init__(
        self,
        name: str = "state",
        minimum_time_in_state: float = 2.0,
        entry_callbacks: Optional[list[Callable[[Any], None]]] = None,
        exit_callbacks: Optional[list[Callable[[Any], None]]] = None,
        **kwargs: Any,
    ) -> None:
        self._name: str = name
        self._time_entered: float = 0.0
        self._time_exited: float = 0.0
        self._min_time_in_state: float = minimum_time_in_state
        self._entry_callbacks: list[Callable[[Any], None]] = entry_callbacks or []
        self._exit_callbacks: list[Callable[[Any], None]] = exit_callbacks or []

        # Set additional attributes
        for key, value in kwargs.items():
            setattr(self, key, value)

    def __eq__(self, __o: Any) -> bool:
        return bool(__o.name == self._name)

    def __ne__(self, __o: Any) -> bool:
        return not self.__eq__(__o)

    def __hash__(self) -> int:
        return hash(self._name)

    def __call__(self, data: Any) -> Any:
        pass

    def __repr__(self) -> str:
        return f"State[{self._name}]"

    def set_minimum_time_spent_in_state(self, time: float) -> None:
        """
        Set the minimum time spent in the state

        Args:
            time (float): Minimum time spent in the state in seconds
        """
        self._min_time_in_state = time

    def on_entry(self, callback: Callable[[Any], None]) -> None:
        self._entry_callbacks.append(callback)

    def on_exit(self, callback: Callable[[Any], None]) -> None:
        self._exit_callbacks.append(callback)

    def start(self, *args: Any, **kwargs: Any) -> None:
        self._time_entered = time.monotonic()
        for c in self._entry_callbacks:
            c(*args, **kwargs)

    def stop(self, *args: Any, **kwargs: Any) -> None:
        self._time_exited = time.monotonic()
        for c in self._exit_callbacks:
            c(*args, **kwargs)

    @property
    def name(self) -> str:
        return self._name

    @property
    def minimum_time_spent_in_state(self) -> float:
        return self._min_time_in_state

    @property
    def current_time_in_state(self) -> float:
        return time.monotonic() - self._time_entered

    @property
    def time_spent_in_state(self) -> float:
        return self._time_exited - self._time_entered


class Event:
    """
    Event class
    """

    def __init__(self, name: str) -> None:
        """
        Args:
            name (str): The name of the event.
        """
        self._name: str = name

    def __eq__(self, __o: Any) -> bool:
        return bool(__o.name == self._name)

    def __ne__(self, __o: Any) -> bool:
        return not self.__eq__(__o)  # TODO: Check this fix

    def __repr__(self) -> str:
        return f"Event[{self._name}]"

    @property
    def name(self) -> str:
        return self._name


class Transition:
    """
    Transition class that handles state transitions in a finite state machine.
    A transition connects a source state to a destination state and is triggered by an event.
    It can include criteria that must be met for the transition to occur and actions to execute
    during the transition.

    Args:
        event (Event): The event that triggers this transition
        source (State): The source state
        destination (State): The destination state
        criteria (Callable): Optional function that returns True if transition should occur
        action (Callable): Optional function to execute during transition
    """

    def __init__(
        self,
        event: Event,
        source: State,
        destination: State,
        criteria: Optional[Callable[[Any], bool]] = None,
        action: Optional[Callable[[Any], None]] = None,
    ) -> None:
        self._event: Event = event
        self._source_state: State = source
        self._destination_state: State = destination
        self._from: State = source  # For backward compatibility
        self._to: State = destination  # For backward compatibility

        self._criteria: Optional[Callable[[Any], bool]] = criteria
        self._action: Optional[Callable[[Any], None]] = action

    def __call__(self, *args: Any, **kwargs: Any) -> State:
        if kwargs.get("spoof", False):
            if self._source_state.current_time_in_state > self._source_state.minimum_time_spent_in_state:
                if self._action:
                    self._action(*args, **kwargs)

                self._source_state.stop(*args, **kwargs)
                self._destination_state.start(*args, **kwargs)

                return self._destination_state

            else:
                return self._source_state

        elif not self._criteria or self._criteria(*args, **kwargs):
            if self._action:
                self._action(*args, **kwargs)

            self._source_state.stop(*args, **kwargs)
            self._destination_state.start(*args, **kwargs)

            return self._destination_state

        else:
            return self._source_state

    def __repr__(self) -> str:
        return f"Transition[{self._source_state.name} -> {self._destination_state.name}]"

    def add_criteria(self, callback: Callable[[Any], bool]) -> None:
        self._criteria = callback

    def add_action(self, callback: Callable[[Any], Any]) -> None:
        self._action = callback

    @property
    def event(self) -> Event:
        return self._event

    @property
    def source_state(self) -> State:
        return self._source_state

    @property
    def destination_state(self) -> State:
        return self._destination_state


class StateMachine:
    """
    State Machine class

    A flexible finite state machine implementation that supports:
    - Multiple states with transitions between them
    - Event-driven transitions with optional criteria
    - Entry and exit actions for states
    - Minimum time constraints for states
    - Testing mode with spoofed transitions

    Parameters
    ----------
    spoof : bool
        If True, the state machine will spoof the state transitions--ie, it will not
        check the criteria for transitioning but will instead transition after the
        minimum time spent in state has elapsed. This is useful for testing.
        Defaults to False.

    Attributes
    ----------
    current_state : State
        The current state of the state machine.
    states : list[str]
        The list of state names in the state machine.
    is_spoofing : bool
        Whether or not the state machine is spoofing the state transitions.
    """

    def __init__(self, spoof: bool = False, **kwargs: Any) -> None:
        # State Machine Variables
        self._states: list[State] = []
        self._events: list[Event] = []
        self._transitions: list[Transition] = []
        self._exit_callback: Optional[Callable[[State, Any], None]] = None
        self._exit_state: State = State(name="exit")
        self.add_state(state=self._exit_state)
        self._initial_state: State = self._exit_state
        self._current_state: State = self._exit_state

        self._exited = True
        self._spoof: bool = spoof

        # For transition lookup
        self._transition_map: dict[State, list[Transition]] = {}

    def __repr__(self) -> str:
        return "StateMachine"

    def add_states(self, states: list[State], initial_state_name: Optional[str] = None) -> None:
        """
        Add multiple states to the state machine at once.

        Args:
            states (list[State]): List of states to add
            initial_state_name (Optional[str]): Name of the state to set as initial state
        """
        for state in states:
            if state not in self._states:
                self._states.append(state)
                # Set up transition map entry
                if state not in self._transition_map:
                    self._transition_map[state] = []
            else:
                LOGGER.warning(f"State {state.name} already exists in state machine")

        # Set initial state if specified
        if initial_state_name:
            initial_state = self.get_state_by_name(initial_state_name)
            if initial_state:
                self._initial_state = initial_state
            else:
                LOGGER.warning(f"Initial state {initial_state_name} not found in added states")

    def add_events(self, events: list[Event]) -> None:
        """
        Add multiple events to the state machine at once.

        Args:
            events (list[Event]): List of events to add
        """
        for event in events:
            if event not in self._events:
                self._events.append(event)
            else:
                LOGGER.warning(f"Event {event.name} already exists in state machine")

    def add_transitions_from_dict(self, transitions_dict: dict) -> None:
        """
        Add multiple transitions from a dictionary configuration.

        The dictionary should have the following structure:
        {
            "source_state_name": {
                "event_name": {
                    "destination": "destination_state_name",
                    "criteria": optional_criteria_function,
                    "action": optional_action_function
                }
            }
        }

        Args:
            transitions_dict (dict): Dictionary of transitions to add
        """
        for source_name, events_dict in transitions_dict.items():
            source_state = self.get_state_by_name(source_name)
            if not source_state:
                LOGGER.warning(f"Source state {source_name} not found, skipping transitions")
                continue

            for event_name, transition_info in events_dict.items():
                # Find the event
                event = next((e for e in self._events if e.name == event_name), None)
                if not event:
                    LOGGER.warning(f"Event {event_name} not found, skipping transition")
                    continue

                # Find the destination state
                dest_name = transition_info.get("destination")
                if not dest_name:
                    LOGGER.warning(f"No destination specified for transition from {source_name} on {event_name}")
                    continue

                dest_state = self.get_state_by_name(dest_name)
                if not dest_state:
                    LOGGER.warning(f"Destination state {dest_name} not found, skipping transition")
                    continue

                # Add the transition - now we're sure source_state and dest_state are not None
                self.add_transition(
                    source=source_state,
                    destination=dest_state,
                    event=event,
                    criteria=transition_info.get("criteria"),
                    action=transition_info.get("action"),
                )

    def create_state(self, name: str, **kwargs: Any) -> State:
        """
        Create a new state and add it to the state machine.

        Args:
            name (str): Name of the state
            **kwargs: Additional arguments to pass to the State constructor

        Returns:
            State: The created state
        """
        state = State(name=name, **kwargs)
        self.add_state(state)
        return state

    def create_event(self, name: str) -> Event:
        """
        Create a new event and add it to the state machine.

        Args:
            name (str): Name of the event

        Returns:
            Event: The created event
        """
        event = Event(name=name)
        self.add_event(event)
        return event

    def add_state(self, state: State, initial_state: bool = False) -> None:
        """
        Add a state to the state machine.

        Args:
            state (State): The state to be added.
            initial_state (bool, optional): Whether the state is the initial state, by default False
        """
        if state in self._states:
            raise ValueError("State already exists.")

        self._states.append(state)

        if initial_state:
            self._initial_state = state

    def add_event(self, event: Event) -> None:
        if event not in self._events:
            self._events.append(event)
        else:
            LOGGER.warning(f"Event {event.name} already exists in state machine")

    def add_transition(
        self,
        source: State,
        destination: State,
        event: Event,
        criteria: Optional[Callable[[Any], bool]] = None,
        action: Optional[Callable[[Any], None]] = None,
    ) -> Optional[Transition]:
        """
        Add a transition to the state machine.

        Parameters
        ----------
        source : State
            The source state.
        destination : State
            The destination state.
        event : Event
            The event that triggers the transition.
        criteria : Callable[[Any], bool], optional
            A callback function that returns a boolean value,
            which determines whether the transition is valid, by default None
        action : Callable[[Any], None], optional
            A callback function to execute during the transition, by default None

        Returns
        -------
        Optional[Transition]
            The created transition, or None if the transition couldn't be created
        """
        transition = None

        if source in self._states and destination in self._states and event in self._events:
            transition = Transition(
                event=event,
                source=source,
                destination=destination,
                criteria=criteria,
                action=action,
            )
            self._transitions.append(transition)

            # Add to transition map for faster lookup
            if source not in self._transition_map:
                self._transition_map[source] = []
            self._transition_map[source].append(transition)

        return transition

    def update(self, event: Event, *args: Any, **kwargs: Any) -> None:
        """
        Update the state machine based on the given event.

        Parameters
        ----------
        event : Event
            The event that may trigger a transition
        *args, **kwargs
            Additional arguments to pass to transition criteria and actions
        """
        if not (self._initial_state or self._current_state):
            raise ValueError("State machine isn't active.")

        # Use transition map for faster lookup
        transitions = self._transition_map.get(self._current_state, [])

        for transition in transitions:
            if transition.event == event:
                # Pass spoof flag if we're in spoof mode
                if self._spoof:
                    kwargs["spoof"] = True

                self._current_state = transition(*args, **kwargs)

                if self._current_state.name == "exit" and not self._exited:
                    self._exited = True
                    if self._exit_callback:
                        self._exit_callback(self._current_state, *args, **kwargs)

                return

        # No valid transition found
        LOGGER.debug(f"Event {event.name} isn't valid at state {self._current_state.name}")

    def start(self, *args: Any, **kwargs: Any) -> None:
        if not self._initial_state:
            raise ValueError("Initial state not set.")

        self._current_state = self._initial_state
        self._exited = False
        self._current_state.start(*args, **kwargs)

    def stop(self, *args: Any, **kwargs: Any) -> None:
        if not self.is_active():
            raise ValueError("State machine isn't active.")

        self._current_state.stop(*args, **kwargs)
        self._current_state = self._exit_state
        self._exited = True

    def __enter__(self) -> "StateMachine":
        self.start()
        return self

    def __exit__(self, *args: Any, **kwargs: Any) -> None:
        self.stop()

    def __del__(self) -> None:
        self.stop()

    def __iter__(self) -> Iterator[State]:
        return iter(self._states)

    def __next__(self) -> State:
        try:
            return next(iter(self._states))
        except StopIteration as e:
            raise StopIteration("No states in state machine") from e

    def is_active(self) -> bool:
        return bool(self._current_state and self._current_state != self._exit_state)

    def spoof(self, spoof: bool) -> None:
        self._spoof = spoof

    @property
    def current_state(self) -> State:
        if self._current_state is None:
            return self._initial_state
        else:
            return self._current_state

    @property
    def states(self) -> list[str]:
        return [state.name for state in self._states]

    @property
    def is_spoofing(self) -> bool:
        return self._spoof

    def set_exit_callback(self, callback: Callable[[State, Any], None]) -> None:
        """
        Set a callback to be called when the state machine exits.

        Parameters
        ----------
        callback : Callable
            The callback function to call when exiting
        """
        self._exit_callback = callback

    def get_state_by_name(self, name: str) -> Optional[State]:
        """
        Get a state by its name.

        Parameters
        ----------
        name : str
            The name of the state to find

        Returns
        -------
        Optional[State]
            The state with the given name, or None if not found
        """
        for state in self._states:
            if state.name == name:
                return state
        return None


if __name__ == "__main__":
    sm = StateMachine()
    idle = State(name="idle")
    moving = State(name="moving")

    sm.add_state(idle)
    sm.add_state(moving)

    sm.add_transition(idle, moving, Event("event1"))

    sm.start()
    sm.update(Event("event1"))
    print(sm.current_state)
