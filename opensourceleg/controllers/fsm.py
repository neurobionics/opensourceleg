"""
(c) 2025 Open-Source Leg

A simple and scalable Finite State Machine module.
It includes the State, Event, Transition, and StateMachine classes.

Usage:
1. Use the `State` class to represent a state in the FSM.
2. Utilize the `Event` class to define events that trigger state transitions.
3. Create transitions between states using the `Transition` class. Add criteria and actions as needed.
4. Instantiate the `StateMachine` class, add states, events, and transitions, and start the FSM.
5. Use the "main" function as an example to see how to use the FSM.

Author:
    - Senthur Ayyappan <senthura@umich.edu>
"""

import time
from collections.abc import Iterator
from typing import Any, Callable, Optional

from opensourceleg.logging.logger import LOGGER
from opensourceleg.time import SoftRealtimeLoop


class State:
    def __init__(
        self,
        name: str = "state",
        minimum_time_in_state: float = 0.0,
        entry_callbacks: Optional[list[Callable[[Any], None]]] = None,
        exit_callbacks: Optional[list[Callable[[Any], None]]] = None,
        **kwargs: Any,
    ) -> None:
        """
        A class to represent a state in a finite state machine.
        Custom attributes can be added to the state using keyword arguments.

        Args:
            name: Name of the state
            minimum_time_in_state: Minimum time spent in the state in seconds. \
                Transition to the next state will not occur until this time has elapsed.
                Defaults to 0.0 seconds.
            entry_callbacks: List of functions to call when entering the state.
            exit_callbacks: List of functions to call when exiting the state.
            **kwargs: Additional attributes to set on the state

        Example:
            >>> state = State(
            ...     name="idle",
            ...     minimum_time_in_state=2.0,
            ...     entry_callbacks=[print("Entering idle state")],
            ...     exit_callbacks=[print("Exiting idle state")],
            ... )
        """
        self._name: str = name
        self._time_entered: float = 0.0
        self._time_exited: float = 0.0
        self._min_time_in_state: float = minimum_time_in_state
        self._entry_callbacks: list[Callable[[Any], None]] = entry_callbacks or []
        self._exit_callbacks: list[Callable[[Any], None]] = exit_callbacks or []

        for key, value in kwargs.items():
            setattr(self, key, value)

    def __eq__(self, __o: Any) -> bool:
        return bool(__o.name == self._name)

    def __ne__(self, __o: Any) -> bool:
        return not self.__eq__(__o)

    def __hash__(self) -> int:
        return hash(self._name)

    def __repr__(self) -> str:
        return f"State[{self._name}]"

    def set_minimum_time_spent_in_state(self, time: float) -> None:
        """
        Set the minimum time spent in the state

        Args:
            time: Minimum time spent in the state in seconds

        Example:
            >>> state.set_minimum_time_spent_in_state(2.0)
        """
        self._min_time_in_state = time

    def add_entry_callback(self, callback: Callable[[Any], None]) -> None:
        """
        Add a callback function to be called when entering the state.

        Args:
            callback: Function to be called when entering the state

        Example:
            >>> state.add_entry_callback(lambda: if t > 0: print("Entering idle state"))
        """
        self._entry_callbacks.append(callback)

    def add_exit_callback(self, callback: Callable[[Any], None]) -> None:
        """
        Add a callback function to be called when exiting the state.

        Args:
            callback: Function to be called when exiting the state

        Example:
            >>> state.add_exit_callback(lambda: if t > 1: print("Exiting idle state"))
        """
        self._exit_callbacks.append(callback)

    def start(self, *args: Any, **kwargs: Any) -> None:
        """
        Start the state.

        Args:
            *args: Arguments to pass to the entry callbacks
            **kwargs: Keyword arguments to pass to the entry callbacks

        Example:
            >>> state.start(x=1)
        """
        self._time_entered = time.monotonic()
        for c in self._entry_callbacks:
            c(*args, **kwargs)

    def stop(self, *args: Any, **kwargs: Any) -> None:
        """
        Stop the state.

        Args:
            *args: Arguments to pass to the exit callbacks
            **kwargs: Keyword arguments to pass to the exit callbacks

        Example:
            >>> state.stop(x=1)
        """
        self._time_exited = time.monotonic()
        for c in self._exit_callbacks:
            c(*args, **kwargs)

    @property
    def name(self) -> str:
        """
        Get the name of the state.

        Returns:
            str: The name of the state
        """
        return self._name

    @property
    def minimum_time_spent_in_state(self) -> float:
        """
        Get the minimum time spent in the state.

        Returns:
            float: The minimum time spent in the state
        """
        return self._min_time_in_state

    @property
    def current_time_in_state(self) -> float:
        """
        Get the current time spent in the state.

        Returns:
            float: The current time spent in the state
        """
        return time.monotonic() - self._time_entered

    @property
    def time_spent_in_state(self) -> float:
        """
        Get the time spent in the state.

        Returns:
            float: The time spent in the state
        """
        return self._time_exited - self._time_entered


class Event:
    def __init__(self, name: str) -> None:
        """
        A class to represent an event in a finite state machine.
        This is a simple label to identify an event that triggers a transition.

        Args:
            name: The name of the event.

        Example:
            >>> event = Event("walk")
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
        """
        Get the name of the event.

        Returns:
            str: The name of the event
        """
        return self._name


class Transition:
    def __init__(
        self,
        event: Event,
        source: State,
        destination: State,
        criteria: Optional[Callable[..., bool]] = None,
        action: Optional[Callable[..., None]] = None,
    ) -> None:
        """
        Transition class that handles state transitions in a finite state machine.
        A transition connects a source state to a destination state and is triggered by an event.
        It should include criteria that must be met for the transition to occur and actions to execute
        during the transition.

        Args:
            event: The event that triggers this transition
            source: The source state
            destination: The destination state
            criteria: Optional function that returns True if transition should occur
            action: Optional function to execute during transition

        Example:
            >>> transition = Transition(
            ...     event=Event("walk"),
            ...     source=State("idle"),
            ...     destination=State("walking"),
            ...     criteria=lambda: True,
            ...     action=lambda: print("Walking")
            ... )
        """
        self._event: Event = event
        self._source_state: State = source
        self._destination_state: State = destination

        self._criteria: Optional[Callable[..., bool]] = criteria
        self._action: Optional[Callable[..., None]] = action

    def __call__(self, **kwargs: Any) -> State:
        if not self._criteria:
            criteria_met = True
        else:
            try:
                import inspect

                # Filter kwargs to only include parameters the function expects
                criteria_params = inspect.signature(self._criteria).parameters
                filtered_kwargs = {k: v for k, v in kwargs.items() if k in criteria_params}

                criteria_met = self._criteria(**filtered_kwargs)
            except Exception as e:
                LOGGER.warning(f"Failed to call criteria function for transition {self}: {e}")
                return self._source_state

        if criteria_met:
            if self._action:
                try:
                    import inspect

                    # Filter kwargs to only include parameters the function expects
                    action_params = inspect.signature(self._action).parameters
                    filtered_kwargs = {k: v for k, v in kwargs.items() if k in action_params}

                    self._action(**filtered_kwargs)
                except Exception as e:
                    LOGGER.warning(f"Failed to call action function for transition {self}: {e}")

            self._source_state.stop(**kwargs)
            self._destination_state.start(**kwargs)

            return self._destination_state
        else:
            return self._source_state

    def __repr__(self) -> str:
        return f"Transition[{self._source_state.name} -> {self._destination_state.name}]"

    def add_criteria(self, callback: Callable[[Any], bool]) -> None:
        """
        Add a criteria function to the transition.

        Args:
            callback: Function that returns True if transition should occur

        Example:
            >>> transition.add_criteria(lambda: if t > 0: True)
        """
        self._criteria = callback

    def add_action(self, callback: Callable[[Any], Any]) -> None:
        """
        Add an action function to the transition.

        Args:
            callback: Function to execute during transition

        Example:
            >>> transition.add_action(lambda: print("Walking"))
        """
        self._action = callback

    @property
    def event(self) -> Event:
        """
        Get the event that triggers this transition.

        Returns:
            Event: The event that triggers this transition
        """
        return self._event

    @property
    def source_state(self) -> State:
        """
        Get the source state of this transition.

        Returns:
            State: The source state of this transition
        """
        return self._source_state

    @property
    def destination_state(self) -> State:
        """
        Get the destination state of this transition.

        Returns:
            State: The destination state of this transition
        """
        return self._destination_state


class StateMachine:
    def __init__(
        self,
        states: Optional[list[State]] = None,
        events: Optional[list[Event]] = None,
        initial_state_name: Optional[str] = None,
    ) -> None:
        """
        A flexible finite state machine class that supports:
        - Multiple states with transitions between them
        - Events that trigger transitions
        - Entry and exit actions for states
        - Minimum time constraints for states

        Args:
            states: List of states to add to the state machine
            events: List of events to add to the state machine
            initial_state_name: Name of the initial state to set

        Example:
            >>> sm = StateMachine(states=[State("idle"), State("walking"), State("running")],
            ...     events=[Event("walk"), Event("run"), Event("stop")],
            ...     initial_state_name="idle")
        """
        self._states: list[State] = []
        self._events: list[Event] = []
        self._transitions: list[Transition] = []
        self._transition_map: dict[State, list[Transition]] = {}

        self._exit_state: State = State(name="exit")
        self._initial_state: Optional[State] = None
        self._current_state: State = self._exit_state
        self._exited = True

        if states:
            self.add_states(states, initial_state_name)

        if events:
            self.add_events(events)

        # add the exit state after all user states are added
        self.add_state(state=self._exit_state)

    def __repr__(self) -> str:
        return "StateMachine"

    def add_states(self, states: list[State], initial_state_name: Optional[str] = None) -> None:
        """
        Add multiple states to the state machine at once.

        Args:
            states: List of states to add
            initial_state_name: Name of the state to set as initial state

        Example:
            >>> sm.add_states([State("idle"), State("walking"), State("running")], initial_state_name="idle")
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
            events: List of events to add

        Example:
            >>> sm.add_events([Event("walk"), Event("run"), Event("stop")])
        """
        for event in events:
            if event not in self._events:
                self._events.append(event)
            else:
                LOGGER.warning(f"Event {event.name} already exists in state machine")

    def add_transitions_from_dict(self, transitions_dict: dict) -> None:
        """
        Add multiple transitions from a dictionary configuration.

        Args:
            transitions_dict: Dictionary of transitions to add

        Example:
            >>> sm.add_transitions_from_dict({
            ...     "idle": {
            ...         "walk": {
            ...             "destination": "walking",
            ...             "criteria": lambda: True,
            ...             "action": lambda: print("Walking")
            ...         },
            ...         "run": {
            ...             "destination": "running",
            ...             "criteria": lambda: True,
            ...             "action": lambda: print("Running")
            ...         },
            ...     }
            ... })
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
            name: Name of the state
            **kwargs: Additional arguments to pass to the State constructor

        Returns:
            State: The created state

        Example:
            >>> sm.create_state(
            ...     name="idle",
            ...     minimum_time_spent_in_state=2.0,
            ...     entry_callbacks=[print("Entering idle state")],
            ...     exit_callbacks=[print("Exiting idle state")],
            ... )
        """
        state = State(name=name, **kwargs)
        self.add_state(state)
        return state

    def create_event(self, name: str) -> Event:
        """
        Create a new event and add it to the state machine.

        Args:
            name: Name of the event

        Returns:
            Event: The created event

        Example:
            >>> sm.create_event("walk")
        """
        event = Event(name=name)
        self.add_event(event)
        return event

    def add_state(self, state: State, initial_state: bool = False) -> None:
        """
        Add a state to the state machine.

        Args:
            state: The state to be added.
            initial_state: Whether the state is the initial state, by default False

        Example:
            >>> sm.add_state(State("idle"), initial_state=True)
        """
        if state in self._states:
            raise ValueError("State already exists.")

        self._states.append(state)

        # Initialize transition map entry
        if state not in self._transition_map:
            self._transition_map[state] = []

        if initial_state:
            self._initial_state = state

        # Set the first non-exit state as initial if no initial state is set yet
        if self._initial_state is None and state.name != "exit":
            self._initial_state = state

    def add_event(self, event: Event) -> None:
        """
        Add an event to the state machine.

        Args:
            event: The event to be added

        Example:
            >>> sm.add_event(Event("walk"))
        """
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

        Args:
            source: The source state
            destination: The destination state
            event: The event that triggers the transition
            criteria: A callback function that returns a boolean value, which determines whether the transition is valid
            action: A callback function to execute during the transition

        Returns:
            Optional[Transition]: The created transition, or None if the transition couldn't be created

        Example:
            >>> sm.add_transition(
            ...     source=State("idle"),
            ...     destination=State("walking"),
            ...     event=Event("walk"),
            ...     criteria=lambda: True,
            ...     action=lambda: print("Walking"),
            ... )
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

    def update(self, **kwargs: Any) -> None:
        """
        Update the state machine, checking all possible transitions from the current state.
        If any transition's criteria are met, the state machine will transition automatically.

        Args:
            **kwargs: Named arguments to pass to transition criteria and actions

        Example:
            >>> sm.update(t=t)
        """
        if not self._current_state:
            raise ValueError("State machine isn't active.")

        # Check if minimum time in state has elapsed
        if self._current_state.current_time_in_state < self._current_state.minimum_time_spent_in_state:
            return

        transitions = self._transition_map.get(self._current_state, [])

        if not transitions and self._current_state != self._exit_state:
            LOGGER.debug(f"No transitions defined for state {self._current_state.name}")
            return

        for transition in transitions:
            next_state = transition(**kwargs)

            # If state changed, update current state and exit
            if next_state != self._current_state:
                self._current_state = next_state

                if self._current_state.name == "exit" and not self._exited:
                    self._exited = True

                return

        LOGGER.debug(f"No valid transitions from state {self._current_state.name}")

    def start(self, *args: Any, **kwargs: Any) -> None:
        """
        Start the state machine.

        Args:
            *args: Arguments to pass to the initial state
            **kwargs: Keyword arguments to pass to the initial state

        Example:
            >>> sm.start(x=1)
        """
        if not self._initial_state:
            raise ValueError(
                "No initial state set. Add at least one state with initial_state=True or add a non-exit state."
            )

        self._current_state = self._initial_state
        self._exited = False
        self._current_state.start(*args, **kwargs)

    def stop(self, *args: Any, **kwargs: Any) -> None:
        """
        Stop the state machine.

        Args:
            *args: Arguments to pass to the exit state
            **kwargs: Keyword arguments to pass to the exit state

        Example:
            >>> sm.stop(x=1)
        """
        if not self.is_active():
            raise ValueError("State machine isn't active.")

        self._current_state.stop(*args, **kwargs)
        self._current_state = self._exit_state
        self._exited = True

    def __enter__(self) -> "StateMachine":
        """
        Enter the context manager for the state machine.

        Returns:
            StateMachine: The state machine

        Example:
            >>> with sm:
            ...     # Use the state machine
            ...     pass
        """
        self.start()
        return self

    def __exit__(self, *args: Any, **kwargs: Any) -> None:
        """
        Exit the context manager for the state machine.

        Args:
            *args: Arguments to pass to the exit state
            **kwargs: Keyword arguments to pass to the exit state

        Example:
            >>> with sm:
            ...     # Use the state machine
            ...     pass
        """
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

    @property
    def current_state(self) -> State:
        """
        Get the current state of the state machine.

        Returns:
            State: The current state of the state machine
        """
        if self._current_state is None:
            return self._initial_state
        else:
            return self._current_state

    @property
    def states(self) -> list[str]:
        """
        Get the names of all states in the state machine.

        Returns:
            list[str]: The names of all states in the state machine
        """
        return [state.name for state in self._states]

    def get_state_by_name(self, name: str) -> Optional[State]:
        """
        Get a state by its name.

        Args:
            name: The name of the state to find

        Returns:
            Optional[State]: The state with the given name, or None if not found
        """
        for state in self._states:
            if state.name == name:
                return state
        return None


if __name__ == "__main__":
    DT = 1 / 200
    clock = SoftRealtimeLoop(DT)

    sm = StateMachine()

    # Define states first
    idle = State(name="idle")
    walking = State(name="walking")
    running = State(name="running")

    # Add states to the state machine
    sm.add_state(idle)
    sm.add_state(walking)
    sm.add_state(running)

    # Create and add events to the state machine
    walk_event = Event("walk")
    run_event = Event("run")
    stop_event = Event("stop")

    sm.add_event(walk_event)
    sm.add_event(run_event)
    sm.add_event(stop_event)

    # Define criteria for transitions
    def walk_criteria(t: float) -> bool:
        return t > 1.0 and t < 2.0

    def run_criteria(t: float) -> bool:
        return t > 2.0 and t < 3.0

    def stop_criteria(t: float) -> bool:
        return t > 3.0 and t < 4.0

    # Now use the event objects when adding transitions to the state machine
    sm.add_transition(idle, walking, walk_event, criteria=walk_criteria)
    sm.add_transition(walking, running, run_event, criteria=run_criteria)
    sm.add_transition(running, idle, stop_event, criteria=stop_criteria)

    # Test the state machine update method with transitions
    with sm:
        for t in clock:
            sm.update(t=t)
            LOGGER.info(f"Current state: {sm.current_state.name}, Time: {t:.2f}")

    LOGGER.info(f"State machine exited, current state: {sm.current_state.name}")
    clock.stop()

    # Test the state machine iterator (offline)
    with sm:
        LOGGER.info(f"Starting state machine with initial state: {sm.current_state.name}")
        for state in sm:
            LOGGER.info(f"Current state: {state.name}")
            time.sleep(1)

    LOGGER.info(f"State machine exited, current state: {sm.current_state.name}")
