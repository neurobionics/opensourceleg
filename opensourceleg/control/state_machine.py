#!/usr/bin/python3
# A simple and scalable Finite State Machine module

from typing import Any, Callable, List, Optional

import time

from opensourceleg.logging import LOGGER

"""
The state_machine module provides classes for implementing a finite state machine (FSM).
It includes the State, Idle, Event, Transition, FromToTransition, and StateMachine classes.

Usage:
1. Use the `State` class to represent a state in the FSM. Configure its parameters
   such as joint activity, impedance parameters, and callbacks.
2. Extend the `Idle` class to create specific idle states with additional properties if needed.
3. Utilize the `Event` class to define events that trigger state transitions.
4. Create transitions between states using the `Transition` and `FromToTransition` classes.
   Add criteria and actions as needed.
5. Instantiate the `StateMachine` class, add states, events, and transitions, and start the FSM.
"""


class State:
    """
    A class to represent a state in a finite state machine.

    Args:
        name (str): Name of the state
        minimum_time_in_state (float): Minimum time spent in the state in seconds. Default: 2.0

    Note:
        The knee and ankle impedance parameters are only used if the
        corresponding joint is active. You can also set custom data
        using the `set_custom_data` method.
    """

    def __init__(
        self,
        name: str = "state",
        minimum_duration: float = 2.0,
        entry_callbacks: list[Callable[[Any], None]] = [],
        exit_callbacks: list[Callable[[Any], None]] = [],
    ) -> None:

        self._name: str = name
        self._time_entered: float = 0.0
        self._time_exited: float = 0.0
        self._minimum_duration: float = minimum_duration

        # Callbacks
        self._entry_callbacks: entry_callbacks
        self._exit_callbacks: exit_callbacks

    def __eq__(self, __o) -> bool:
        if __o.name == self._name:
            return True
        else:
            return False

    def __ne__(self, __o) -> bool:
        return not self.__eq__(__o)

    def __call__(self, data: Any) -> Any:
        pass

    def __repr__(self) -> str:
        return f"State[{self._name}]"

    def set_minimum_duration(self, value: float) -> None:
        """
        Set the minimum time spent in the state

        Args:
            time (float): Minimum time spent in the state in seconds
        """
        self._minimum_duration = value

    def add_entry_callback(self, callback: Callable[[Any], None]) -> None:
        self._entry_callbacks.append(callback)

    def add_exit_callback(self, callback: Callable[[Any], None]) -> None:
        self._exit_callbacks.append(callback)

    def start(self, data: Any) -> None:
        self._time_entered = time.time()
        for c in self._entry_callbacks:
            c(data)

    def stop(self, data: Any) -> None:
        self._time_exited = time.time()
        for c in self._exit_callbacks:
            c(data)

    @property
    def name(self) -> str:
        return self._name

    @property
    def minimum_time_spent_in_state(self) -> float:
        return self._minimum_duration

    @property
    def current_time_in_state(self) -> float:
        return time.time() - self._time_entered

    @property
    def time_spent_in_state(self) -> float:
        return self._time_exited - self._time_entered


class Event:
    """
    Event class
    """

    def __init__(self, name) -> None:
        """
        Parameters
        ----------
        name : str
            The name of the event.
        """
        self._name = name

    def __eq__(self, __o) -> bool:
        if __o.name == self._name:
            return True
        else:
            return False

    def __ne__(self, __o) -> bool:
        return not self.__eq__

    def __repr__(self) -> str:
        return f"Event[{self._name}]"

    @property
    def name(self):
        return self._name


class Transition:
    """
    Transition class
    """

    def __init__(
        self,
        event: Event,
        source: State,
        destination: State,
        criteria: Callable[[Any], bool],
        action: Callable[[Any], None],
    ) -> None:
        self._event: Event = event
        self._source_state: State = source
        self._destination_state: State = destination

        self._criteria: Callable[[Any], bool] = criteria
        self._action: Callable[[Any], None] = action

    def __call__(self, data: Any) -> Any:
        raise NotImplementedError

    def __repr__(self) -> str:
        return (
            f"Transition[{self._source_state.name} -> {self._destination_state.name}]"
        )

    @property
    def event(self) -> Event:
        return self._event

    @property
    def source_state(self) -> State:
        return self._source_state

    @property
    def destination_state(self) -> State:
        return self._destination_state


class FromToTransition(Transition):
    def __init__(
        self,
        event: Event,
        source: State,
        destination: State,
        callback: Callable[[Any], bool] = None,
    ) -> None:
        super().__init__(
            event=event, source=source, destination=destination, callback=callback
        )

        self._from: State = source
        self._to: State = destination

    def __call__(self, data: Any, spoof: bool = False) -> State:
        if spoof:
            if (
                self._from.current_time_in_state
                > self._from.minimum_time_spent_in_state
            ):
                if self._action:
                    self._action(data)

                self._from.stop(data=data)
                self._to.start(data=data)

                return self._to

            else:
                return self._from

        elif not self._criteria or self._criteria(data):
            if self._action:
                self._action(data)

            self._from.stop(data=data)
            self._to.start(data=data)

            return self._to

        else:
            return self._from


class StateMachine:
    """
    State Machine class

    Parameters
    ----------
    osl : Any
        The OpenSourceLeg object.
    spoof : bool
        If True, the state machine will spoof the state transitions--ie, it will not
        check the criteria for transitioning but will instead transition after the
        minimum time spent in state has elapsed. This is useful for testing.
        Defaults to False.

    Attributes
    ----------
    current_state : State
        The current state of the state machine.
    states : list[State]
        The list of states in the state machine.
    is_spoofing : bool
        Whether or not the state machine is spoofing the state transitions.
    """

    def __init__(self, robot=None, spoof: bool = False) -> None:
        # State Machine Variables
        self._states: list[State] = []
        self._events: list[Event] = []
        self._transitions: list[FromToTransition] = []
        self._exit_callback: Optional[Callable[[Idle, Any], None]] = None
        self._exit_state: State = Idle()
        self.add_state(state=self._exit_state)
        self._initial_state: State = self._exit_state
        self._current_state: State = self._exit_state

        self._exited = True
        self._robot: Any = robot

        self._spoof: bool = spoof

    def __repr__(self) -> str:
        return f"StateMachine"

    def add_state(self, state: State, initial_state: bool = False) -> None:
        """
        Add a state to the state machine.

        Parameters
        ----------
        state : State
            The state to be added.
        initial_state : bool, optional
            Whether the state is the initial state, by default False
        """
        if state in self._states:
            raise ValueError("State already exists.")

        self._states.append(state)

        if initial_state:
            self._initial_state = state

    def add_event(self, event: Event) -> None:
        self._events.append(event)

    def add_transition(
        self,
        source: State,
        destination: State,
        event: Event,
        callback: Callable[[Any], bool] = None,
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
        callback : Callable[[Any], bool], optional
            A callback function that returns a boolean value, which determines whether the transition is valid, by default None
        """
        transition = None

        if (
            source in self._states
            and destination in self._states
            and event in self._events
        ):
            transition = FromToTransition(
                event=event, source=source, destination=destination, callback=callback
            )
            self._transitions.append(transition)

        return transition

    def update(self, data: Any = None) -> None:
        validity = False

        if not (self._initial_state or self._current_state):
            raise ValueError("StateMachine isn't active.")

        for transition in self._transitions:
            if transition.source_state == self._current_state:
                self._current_state = transition(self._robot, spoof=self.is_spoofing)

                if isinstance(self._current_state, Idle) and not self._exited:
                    self._exited = True

                    if self._exit_callback:
                        self._exit_callback(self._current_state, data)

                validity = True
                break

        if not validity:
            assert self._robot is not None
            LOGGER.debug(f"Event isn't valid at {self._current_state.name}")

    def start(self, data: Any = None) -> None:
        if not self._initial_state:
            raise ValueError("Initial state not set.")

        self._current_state = self._initial_state
        self._exited = False
        self._current_state.start(data=data)

    def stop(self, data: Any = None) -> None:
        if not (self._initial_state or self._current_state):
            raise ValueError("OSL isn't active.")

        self._current_state.stop(data=data)
        self._current_state = self._exit_state
        self._exited = True

    def is_on(self) -> bool:
        if self._current_state and self._current_state != self._exit_state:
            return True
        else:
            return False

    def spoof(self, spoof: bool) -> None:
        self._spoof = spoof

    @property
    def current_state(self):
        if self._current_state is None:
            return self._initial_state
        else:
            return self._current_state

    @property
    def states(self):
        return [state.name for state in self._states]

    @property
    def is_spoofing(self):
        return self._spoof


if __name__ == "__main__":
    pass
