#!/usr/bin/python3
# A simple and scalable Finite State Machine module

from typing import Any, Callable, List, Optional

import time
from dataclasses import dataclass, field

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
        is_knee_active (bool): Whether the knee is active. Default: False
        knee_stiffness (float): Knee stiffness in Nm/rad
        knee_damping (float): Knee damping in Nm/rad/sec
        knee_equilibrium_angle (float): Knee equilibrium angle
        is_ankle_active (bool): Whether the ankle is active. Default: False
        ankle_stiffness (float): Ankle stiffness in Nm/rad
        ankle_damping (float): Ankle damping in Nm/rad/sec
        ankle_equilibrium_angle (float): Ankle equilibrium angle
        minimum_time_in_state (float): Minimum time spent in the state in seconds. Default: 2.0

    Note:
        The knee and ankle impedance parameters are only used if the
        corresponding joint is active. You can also set custom data
        using the `set_custom_data` method.
    """

    def __init__(
        self,
        name: str = "state",
        is_knee_active: bool = False,
        knee_stiffness: float = 0.0,
        knee_damping: float = 0.0,
        knee_equilibrium_angle: float = 0.0,
        is_ankle_active: bool = False,
        ankle_stiffness: float = 0.0,
        ankle_damping: float = 0.0,
        ankle_equilibrium_angle: float = 0.0,
        minimum_time_in_state: float = 2.0,
    ) -> None:

        self._name: str = name

        self._is_knee_active: bool = is_knee_active
        self._knee_stiffness: float = knee_stiffness
        self._knee_damping: float = knee_damping
        self._knee_theta: float = knee_equilibrium_angle

        self._is_ankle_active: bool = is_ankle_active
        self._ankle_stiffness: float = ankle_stiffness
        self._ankle_damping: float = ankle_damping
        self._ankle_theta: float = ankle_equilibrium_angle

        self._custom_data: dict[str, Any] = field(default_factory=dict)

        self._time_entered: float = 0.0
        self._time_exited: float = 0.0
        self._min_time_in_state: float = minimum_time_in_state

        # Callbacks
        self._entry_callbacks: list[Callable[[Any], None]] = []
        self._exit_callbacks: list[Callable[[Any], None]] = []

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

    def set_minimum_time_spent_in_state(self, time: float) -> None:
        """
        Set the minimum time spent in the state

        Args:
            time (float): Minimum time spent in the state in seconds
        """
        self._min_time_in_state = time

    def set_knee_impedance_paramters(self, theta, k, b) -> None:
        """
        Set the knee impedance parameters

        Args:
            theta (float): Equilibrium angle of the knee joint
            k (float): Stiffness of the knee joint
            b (float): Damping of the knee joint

        Note:
            The knee impedance parameters are only used if the knee is
            active. You can make the knee active by calling the
            `make_knee_active` method.
        """
        self._knee_theta = theta
        self._knee_stiffness = k
        self._knee_damping = b

    def set_ankle_impedance_paramters(self, theta, k, b) -> None:
        """
        Set the ankle impedance parameters

        Args:
            theta (float): Equilibrium angle of the ankle joint
            k (float): Stiffness of the ankle joint
            b (float): Damping of the ankle joint

        Note:
            The ankle impedance parameters are only used if the ankle is
            active. You can make the ankle active by calling the
            `make_ankle_active` method.
        """
        self._ankle_theta = theta
        self._ankle_stiffness = k
        self._ankle_damping = b

    def set_custom_data(self, key: str, value: Any) -> None:
        """
        Set custom data for the state. The custom data is a dictionary
        that can be used to store any data you want to associate with
        the state.

        Args:
            key (str): Key of the data
            value (Any): Value of the data
        """
        self._custom_data[key] = value

    def get_custom_data(self, key: str) -> Any:
        """
        Get custom data for the state. The custom data is a dictionary
        that can be used to store any data you want to associate with
        the state.

        Args:
            key (str): Key of the data

        Returns:
            Any: Value of the data
        """
        return self._custom_data[key]

    def on_entry(self, callback: Callable[[Any], None]) -> None:
        self._entry_callbacks.append(callback)

    def on_exit(self, callback: Callable[[Any], None]) -> None:
        self._exit_callbacks.append(callback)

    def start(self, data: Any) -> None:
        self._time_entered = time.time()
        for c in self._entry_callbacks:
            c(data)

    def stop(self, data: Any) -> None:
        self._time_exited = time.time()
        for c in self._exit_callbacks:
            c(data)

    def make_knee_active(self):
        """
        Make the knee active

        Note:
            The knee impedance parameters are only used if the knee is
            active.
        """
        self._is_knee_active = True

    def make_ankle_active(self):
        """
        Make the ankle active

        Note:
            The ankle impedance parameters are only used if the ankle is
            active.
        """
        self._is_ankle_active = True

    @property
    def name(self) -> str:
        return self._name

    @property
    def knee_stiffness(self) -> float:
        return self._knee_stiffness

    @property
    def knee_damping(self) -> float:
        return self._knee_damping

    @property
    def knee_theta(self) -> float:
        return self._knee_theta

    @property
    def ankle_stiffness(self) -> float:
        return self._ankle_stiffness

    @property
    def ankle_damping(self) -> float:
        return self._ankle_damping

    @property
    def ankle_theta(self) -> float:
        return self._ankle_theta

    @property
    def is_knee_active(self) -> bool:
        return self._is_knee_active

    @property
    def is_ankle_active(self) -> bool:
        return self._is_ankle_active

    @property
    def minimum_time_spent_in_state(self) -> float:
        return self._min_time_in_state

    @property
    def current_time_in_state(self) -> float:
        return time.time() - self._time_entered

    @property
    def time_spent_in_state(self) -> float:
        return self._time_exited - self._time_entered


class Idle(State):
    def __init__(self) -> None:
        self._name = "idle"
        super().__init__(name=self._name)

    @property
    def status(self) -> str:
        return self._name


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
        callback: Callable[[Any], bool] = None,
    ) -> None:
        self._event: Event = event
        self._source_state: State = source
        self._destination_state: State = destination

        self._criteria: Optional[Callable[[Any], bool]] = callback
        self._action: Optional[Callable[[Any], None]] = None

    def __call__(self, data: Any) -> Any:
        raise NotImplementedError

    def __repr__(self) -> str:
        return (
            f"Transition[{self._source_state.name} -> {self._destination_state.name}]"
        )

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

    def __init__(self, osl=None, spoof: bool = False) -> None:
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
        self._osl: Any = osl

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
            raise ValueError("OSL isn't active.")

        for transition in self._transitions:
            if transition.source_state == self._current_state:
                self._current_state = transition(self._osl, spoof=self.is_spoofing)

                if isinstance(self._current_state, Idle) and not self._exited:
                    self._exited = True

                    if self._exit_callback:
                        self._exit_callback(self._current_state, data)

                validity = True
                break

        if not validity:
            assert self._osl is not None
            self._osl.log.debug(f"Event isn't valid at {self._current_state.name}")

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
