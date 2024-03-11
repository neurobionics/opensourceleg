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
    Base class for all the states in the state machine. Please note that the knee and ankle
    impedance parameters are only used if the corresponding joint is active in the state. In addition
    to the default impedance parameters for each joint, you can also define custom parameters
    using the set_custom_data method.

    Parameters:
        name (str): Name of the state. Defaults to state.
        is_knee_active (bool): If True, the knee joint will be active in this state. Defaults to False.
        knee_stiffness (float): Stiffness of the knee joint in this state in Nm/rad. Defaults to 0.0.
        knee_damping (float): Damping of the knee joint in this state in Nm/(rad/s). Defaults to 0.0.
        knee_equilibrium_angle (float): Equilibrium angle of the knee joint in this state in radians. Defaults to 0.0.
        is_ankle_active (bool): If True, the ankle joint will be active in this state. Defaults to False.
        ankle_stiffness (float): Stiffness of the knee joint in this state in Nm/rad. Defaults to 0.0.
        ankle_damping (float): Damping of the knee joint in this state in Nm/(rad/s). Defaults to 0.0.
        ankle_equilibrium_angle (float): Equilibrium angle of the knee joint in this state in radians. Defaults to 0.0.
        minimum_time_in_state (float): Minimum time to be spent in this state in seconds. If the state is exited before
        this time, the state machine will wait until the minimum time has elapsed before transitioning to the next state.
        Defaults to 2.0.

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
        Sets the minimum time to be spent in this state in seconds.

        Parameters:
            time (float): Minimum time to be spent in this state in seconds. If the state is
                          exited before this time, the state machine will wait until the minimum time has
                          elapsed before transitioning to the next state.
        """
        self._min_time_in_state = time

    def set_knee_impedance_paramters(self, theta, k, b) -> None:
        """
        Sets the impedance parameters of the knee joint in this state. The impedance
        parameters are only used if the knee joint is active in this state.

        Parameters:
            theta (float): Equilibrium angle of the knee joint in this state in radians.
            k (float): Stiffness of the knee joint in this state in Nm/rad.
            b (float): Damping of the knee joint in this state in Nm/(rad/s).

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
        Sets the impedance parameters of the ankle joint in this state. The impedance
        parameters are only used if the ankle joint is active in this state.

        Parameters:
            theta (float): Equilibrium angle of the ankle joint in this state in radians.
            k (float): Stiffness of the ankle joint in this state in Nm/rad.
            b (float): Damping of the ankle joint in this state in Nm/(rad/s).

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
        Sets a custom data entry for the state. This data can be used to store any
        additional information that you want to associate with the state. Custom data is stored as a
        dictionary as key value pairs. Multiple custom data entries can be added to the state.

        Parameters:
            key (str): Key of the custom data
            value (Any): Value of the custom data
        """
        self._custom_data[key] = value

    def get_custom_data(self, key: str) -> Any:
        """
        Gets a custom data entry for the state. Please note that if the key does not exist, this method will raise a KeyError.

        Parameters:
            key (str): Key of the custom data.

        Returns:
            Any: Value of the custom data
        """
        return self._custom_data[key]

    def on_entry(self, callback: Callable[[Any], None]) -> None:
        """
        Adds a function or callback to be called when the state is entered.

        Parameters:
            callback (Callable[[Any], None]): Function to be called when the state is entered.
        """
        self._entry_callbacks.append(callback)

    def on_exit(self, callback: Callable[[Any], None]) -> None:
        """
        Adds a function or callback to be called when the state is exited.

        Parameters:
            callback (Callable[[Any], None]): Function to be called when the state is exited.
        """
        self._exit_callbacks.append(callback)

    def start(self, data: Any) -> None:
        """

        Parameters:
            data (Any): Any custom data that you'd like to pass to the state's entry callbacks.
        """
        self._time_entered = time.time()
        for c in self._entry_callbacks:
            c(data)

    def stop(self, data: Any) -> None:
        """
        Stops the state by calling all of its exit callbacks in the order they were added.

        Parameters:
            data (Any): Any custom data that you'd like to pass to the state's exit callbacks.
        """
        self._time_exited = time.time()
        for c in self._exit_callbacks:
            c(data)

    def make_knee_active(self):
        """
        Sets the knee joint to be active in this state.

        Parameters:
            None

        Note:
            The knee impedance parameters are only used if the knee is
            active.
        """
        self._is_knee_active = True

    def make_ankle_active(self):
        """
        Sets the ankle joint to be active in this state.

        Parameters:
            None

        Note:
            The ankle impedance parameters are only used if the ankle is
            active.
        """
        self._is_ankle_active = True

    @property
    def name(self) -> str:
        """name (str): Name of the state."""
        return self._name

    @property
    def knee_stiffness(self) -> float:
        """knee_stiffness (float): Stiffness of the knee joint in this state in Nm/rad."""
        return self._knee_stiffness

    @property
    def knee_damping(self) -> float:
        """knee_damping (float): Damping of the knee joint in this state in Nm/(rad/s)."""
        return self._knee_damping

    @property
    def knee_theta(self) -> float:
        """knee_theta (float): Equilibrium angle of the knee joint in this state in radians."""
        return self._knee_theta

    @property
    def ankle_stiffness(self) -> float:
        """ankle_stiffness (float): Stiffness of the ankle joint in this state in Nm/rad."""
        return self._ankle_stiffness

    @property
    def ankle_damping(self) -> float:
        """ankle_damping (float): Damping of the ankle joint in this state in Nm/(rad/s)."""
        return self._ankle_damping

    @property
    def ankle_theta(self) -> float:
        """ankle_theta (float): Equilibrium angle of the ankle joint in this state in radians."""
        return self._ankle_theta

    @property
    def is_knee_active(self) -> bool:
        """is_knee_active (bool): Is the knee joint set to be active in this state?"""
        return self._is_knee_active

    @property
    def is_ankle_active(self) -> bool:
        """is_ankle_active (bool): Is the ankle joint set to be active in this state?"""
        return self._is_ankle_active

    @property
    def minimum_time_spent_in_state(self) -> float:
        """minimum_time_spent_in_state (float): Minimum time to be spent in this state in seconds."""
        return self._min_time_in_state

    @property
    def current_time_in_state(self) -> float:
        """current_time_in_state (float): Current time spent in this state in seconds."""
        return time.time() - self._time_entered

    @property
    def time_spent_in_state(self) -> float:
        """time_spent_in_state (float): Total time spent in this state in seconds."""
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
    Base class for all the events in the state machine. An event is an unique identifier that
    corresponds to a transition in the state machine.

    Parameters:
        name (str): Name of the event.
    """

    def __init__(self, name) -> None:
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
        """name (str): Name of the event."""
        return self._name


class Transition:
    """
    Base class for all the transitions in the state machine. A transition is a directed link between
    two states in the state machine. A transition is tied to an event and is triggered only when the
    callback function returns True.

    Parameters:
        event (Event): Event corresponding to the transition.
        source (State): Source state of the transition.
        destination (State): Destination state of the transition.
        callback (Callable[[Any], bool]): A callback function that returns a boolean value, which
        determines whether the transition should be triggered.
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
        """
        Adds a criteria to the transition. The transition will be triggered only if all the criteria are met.

        Parameters:
            callback (Callable[[Any], bool]): A callback function that returns a boolean
                                              value, which determines whether the transition should be triggered.
        """
        self._criteria = callback

    def add_action(self, callback: Callable[[Any], Any]) -> None:
        """
        Adds an action to the transition. This function will be called when the transition is triggered.

        Parameters:
            callback (Callable[[Any], Any]): Function or callback to be called when the
                                             transition is triggered.

        """
        self._action = callback

    @property
    def event(self) -> Event:
        """event (Event): Event corresponding to the transition."""
        return self._event

    @property
    def source_state(self) -> State:
        """source_state (State): Source state of the transition."""
        return self._source_state

    @property
    def destination_state(self) -> State:
        """destination_state (State): Destination state of the transition."""
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
    A Finite State Machine (FSM) class to design and implement state machines for controlling
    the Open-Source Leg or any other hardware system.

    Parameters:

    osl (Any): Open-Source Leg instance or any other hardware object to be
               controlled. This object should ideally have all the necessary control methods and sensor data.
               Defaults to none.
    spoof (bool): If True, the state machine will spoof the state transitions--ie, it will not check
                  the criteria for transitioning but will instead transition after the minimum time spent in state
                  has elapsed. This is useful for testing.

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
        Adds a state to the state machine.

        Parameters:
            state (State): State to be added to the state machine.
            initial_state (bool): If True, the state will be set as the initial state of the state machine. Defaults to False.

        Raises:
            ValueError: If state already exixts in the state machine
        """
        if state in self._states:
            raise ValueError("State already exists.")

        self._states.append(state)

        if initial_state:
            self._initial_state = state

    def add_event(self, event: Event) -> None:
        """
        Adds an event to the state machine.

        Parameters:
            event (Event): Event to be added to the state machine.
        """
        self._events.append(event)

    def add_transition(
        self,
        source: State,
        destination: State,
        event: Event,
        callback: Callable[[Any], bool] = None,
    ) -> Optional[Transition]:
        """
        Adds a transition to the state machine.

        Parameters:
            source (State): Source state of the transition.
            destination (State): Destination state of the transition.
            event (Event): Event to trigger the transition.
            callback (Callable[[Any], bool]): A callback function that returns a boolean value, which determines whether the transition
                                              should be triggered. Defaults to None.

        Returns:
            Optional[Transition]
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
        """
        Updates the state machine. This method should be called in a loop to update
        the state machine's state and to trigger transitions.

        Parameters:
            data (Any): Any custom data to be used with the state machine. This data will be passed to the state's exit callbacks.

        Raises:
            ValueError: It the OSL isn't active
        """
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
        """
        Starts the state machine. This method should be called before calling the update method.

        Parameters:
            data (Any): Any custom data that you'd like to pass to the initial state's entry callbacks.

        Raises:
            ValueError: If initial state is not set
        """
        if not self._initial_state:
            raise ValueError("Initial state not set.")

        self._current_state = self._initial_state
        self._exited = False
        self._current_state.start(data=data)

    def stop(self, data: Any = None) -> None:
        """
        Stops the state machine. This method should be called before the state machine goes out of scope.

        Parameters:
            data (Any): Any custom data that you'd like to pass to the exit state's exit callbacks.

        Raises:
            ValueError: If the OSL isn't active
        """
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
        """current_state (State): Current state of the state machine."""
        if self._current_state is None:
            return self._initial_state
        else:
            return self._current_state

    @property
    def states(self):
        """states (list[State]): List of all the states in the state machine."""
        return [state.name for state in self._states]

    @property
    def is_spoofing(self):
        """is_spoofing (bool): True if the state machine is spoofing the state transitions."""
        return self._spoof


if __name__ == "__main__":
    pass
