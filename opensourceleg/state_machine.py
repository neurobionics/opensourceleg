#!/usr/bin/python3
# A simple and scalable Finite State Machine module

from typing import Any, Callable, List, Optional

from dataclasses import dataclass, field


@dataclass
class StateMachineData:
    """
    A class to represent the data of the state machine.
    """

    is_knee_active: bool = False
    knee_stiffness: float = 0.0
    knee_damping: float = 0.0
    knee_theta: float = 0.0

    is_ankle_active: bool = False
    ankle_stiffness: float = 0.0
    ankle_damping: float = 0.0
    ankle_theta: float = 0.0

    custom_data: dict[str, Any] = field(default_factory=dict)


class State:
    """
    A class to represent a state in a finite state machine.
    """

    def __init__(
        self, name: str = "state", data: StateMachineData = StateMachineData()
    ) -> None:
        """
        Parameters
        ----------
        name : str
            The name of the state.
        theta : float, optional
            The theta parameter of the impedance model, by default 0.0
        k : float, optional
            The stiffness of the joint in PID units, by default 0.0
        b : float, optional
            The damping of the joint in PID units, by default 0.0
        """
        self._name: str = name
        self._data: StateMachineData = data

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

    def set_knee_impedance_paramters(self, theta, k, b) -> None:
        self._data.knee_theta = theta
        self._data.knee_stiffness = k
        self._data.knee_damping = b

    def set_ankle_impedance_paramters(self, theta, k, b) -> None:
        self._data.ankle_theta = theta
        self._data.ankle_stiffness = k
        self._data.ankle_damping = b

    def set_custom_data(self, key: str, value: Any) -> None:
        self._data.custom_data[key] = value

    def get_custom_data(self, key: str) -> Any:
        return self._data.custom_data[key]

    def on_entry(self, callback: Callable[[Any], None]) -> None:
        self._entry_callbacks.append(callback)

    def on_exit(self, callback: Callable[[Any], None]) -> None:
        self._exit_callbacks.append(callback)

    def start(self, data: Any) -> None:
        for c in self._entry_callbacks:
            c(data)

    def stop(self, data: Any) -> None:
        for c in self._exit_callbacks:
            c(data)

    @property
    def name(self) -> str:
        return self._name

    @property
    def knee_stiffness(self) -> float:
        return self._data.knee_stiffness

    @property
    def knee_damping(self) -> float:
        return self._data.knee_damping

    @property
    def knee_theta(self) -> float:
        return self._data.knee_theta

    @property
    def ankle_stiffness(self) -> float:
        return self._data.ankle_stiffness

    @property
    def ankle_damping(self) -> float:
        return self._data.ankle_damping

    @property
    def ankle_theta(self) -> float:
        return self._data.ankle_theta

    @property
    def is_knee_active(self) -> bool:
        return self._data.is_knee_active

    @is_knee_active.setter
    def is_knee_active(self, value: bool) -> None:
        self._data.is_knee_active = value

    @property
    def is_ankle_active(self) -> bool:
        return self._data.is_ankle_active

    @is_ankle_active.setter
    def is_ankle_active(self, value: bool) -> None:
        self._data.is_ankle_active = value


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
        callback: Callable[[Any], bool] = None,  # type: ignore
    ) -> None:
        self._event: Event = event
        self._source_state: State = source
        self._destination_state: State = destination

        self._criteria: Optional[Callable[[Any], bool]] = callback
        self._action: Optional[Callable[[Any], None]] = None

    def __call__(self, data: Any) -> Any:
        raise NotImplementedError

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
        callback: Callable[[Any], bool] = None,  # type: ignore
    ) -> None:
        super().__init__(
            event=event, source=source, destination=destination, callback=callback
        )

        self._from: State = source
        self._to: State = destination

    def __call__(self, data: Any) -> State:
        if not self._criteria or self._criteria(data):
            if self._action:
                self._action(data)

            self._from.stop(data=data)
            self._to.start(data=data)

            return self._to
        else:
            return self._from


class StateMachine:
    def __init__(self, osl=None) -> None:
        # State Machine Variables
        self._states: list[State] = []
        self._events: list[Event] = []
        self._transitions: list[Transition] = []
        self._initial_state: Optional[State] = None
        self._current_state: Optional[State] = None
        self._exit_callback: Optional[Callable[[Idle, Any], None]] = None
        self._exit_state = Idle()
        self.add_state(state=self._exit_state)
        self._initial_state = self._exit_state

        self._exited = True
        self._osl: Any = None  # type: ignore

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
        callback: Callable[[Any], bool] = None,  # type: ignore
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
                self._current_state = transition(self._osl)

                if isinstance(self._current_state, Idle) and not self._exited:
                    self._exited = True

                    if self._exit_callback:
                        self._exit_callback(self._current_state, data)

                validity = True
                break

        if not validity:
            assert self._osl is not None
            self._osl.log.debug(f"Event isn't valid at {self._current_state.name}")  # type: ignore

    def start(self, data: Any = None) -> None:
        if not self._initial_state:
            raise ValueError("Initial state not set.")

        self._current_state = self._initial_state
        self._exited = False
        self._current_state.start(data=data)

    def stop(self, data: Any = None) -> None:
        if not (self._initial_state or self._current_state):
            raise ValueError("OSL isn't active.")

        self._current_state.stop(data=data)  # type: ignore
        self._current_state = self._exit_state
        self._exited = True

    def is_on(self) -> bool:
        if self._current_state and self._current_state != self._exit_state:
            return True
        else:
            return False

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
    def current_state_name(self):
        return self.current_state.name  # type: ignore
