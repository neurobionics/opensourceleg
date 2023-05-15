#!/usr/bin/python3
# A simple and scalable Finite State Machine module

from typing import Any, Callable, List, Optional


class State:
    """
    State class
    """

    def __init__(self, name, theta=0.0, k=0.0, b=0.0) -> None:
        self._name = name

        # Impedance parameters
        self._theta = theta
        self._k = k
        self._b = b

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

    def set_impedance_paramters(self, theta, k, b) -> None:
        self._theta = theta
        self._k = k
        self._b = b

    def get_impedance_paramters(self):
        return self._theta, self._k, self._b

    def on_entry(self, callback: Callable[[Any], None]):
        self._entry_callbacks.append(callback)

    def on_exit(self, callback: Callable[[Any], None]):
        self._exit_callbacks.append(callback)

    def start(self, data: Any):
        for c in self._entry_callbacks:
            c(data)

    def stop(self, data: Any):
        for c in self._exit_callbacks:
            c(data)

    @property
    def name(self):
        return self._name

    @property
    def equilibrium_angle(self):
        return self._theta

    @property
    def stiffness(self):
        return self._k

    @property
    def damping(self):
        return self._b


class Idle(State):
    def __init__(self, status="Idle") -> None:
        self._name = status
        super().__init__(self._name)

    @property
    def status(self):
        return self._name


class Event:
    """
    Event class
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
        self._event = event
        self._source_state = source
        self._destination_state = destination

        self._criteria: Optional[Callable[[Any], bool]] = callback
        self._action: Optional[Callable[[Any], None]] = None

    def __call__(self, data: Any) -> Any:
        raise NotImplementedError

    def add_criteria(self, callback: Callable[[Any], bool]):
        self._criteria = callback

    def add_action(self, callback: Callable[[Any], Any]):
        self._action = callback

    @property
    def event(self):
        return self._event

    @property
    def source_state(self):
        return self._source_state

    @property
    def destination_state(self):
        return self._destination_state


class FromToTransition(Transition):
    def __init__(
        self,
        event: Event,
        source: State,
        destination: State,
        callback: Callable[[Any], bool] = None,
    ) -> None:
        super().__init__(event, source, destination, callback)

        self._from = source
        self._to = destination

    def __call__(self, data: Any):
        if not self._criteria or self._criteria(data):
            if self._action:
                self._action(data)

            self._from.stop(data)
            self._to.start(data)

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
        self.add_state(self._exit_state)
        self._exited = True

        self._osl = osl

    def add_state(self, state: State, initial_state: bool = False):
        if state in self._states:
            raise ValueError("State already exists.")

        self._states.append(state)

        if not self._initial_state and initial_state:
            self._initial_state = state

    def add_event(self, event: Event):
        self._events.append(event)

    def add_transition(
        self,
        source: State,
        destination: State,
        event: Event,
        callback: Callable[[Any], bool] = None,
    ) -> Optional[Transition]:
        transition = None

        if (
            source in self._states
            and destination in self._states
            and event in self._events
        ):
            transition = FromToTransition(event, source, destination, callback)
            self._transitions.append(transition)

        return transition

    def update(self, data: Any = None):
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
            self._osl.log.warn(f"Event isn't valid at {self._current_state.name}")

    def start(self, data: Any = None):
        if not self._initial_state:
            raise ValueError("Initial state not set.")

        self._current_state = self._initial_state
        self._exited = False
        self._current_state.start(data)

    def stop(self, data: Any = None):
        if not (self._initial_state or self._current_state):
            raise ValueError("OSL isn't active.")

        self._current_state.stop(data)
        self._current_state = self._exit_state
        self._exited = True

    def is_on(self) -> bool:
        if self._current_state and self._current_state != self._exit_state:
            return True
        else:
            return False

    @property
    def current_state(self):
        return self._current_state
