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
        print("Entering: ", self._name)
        for c in self._entry_callbacks:
            c(data)

    def stop(self, data: Any):
        print("Exiting: ", self._name)
        for c in self._exit_callbacks:
            c(data)

    def increase_theta(self, increment=45.5111):
        self._theta = self._theta + increment

    def decrease_theta(self, decrement=45.5111):
        self._theta = self._theta - decrement

    def increase_stiffness(self, increment=10):
        self._k = self._k + increment

    def decrease_stiffness(self, decrement=10):
        self._k = self._k - decrement

    def increase_damping(self, increment=10):
        self._b = self._b + increment

    def decrease_damping(self, decrement=10):
        self._b = self._b - decrement

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
