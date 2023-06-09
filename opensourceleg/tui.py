from typing import Any, Callable

import threading
import time
from dataclasses import dataclass

import TermTk as ttk
from TermTk.TTkCore.string import TTkString


# create a dataclass for various colors (str: hex codes)
@dataclass
class Colors:
    red: str = "#ff0000"
    green: str = "#00ff00"
    blue: str = "#0000ff"
    yellow: str = "#ffff00"
    cyan: str = "#00ffff"
    magenta: str = "#ff00ff"
    white: str = "#ffffff"
    black: str = "#000000"
    turquoise: str = "#00dddd"
    orange: str = "#ff8800"
    grey: str = "#888888"
    mblue: str = "#00274C"
    maize: str = "#FBEC5D"


@dataclass
class Plot:
    title: str = "plot"
    parent: ttk.TTkFrame = None  # type: ignore
    object: Any = None
    attribute: str = None  # type: ignore
    graph: ttk.TTkGraph = None  # type: ignore


@dataclass
class StateVisualizer:
    frame: ttk.TTkFrame = None  # type: ignore
    object: Any = None
    attribute: str = None  # type: ignore
    states: dict[ttk.TTkButton] = None  # type: ignore
    previous_state: str = None  # type: ignore


COLORS = Colors()


class TUI:
    """
    A class used to represent a TUI
    """

    def __init__(
        self,
        title: str = " Open-source Leg ",
        frequency: int = 30,
        layout: str = "horizontal",
    ) -> None:
        self.root_ttk = ttk.TTk()
        self.timer = ttk.TTkTimer()
        self.dt: float = 1.0 / frequency

        self._is_running = False

        self._layouts = {
            "horizontal": ttk.TTkHBoxLayout,
            "vertical": ttk.TTkVBoxLayout,
            "grid": ttk.TTkGridLayout,
        }

        self.root_ttk.setLayout(layout=self._layouts["grid"]())

        self._panels = {}
        self._plots = {}
        self._buttons = {}
        self._radio_buttons = {}
        self._checkboxes = {}
        self._categories = {}
        self._dropdowns = {}
        self._values = {}
        self._texts = {}

        self._state_vizualisers = {}

        self.frame = ttk.TTkFrame(
            parent=self.root_ttk,
            border=True,
            title=title,
            titleColor=ttk.TTkColor.BOLD + ttk.TTkColor.fg(COLORS.cyan),
        )

        self.frame.setLayout(self._layouts[layout]())
        self._panels["root"] = self.frame

        self._update_callbacks: list[Callable] = []  # type: ignore
        self._update_args: list = []  # type: ignore

        self.timer.timeout.connect(self.update)
        self.timer.start(1)

    @ttk.pyTTkSlot()
    def update(self):
        if self._plots:
            for _plot in self._plots.keys():
                if self._plots[_plot].object is not None:
                    self._plots[_plot].graph.addValue(
                        [
                            getattr(
                                self._plots[_plot].object, self._plots[_plot].attribute
                            )
                        ]
                    )
                else:
                    self._plots[_plot].graph.addValue([0.0])

        if self._state_vizualisers:
            for _sv in self._state_vizualisers.keys():
                if self._state_vizualisers[_sv].object is not None:
                    _current_state = getattr(
                        self._state_vizualisers[_sv].object,
                        self._state_vizualisers[_sv].attribute,
                    )

                    self._state_vizualisers[_sv].states[_current_state].setDisabled(
                        False
                    )

                    if (
                        self._state_vizualisers[_sv].previous_state
                        and self._state_vizualisers[_sv].previous_state
                        != _current_state
                    ):
                        self._state_vizualisers[_sv].states[
                            self._state_vizualisers[_sv].previous_state
                        ].setDisabled(True)

                    self._state_vizualisers[_sv].previous_state = _current_state

        for i, callback in enumerate(self._update_callbacks):
            if self._update_args[i]:
                callback(self._update_args[i])

        self.timer.start(self.dt)

    def add_update_callback(self, callback: Callable = lambda: None, args: Any = None):  # type: ignore
        self._update_callbacks.append(callback)
        self._update_args.append(args)

    def set_active_attribute(self, attribute: str):
        self._attribute = attribute

    def set_plot_value(self, value: float):
        self._pvalue = [value]

    def set_plot_title(self, title: str):
        self._plot_title = title

    def run(self):
        self._is_running = True
        self.root_ttk.mainloop()

    def quit(self):
        self._is_running = False
        self.root_ttk.quit()

    def add_panel(
        self,
        name: str = "Panel",
        parent: str = "root",
        layout: str = "horizontal",
        title_color: str = COLORS.white,
        border_color: str = COLORS.white,
        border: bool = True,
        show_title: bool = False,
        row: int = 0,
        col: int = 0,
    ):
        """
        Adds a panel to the TUI

        Args:
            name (str): Name of the panel. Defaults to "Panel".
            parent (str): Parent of the panel. Defaults to "root".
            layout (str): Layout of the panel. Defaults to "horizontal".
            title_color (str): Title color of the panel. Defaults to COLORS.white.
            border_color (str): _description_. Defaults to COLORS.white.
            border (bool): _description_. Defaults to True.
            show_title (bool): _description_. Defaults to False.
            row (int): _description_. Defaults to 0.
            col (int): _description_. Defaults to 0.

        Raises:
            ValueError: _description_
        """

        self._panels[name] = ttk.TTkFrame(
            border=border,
            titleColor=ttk.TTkColor.fg(title_color),
            borderColor=ttk.TTkColor.fg(border_color),
        )

        _name = " ".join(name.split("_")).title()

        if show_title:
            self._panels[name].setTitle(ttk.TTkString(" " + _name + " "))

        self._panels[name].setLayout(self._layouts[layout]())

        if parent in self._panels:

            if (
                self._panels[parent].layout().__class__.__name__
                == ttk.TTkGridLayout.__name__
            ):
                self._panels[parent].layout().addWidget(
                    self._panels[name],
                    row,
                    col,
                )

            else:
                self._panels[parent].layout().addWidget(self._panels[name])
        else:
            raise ValueError(f"Parent panel: {parent} does not exist.")

    def set_panel_color(self, name: str, color: str):
        self._panels[name].setBorderColor(ttk.TTkColor.fg(color))

    def add_state_visualizer(
        self,
        name: str = "state_visualizer",
        parent: str = "root",
        states: list[str] = [],
        object: Any = None,
        attribute: str = None,  # type: ignore
        layout: str = "horizontal",
        title_color: str = COLORS.white,
        border_color: str = COLORS.white,
        border: bool = True,
        show_title: bool = True,
        row: int = 0,
        col: int = 0,
    ):
        _sv_frame = ttk.TTkFrame(
            border=border,
            titleColor=ttk.TTkColor.fg(title_color),
            borderColor=ttk.TTkColor.fg(border_color),
        )

        _sv_title = " ".join(name.split("_")).title()

        if show_title:
            _sv_frame.setTitle(ttk.TTkString(" " + _sv_title + " "))

        _sv_frame.setLayout(self._layouts[layout]())

        if (
            self._panels[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._panels[parent].layout().addWidget(
                _sv_frame,
                row,
                col,
            )

        else:
            self._panels[parent].layout().addWidget(
                _sv_frame,
            )

        _state_displays = {}

        for state in states:
            _sv_frame.layout().addWidget(  # type: ignore
                _button := ttk.TTkButton(
                    text=" ".join(state.split("_")).title(),
                    color=ttk.TTkColor.fg(title_color),
                    border=border,
                )
            )

            _button.setDisabled(disabled=True)
            _button.setBorderColor(color=ttk.TTkColor.fg(COLORS.green))
            _state_displays[state] = _button

        self._state_vizualisers[name] = StateVisualizer(
            frame=_sv_frame,
            object=object,
            attribute=attribute,
            states=_state_displays,
        )

    def add_plot(
        self,
        name: str = "plot",
        parent: str = "root",
        object=None,
        attribute: str = None,  # type: ignore
        color: str = COLORS.white,
        row: int = 0,
        col: int = 0,
    ) -> None:
        self._plots[name] = Plot(
            title=" ".join(name.split("_")).title(),
            parent=self._panels[parent],
            object=object,
            attribute=attribute,
            graph=ttk.TTkGraph(
                color=ttk.TTkColor.fg(
                    color,
                )
            ),
        )

        if (
            self._panels[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._panels[parent].layout().addWidget(
                self._plots[name].graph,
                row,
                col,
            )

        else:
            self._panels[parent].layout().addWidget(self._plots[name].graph)

    def add_dropdown(
        self,
        name: str = "dropdown",
        parent: str = "root",
        options: list[str] = [],
        callback: Callable = lambda: None,
        callback_args: list = [],
        row: int = 0,
        col: int = 0,
    ):
        if (
            self._panels[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._panels[parent].layout().addWidget(
                _dropdown := ttk.TTkComboBox(
                    list=options,
                ),
                row,
                col,
            )

        else:
            self._panels[parent].layout().addWidget(
                _dropdown := ttk.TTkComboBox(
                    list=options,
                )
            )

        self._dropdowns[parent + "_" + name] = _dropdown

        if callback is not None:
            self.connect_dropdown(
                name=name,
                parent=parent,
                callback=callback,
                callback_args=callback_args,
            )

    def add_text(
        self,
        name: str = "text",
        parent: str = "root",
        color: str = COLORS.white,
        row: int = 0,
        col: int = 0,
    ):

        if (
            self._panels[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._panels[parent].layout().addWidget(
                ttk.TTkLabel(
                    text=" ".join(name.split("_")).title(), color=ttk.TTkColor.fg(color)
                ),
                row,
                col,
            )

        else:
            self._panels[parent].layout().addWidget(
                ttk.TTkLabel(
                    text=" ".join(name.split("_")).title(), color=ttk.TTkColor.fg(color)
                ),
            )

    def add_value(
        self,
        name: str = "value",
        parent: str = "root",
        default: int = 0,
        callback: Callable = lambda: None,
        callback_args: list = [],
        row: int = 0,
        col: int = 0,
    ):

        if (
            self._panels[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._panels[parent].layout().addWidget(
                _value := ttk.TTkLineEdit(
                    text=default, inputType=ttk.TTkK.Input_Number
                ),
                row,
                col,
            )

        else:
            self._panels[parent].layout().addWidget(
                _value := ttk.TTkLineEdit(
                    text=default, inputType=ttk.TTkK.Input_Number
                ),
            )

        self._values[parent + "_" + name] = _value

        if callback is not None:
            self.connect_value(
                name=name,
                parent=parent,
                callback=callback,
                callback_args=callback_args,
            )

    def add_button(
        self,
        name: str = "button",
        parent: str = "root",
        callback: Callable = lambda: None,
        callback_args: list = [],
        color: str = COLORS.white,
        border: bool = True,
        row: int = 0,
        col: int = 0,
    ):
        _name = " ".join(name.split("_")).title()

        if (
            self._panels[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._panels[parent].layout().addWidget(
                _button := ttk.TTkButton(
                    text=_name,
                    color=ttk.TTkColor.BOLD + ttk.TTkColor.bg(color),
                    border=border,
                ),
                row,
                col,
            )

        else:
            self._panels[parent].layout().addWidget(
                _button := ttk.TTkButton(
                    text=_name, color=ttk.TTkColor.fg(color), border=border
                )
            )

        self._buttons[parent + "_" + name] = _button

        if callback is not None:
            self.connect_button(
                name=name,
                parent=parent,
                callback=callback,
                callback_args=callback_args,
            )

    def add_radio_button(
        self,
        name: str = "radio_button",
        category: str = "attributes",
        parent: str = "root",
        callback: Callable = lambda: None,
        callback_args: list = [],
        color: str = COLORS.white,
        is_checked: bool = False,
        row: int = 0,
        col: int = 0,
    ):
        """
        Adds a radio button to the TUI

        Args:
            name (str): Name of the radio button. Defaults to "radio_button".
            category (str): Category of the radio button; only one radio button can be checked in a category.
                            Defaults to "Attributes".
            parent (str): Parent of the radio button. Defaults to "root".
            callback (Callable): Callback function for the radio button. Defaults to None.
            callback_args (list): Arguments for the callback function. Defaults to [].
            color (str): Color of the radio button. Defaults to COLORS.white.
            is_checked (bool): Is the radio button checked? Defaults to False.
            row (int): Row of the radio button. Defaults to 0.
            col (int): Column of the radio button. Defaults to 0.
        """

        _name = " ".join(name.split("_")).title()

        if (
            self._panels[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._panels[parent].layout().addWidget(
                _radio_button := ttk.TTkRadioButton(
                    text=" " + _name,
                    name=category,
                    color=ttk.TTkColor.fg(color),
                    checked=is_checked,
                ),
                row,
                col,
            )

        else:
            self._panels[parent].layout().addWidget(
                _radio_button := ttk.TTkRadioButton(
                    text=" " + _name,
                    name=category,
                    color=ttk.TTkColor.fg(color),
                    checked=is_checked,
                )
            )

        self._radio_buttons[parent + "_" + name] = _radio_button

        if is_checked:
            self._categories[category] = name

        if callback is not None:
            self.connect_radio_button(
                name=name,
                parent=parent,
                category=category,
                callback=callback,
                callback_args=callback_args,
            )

    def add_checkbox(
        self,
        name: str = "Checkbox",
        parent: str = "root",
        callback: Callable = lambda: None,
        callback_args: list = [],
        color: str = COLORS.white,
        is_checked: bool = False,
        row: int = 0,
        col: int = 0,
    ):
        _name = " ".join(name.split("_")).title()

        if (
            self._panels[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._panels[parent].layout().addWidget(
                _checkbox := ttk.TTkCheckbox(
                    text=_name,
                    color=ttk.TTkColor.fg(color),
                    checked=is_checked,
                ),
                row,
                col,
            )

        else:
            self._panels[parent].layout().addWidget(
                _checkbox := ttk.TTkCheckbox(
                    text=_name,
                    color=ttk.TTkColor.fg(color),
                    checked=is_checked,
                )
            )

        self._checkboxes[parent + "_" + _name] = _checkbox

        if callback is not None:
            # TODO: Implement a connect checkbox function
            pass

    def connect_button(
        self,
        name: str,
        parent: str,
        callback: Callable = lambda: None,
        callback_args: list = [],
    ):
        self._buttons[parent + "_" + name].clicked.connect(
            lambda: callback(name=name, parent=parent, args=callback_args)
        )

    def connect_radio_button(
        self,
        name: str,
        parent: str,
        category: str = "attributes",
        callback: Callable = lambda: None,
        callback_args: list = [],
    ):
        callback_args.insert(0, category)

        self._radio_buttons[parent + "_" + name].clicked.connect(
            lambda: callback(name=name, parent=parent, args=callback_args)
        )

    def connect_value(
        self,
        name: str,
        parent: str,
        callback: Callable = lambda: None,
        callback_args: list = [],
    ):

        callback_args.insert(0, self._values[parent + "_" + name].text())

        self._values[parent + "_" + name].returnPressed.connect(
            lambda: callback(name=name, parent=parent, args=callback_args)
        )

    def connect_dropdown(
        self,
        name: str,
        parent: str,
        callback: Callable = lambda: None,
        callback_args: list = [],
    ):
        callback_args.insert(0, self._dropdowns[parent + "_" + name].currentIndex())

        self._dropdowns[parent + "_" + name].currentIndexChanged.connect(
            lambda x: callback(name=name, parent=parent, args=callback_args)
        )

    def set_category(
        self,
        **kwargs,
    ):
        name = kwargs["name"]
        args = kwargs["args"]
        category = args[0]

        self._categories[category] = name

    def set_value(
        self,
        **kwargs,
    ):
        pass

    def test_button(
        self,
        **kwargs,
    ):
        name = kwargs["name"]
        parent = kwargs["parent"]
        args = kwargs["args"]

        print(f"Button {name} in panel {parent} was clicked.")

    @property
    def panels(self):
        return self._panels

    @property
    def plots(self):
        return self._plots

    @property
    def buttons(self):
        return self._buttons

    @property
    def radio_buttons(self):
        return self._radio_buttons

    @property
    def checkboxes(self):
        return self._checkboxes

    @property
    def categories(self):
        return self._categories

    @property
    def values(self):
        return self._values

    @property
    def texts(self):
        return self._texts

    @property
    def dropdowns(self):
        return self._dropdowns

    @property
    def is_running(self):
        return self._is_running


def quit(**kwargs):
    tui.quit()


def other_thread(loop, a):
    for t in loop:
        a.value += 1
        a.t = t


class V:
    def __init__(self, value, loop, states) -> None:
        self.value = value
        self.states = states
        self.loop = loop
        self.state = states[0]

    def run(self):
        for t in self.loop:
            self.value += 1

            if self.value % 100 == 0:
                self.state = self.states[
                    (self.states.index(self.state) + 1) % len(self.states)
                ]


if __name__ == "__main__":
    from opensourceleg.utilities import SoftRealtimeLoop

    tui = TUI(frequency=200)
    loop = SoftRealtimeLoop(dt=1 / 200)

    states = ["early_stance", "late_stance", "early_swing", "late_swing"]

    a = V(0, loop, states)

    tui.add_state_visualizer(
        name="state_machine",
        parent="root",
        states=states,
        object=a,
        attribute="state",
    )

    threading.Thread(target=a.run).start()

    tui.run()
