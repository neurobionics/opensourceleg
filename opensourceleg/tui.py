from typing import Callable

import TermTk as ttk
from attr import dataclass


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


# create a tree data structure class
class Group:
    def __init__(self, name: str, parent: "Group" = None):
        self._name = name
        self._parent = parent
        self._children = []

    def add_child(self, child: "Group" = None):
        self._children.append(child)

    @property
    def children(self):
        return self._children

    @property
    def parent(self):
        return self._parent

    @property
    def name(self):
        return self._name

    @property
    def is_root(self):
        return self.parent is None

    @property
    def is_leaf(self):
        return len(self.children) == 0

    def __str__(self):
        return f"{self.name} -> {self.children}"


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
    ):
        self.root_ttk = ttk.TTk()
        self.timer = ttk.TTkTimer()
        self.dt = 1.0 / frequency

        self._is_running = False

        self._layouts = {
            "horizontal": ttk.TTkHBoxLayout,
            "vertical": ttk.TTkVBoxLayout,
            "grid": ttk.TTkGridLayout,
        }

        self._modes = {
            0: "voltage",
            1: "position",
            2: "current",
            3: "impedance",
        }

        self.root_ttk.setLayout(self._layouts["grid"]())
        self._groups = {}

        self._knee = None
        self._ankle = None

        self._joint = None
        self._loadcell = None
        self._attribute = "motor_position"

        self._buttons = {}
        self._radio_buttons = {}
        self._checkboxes = {}
        self._categories = {}
        self._dropdowns = {}
        self._values = {}

        self._knee_values = {}
        self._ankle_values = {}
        self._other_values = {}

        self._texts = {}

        self.frame = ttk.TTkFrame(
            parent=self.root_ttk,
            border=True,
            title=title,
            titleColor=ttk.TTkColor.BOLD + ttk.TTkColor.fg(COLORS.maize),
        )
        self.frame.setLayout(self._layouts[layout]())
        self._groups["root"] = self.frame

        self._plot_parent: ttk.TTkFrame = None
        self._plot: ttk.TTkGraph = None
        self._pvalue = [0.0]
        self._pcounter: int = 0

        self._kmode: int = 0
        self._amode: int = 0

        self._plot_title: str = "Plot"

        self._update_callbacks: list[callable] = []

        self.timer.timeout.connect(self.update)
        self.timer.start(1)

    @ttk.pyTTkSlot()
    def update(self):
        if self._plot is not None:

            if "loadcell" in self._attribute:
                if self.loadcell is not None:
                    if "fx" in self._attribute:
                        self.set_plot_value(getattr(self.loadcell, "fx"))
                    elif "fy" in self._attribute:
                        self.set_plot_value(getattr(self.loadcell, "fy"))
                    else:
                        self.set_plot_value(getattr(self.loadcell, "fz"))

            else:
                if self.joint is not None:
                    self.set_plot_value(getattr(self.joint, self._attribute))

            self._plot.addValue(self._pvalue)

            if "attributes" in self._categories:
                _plot_name = self.attribute.replace("_", " ").title()

            else:
                _plot_name = self._plot_title

            self._plot_parent.setTitle(
                ttk.TTkString(f"{_plot_name}: {self._pvalue[0]:0.2f}")
            )
            self._pcounter += 1

        for callback in self._update_callbacks:
            callback()

        self.timer.start(self.dt)

    def add_update_callback(self, callback: callable):
        self._update_callbacks.append(callback)

    def set_active_attribute(self, attribute: str):
        self._attribute = attribute

    def set_plot_value(self, value: float):
        self._pvalue = [value]

    def add_knee(self, knee):
        self._knee = knee

    def add_ankle(self, ankle):
        self._ankle = ankle

    def run(self):
        self._is_running = True
        self.root_ttk.mainloop()

    def quit(self):
        self._is_running = False
        self.root_ttk.quit()

    def add_group(
        self,
        name: str = "Group",
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
        Adds a group to the TUI

        Args:
            name (str): Name of the group. Defaults to "Group".
            parent (str): Parent of the group. Defaults to "root".
            layout (str): Layout of the group. Defaults to "horizontal".
            title_color (str): Title color of the group. Defaults to COLORS.white.
            border_color (str): _description_. Defaults to COLORS.white.
            border (bool): _description_. Defaults to True.
            show_title (bool): _description_. Defaults to False.
            row (int): _description_. Defaults to 0.
            col (int): _description_. Defaults to 0.

        Raises:
            ValueError: _description_
        """

        self._groups[name] = ttk.TTkFrame(
            border=border,
            titleColor=ttk.TTkColor.fg(title_color),
            borderColor=ttk.TTkColor.fg(border_color),
        )

        _name = " ".join(name.split("_")).title()

        if show_title:
            self._groups[name].setTitle(ttk.TTkString(" " + _name + " "))

        self._groups[name].setLayout(self._layouts[layout]())

        if parent in self._groups:

            if (
                self._groups[parent].layout().__class__.__name__
                == ttk.TTkGridLayout.__name__
            ):
                self._groups[parent].layout().addWidget(
                    self._groups[name],
                    row,
                    col,
                )

            else:
                self._groups[parent].layout().addWidget(self._groups[name])
        else:
            raise ValueError(f"Parent group: {parent} does not exist.")

    def add_plot(
        self,
        parent: str = "root",
        color: str = COLORS.white,
        row: int = 0,
        col: int = 0,
    ):
        self._plot = ttk.TTkGraph(
            color=ttk.TTkColor.fg(
                color,
            )
        )

        self._plot_parent = self._groups[parent]

        if (
            self._groups[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._groups[parent].layout().addWidget(
                self._plot,
                row,
                col,
            )

        else:
            self._groups[parent].layout().addWidget(self._plot)

    def add_dropdown(
        self,
        name: str = "dropdown",
        parent: str = "root",
        options: list[str] = [],
        callback: callable = None,
        callback_args: list = [],
        row: int = 0,
        col: int = 0,
    ):
        if (
            self._groups[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._groups[parent].layout().addWidget(
                _dropdown := ttk.TTkComboBox(
                    list=options,
                ),
                row,
                col,
            )

        else:
            self._groups[parent].layout().addWidget(
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
            self._groups[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._groups[parent].layout().addWidget(
                ttk.TTkLabel(text=name, color=ttk.TTkColor.fg(color)),
                row,
                col,
            )

        else:
            self._groups[parent].layout().addWidget(
                ttk.TTkLabel(text=name, color=ttk.TTkColor.fg(color)),
            )

    def add_value(
        self,
        name: str = "value",
        parent: str = "root",
        default: int = 0,
        callback: callable = None,
        callback_args: list = [],
        row: int = 0,
        col: int = 0,
    ):

        if (
            self._groups[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._groups[parent].layout().addWidget(
                _value := ttk.TTkLineEdit(
                    text=default, inputType=ttk.TTkK.Input_Number
                ),
                row,
                col,
            )

        else:
            self._groups[parent].layout().addWidget(
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
        callback: Callable = None,
        callback_args: list = [],
        color: str = COLORS.white,
        border: bool = True,
        row: int = 0,
        col: int = 0,
    ):
        _name = " ".join(name.split("_")).title()

        if (
            self._groups[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._groups[parent].layout().addWidget(
                _button := ttk.TTkButton(
                    text=_name,
                    color=ttk.TTkColor.BOLD + ttk.TTkColor.bg(color),
                    border=border,
                ),
                row,
                col,
            )

        else:
            self._groups[parent].layout().addWidget(
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
        callback: Callable = None,
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
            self._groups[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._groups[parent].layout().addWidget(
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
            self._groups[parent].layout().addWidget(
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
        callback: Callable = None,
        callback_args: list = [],
        color: str = COLORS.white,
        is_checked: bool = False,
        row: int = 0,
        col: int = 0,
    ):
        _name = " ".join(name.split("_")).title()

        if (
            self._groups[parent].layout().__class__.__name__
            == ttk.TTkGridLayout.__name__
        ):
            self._groups[parent].layout().addWidget(
                _checkbox := ttk.TTkCheckbox(
                    text=_name,
                    color=ttk.TTkColor.fg(color),
                    checked=is_checked,
                ),
                row,
                col,
            )

        else:
            self._groups[parent].layout().addWidget(
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
        callback: Callable = None,
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
        callback: Callable = None,
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
        callback: Callable = None,
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
        callback: Callable = None,
        callback_args: list = [],
    ):
        callback_args.insert(0, self._dropdowns[parent + "_" + name].currentIndex())

        self._dropdowns[parent + "_" + name].currentIndexChanged.connect(
            lambda x: callback(name=name, parent=parent, args=callback_args)
        )

    def set_joint_mode(self, **kwargs):
        name = kwargs["name"]
        parent = kwargs["parent"]
        mode_id = int(self._dropdowns[parent + "_" + name].currentIndex())

        if "knee" in name.lower():
            self._kmode = mode_id
        elif "ankle" in name.lower():
            self._amode = mode_id

    def set_category(
        self,
        **kwargs,
    ):
        name = kwargs["name"]
        args = kwargs["args"]
        category = args[0]

        self._categories[category] = name

    def set_active_joint(self, **kwargs):
        name = kwargs["name"]
        parent = kwargs["parent"]

        _name = (name + parent).lower()

        if "knee" in _name:
            self._joint = self._knee
        elif "ankle" in _name:
            self._joint = self._ankle

    def set_loadcell(
        self,
        loadcell,
    ):
        self._loadcell = loadcell

    def set_value(
        self,
        **kwargs,
    ):

        name = kwargs["name"]
        parent = kwargs["parent"]
        value = int(self._values[parent + "_" + name].text())

        _name = parent + name

        if "knee" in _name.lower():
            self._knee_values[name] = value
        elif "ankle" in _name.lower():
            self._ankle_values[name] = value
        else:
            self._other_values[name] = value

    def test_button(
        self,
        **kwargs,
    ):
        name = kwargs["name"]
        parent = kwargs["parent"]
        args = kwargs["args"]

        print(f"Button {name} in group {parent} was clicked.")

    @property
    def joint(self):
        return self._joint

    @property
    def loadcell(self):
        return self._loadcell

    @property
    def attribute(self):
        return self._attribute

    @property
    def categories(self):
        return self._categories

    @property
    def values(self):
        return self._values

    @property
    def dropdowns(self):
        return self._dropdowns

    @property
    def control_modes(self):
        return self._modes

    @property
    def is_running(self):
        return self._is_running


if __name__ == "__main__":
    tui = TUI()
    tui.run()
