from typing import Callable

import random

import numpy as np
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


class TUI(Colors):
    """
    A class used to represent a TUI
    """

    def __init__(
        self,
        title: str = " Open-source Leg ",
        frequency: int = 30,
        layout: str = "horizontal",
        default_cfg: bool = False,
    ):
        self.root_ttk = ttk.TTk()
        self.timer = ttk.TTkTimer()
        self.dt = 1.0 / frequency

        self._default_cfg = default_cfg

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
        self._pvalue: float = 0.0
        self._pcounter: int = 0

        self._kmode: int = 0
        self._amode: int = 0

        self._plot_title: str = "Plot"

        self._update_callbacks: list[callable] = []

        self.timer.timeout.connect(self.update)
        self.timer.start(1)

        if self._default_cfg:
            self.load_default_cfg()

    @ttk.pyTTkSlot()
    def update(self):
        if self._plot is not None:
            self._pvalue = [np.sin(self._pcounter * np.pi / 40)]
            self._plot.addValue(self._pvalue)

            if "attributes" in self._categories:
                _plot_name = self._categories["attributes"].replace("_", " ").title()

            else:
                _plot_name = self._plot_title

            self._plot_parent.setTitle(
                ttk.TTkString(f"{_plot_name}: {self._pvalue[0]:0.3f}")
            )
            self._pcounter += 1

        for callback in self._update_callbacks:
            callback()

        self.timer.start(self.dt)

    def add_update_callback(self, callback: callable):
        self._update_callbacks.append(callback)

    def run(self):
        self.root_ttk.mainloop()

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
        Adds a group or frame to the TUI

        Args:
            name (str): Name of the group. Defaults to "Group".
            parent (str): Parent of the group. Defaults to "Groups".
            layout (str): Layout of the group. Defaults to "horizontal".
            color (str): Title color of the group . Defaults to COLORS.white.
            border_color (str): Border color of the group. Defaults to COLORS.white.
            border (bool): Boolean flag to show or hide the border. Defaults to True.
            show_title (bool): Boolean flag to show or hide the title. Defaults to False.
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
        color: str = COLORS.white,
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
        color: str = COLORS.white,
        border: bool = True,
        callback: Callable = None,
        callback_args: list = [],
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
            pass

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
            color (str): Color of the radio button. Defaults to COLORS.white.
            is_checked (bool): Is the radio button checked? Defaults to False.
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
            pass

        elif self._default_cfg:
            self.connect_radio_button(
                name=name, parent=parent, category=category, callback=self.set_category
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
            lambda x: callback(name=name, parent=parent, args=callback_args)
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

        if "knee" in name.lower():
            self._kmode = id
        elif "ankle" in name.lower():
            self._amode = id

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

    def load_default_cfg(self):

        self.add_group(
            name="left",
            parent="root",
            layout="vertical",
            border=True,
        )

        self.add_group(
            name="control_panel",
            parent="root",
            layout="vertical",
            border=False,
            show_title=False,
        )

        self.add_group(
            name="plot",
            parent="left",
            layout="vertical",
            border=True,
            show_title=True,
        )

        self.add_group(
            name="plot_attributes",
            parent="left",
            layout="vertical",
            border=False,
            show_title=False,
        )

        self.add_plot(
            parent="plot",
            color=COLORS.white,
        )

        self.add_group(
            name="joint",
            parent="plot_attributes",
            layout="grid",
            border=True,
            show_title=True,
        )

        self.add_group(
            name="attributes",
            parent="plot_attributes",
            layout="grid",
            border=True,
            show_title=True,
        )

        self.add_radio_button(
            name="knee",
            category="joint",
            parent="joint",
            color=COLORS.white,
            is_checked=True,
            row=0,
            col=0,
        )

        self.add_radio_button(
            name="ankle",
            category="joint",
            parent="joint",
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=1,
        )

        self.add_radio_button(
            name="motor_position",
            category="attributes",
            parent="attributes",
            color=COLORS.white,
            is_checked=True,
            row=0,
            col=0,
        )

        self.add_radio_button(
            name="motor_velocity",
            category="attributes",
            parent="attributes",
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=1,
        )

        self.add_radio_button(
            name="motor_current",
            category="attributes",
            parent="attributes",
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=2,
        )

        self.add_radio_button(
            name="joint_position",
            category="attributes",
            parent="attributes",
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=0,
        )

        self.add_radio_button(
            name="joint_velocity",
            category="attributes",
            parent="attributes",
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=1,
        )

        self.add_radio_button(
            name="joint_torque",
            category="attributes",
            parent="attributes",
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=2,
        )

        self.add_radio_button(
            name="loadcell_Fx",
            category="attributes",
            parent="attributes",
            color=COLORS.white,
            is_checked=False,
            row=2,
            col=0,
        )

        self.add_radio_button(
            name="loadcell_fy",
            category="attributes",
            parent="attributes",
            color=COLORS.white,
            is_checked=False,
            row=2,
            col=1,
        )

        self.add_radio_button(
            name="loadcell_fz",
            category="attributes",
            parent="attributes",
            color=COLORS.white,
            is_checked=False,
            row=2,
            col=2,
        )

        self.add_group(
            name="joint_control",
            parent="control_panel",
            layout="horizontal",
            border=False,
            show_title=False,
        )

        self.add_group(
            name="knee",
            parent="joint_control",
            layout="grid",
            border=True,
            show_title=True,
        )

        self.add_group(
            name="ankle",
            parent="joint_control",
            layout="grid",
            border=True,
            show_title=True,
        )

        self.add_group(
            name="utility",
            parent="control_panel",
            layout="grid",
            border=False,
            show_title=False,
        )

        self.add_group(
            name="estop",
            parent="control_panel",
            layout="grid",
            border=False,
            show_title=False,
        )

        self.add_button(
            name="emergency_stop",
            parent="estop",
            color=COLORS.maize,
            border=True,
        )

        self.add_button(
            name="safety_reset",
            parent="utility",
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.add_group(
            name="calibrate",
            parent="utility",
            layout="grid",
            border=True,
            show_title=True,
            row=0,
            col=1,
        )

        self.add_button(
            name="encoders",
            parent="calibrate",
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.add_button(
            name="loadcell",
            parent="calibrate",
            color=COLORS.white,
            row=1,
            col=0,
        )

        self.add_button(
            name="home",
            parent="knee",
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.add_group(
            name="kdata",
            parent="knee",
            layout="grid",
            border=True,
            show_title=False,
            row=1,
            col=0,
        )

        self.add_text(
            name="Voltage (mV): ",
            parent="kdata",
            row=0,
            col=0,
        )

        self.add_value(
            name="voltage",
            parent="kdata",
            default=0,
            row=0,
            col=1,
        )

        self.add_text(
            name="Position (deg): ",
            parent="kdata",
            row=1,
            col=0,
        )

        self.add_value(
            name="position",
            parent="kdata",
            default=0,
            row=1,
            col=1,
        )

        self.add_text(
            name="Current (mA): ",
            parent="kdata",
            row=2,
            col=0,
        )

        self.add_value(
            name="current",
            parent="kdata",
            default=0,
            row=2,
            col=1,
        )

        self.add_text(
            name=" ",
            parent="kdata",
            row=3,
            col=0,
        )

        self.add_text(
            name="Stiffness (N/rad): ",
            parent="kdata",
            row=4,
            col=0,
        )

        self.add_value(
            name="stiffness",
            parent="kdata",
            default=0,
            row=4,
            col=1,
        )

        self.add_text(
            name="Damping (N/(rad/s)): ",
            parent="kdata",
            row=5,
            col=0,
        )

        self.add_value(
            name="damping",
            parent="kdata",
            default=0,
            row=5,
            col=1,
        )

        self.add_text(
            name="Equilibrium Angle (deg): ",
            parent="kdata",
            row=6,
            col=0,
        )

        self.add_value(
            name="equilibrium_angle",
            parent="kdata",
            default=0,
            row=6,
            col=1,
        )

        self.connect_value(
            name="equilibrium_angle",
            parent="kdata",
            callback=self.set_value,
        )

        self.add_group(
            name="mode",
            parent="knee",
            layout="grid",
            border=True,
            show_title=True,
            row=2,
            col=0,
        )

        self.add_group(
            name="knee_updater",
            parent="knee",
            layout="grid",
            border=False,
            show_title=False,
            row=3,
            col=0,
        )

        self.add_button(
            name="stop",
            parent="knee_updater",
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.add_button(
            name="update",
            parent="knee_updater",
            color=COLORS.white,
            row=0,
            col=1,
        )

        self.add_dropdown(
            name="knee",
            parent="mode",
            options=["Voltage (V)", "Position (P)", "Current (I)", "Impedance (Z)"],
        )

        self.connect_dropdown(name="knee", parent="mode", callback=self.set_joint_mode)

        self.add_button(
            name="home",
            parent="ankle",
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.add_group(
            name="adata",
            parent="ankle",
            layout="grid",
            border=True,
            show_title=False,
            row=1,
            col=0,
        )

        self.add_text(
            name="Voltage (mV): ",
            parent="adata",
            row=0,
            col=0,
        )

        self.add_value(
            name="voltage",
            parent="adata",
            default=0,
            row=0,
            col=1,
        )

        self.add_text(
            name="Position (deg): ",
            parent="adata",
            row=1,
            col=0,
        )

        self.add_value(
            name="position",
            parent="adata",
            default=0,
            row=1,
            col=1,
        )

        self.add_text(
            name="Current (mA): ",
            parent="adata",
            row=2,
            col=0,
        )

        self.add_value(
            name="current",
            parent="adata",
            default=0,
            row=2,
            col=1,
        )

        self.add_text(
            name=" ",
            parent="adata",
            row=3,
            col=0,
        )

        self.add_text(
            name="Stiffness (N/rad): ",
            parent="adata",
            row=4,
            col=0,
        )

        self.add_value(
            name="stiffness",
            parent="adata",
            default=0,
            row=4,
            col=1,
        )

        self.add_text(
            name="Damping (N/(rad/s)): ",
            parent="adata",
            row=5,
            col=0,
        )

        self.add_value(
            name="damping",
            parent="adata",
            default=0,
            row=5,
            col=1,
        )

        self.add_text(
            name="Equilibrium Angle (deg): ",
            parent="adata",
            row=6,
            col=0,
        )

        self.add_value(
            name="equilibrium_angle",
            parent="adata",
            default=0,
            row=6,
            col=1,
        )

        self.add_group(
            name="mode",
            parent="ankle",
            layout="grid",
            border=True,
            show_title=True,
            row=2,
            col=0,
        )

        self.add_group(
            name="aupdater",
            parent="ankle",
            layout="grid",
            border=False,
            show_title=False,
            row=3,
            col=0,
        )

        self.add_button(
            name="stop",
            parent="aupdater",
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.add_button(
            name="update",
            parent="aupdater",
            color=COLORS.white,
            row=0,
            col=1,
        )

        self.add_dropdown(
            name="ankle",
            parent="mode",
            options=["Voltage (V)", "Position (P)", "Current (I)", "Impedance (Z)"],
        )

        self.connect_dropdown(name="ankle", parent="mode", callback=self.set_joint_mode)


if __name__ == "__main__":
    tui = TUI(default_cfg=True)

    tui.run()
