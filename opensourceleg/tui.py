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


class TUI:
    """
    A class used to represent a TUI
    """

    def __init__(self, title: str = "Open-source Leg", frequency: int = 30):
        self.colors = Colors()
        self.root = ttk.TTk()
        self.timer = ttk.TTkTimer()
        self.dt = 1.0 / frequency
        self.layout = ttk.TTkGridLayout(columnMinHeight=1, columnMinWidth=1)

        self.root.setLayout(self.layout)

        self.frame = ttk.TTkFrame(
            parent=self.root,
            border=True,
            title=title,
            titleColor=ttk.TTkColor.BOLD + ttk.TTkColor.fg("#00FFFF"),
        )
        self.frame.setLayout(ttk.TTkHBoxLayout())

        self.left_frame = ttk.TTkFrame(
            parent=self.frame, border=True, borderColor=ttk.TTkColor.fg("#00FFFF")
        )
        self.left_frame.setLayout(ttk.TTkVBoxLayout())
        self.right_frame = ttk.TTkFrame(parent=self.frame, border=True)
        self.right_frame.setLayout(ttk.TTkVBoxLayout())

        self.status_frame = ttk.TTkFrame(
            border=True, title="Status", titleColor=ttk.TTkColor.fg("#00ff00")
        )
        self.buttons_frame = ttk.TTkFrame(border=False)
        self.attributes_frame = ttk.TTkFrame(border=False)
        self.graph_frame = ttk.TTkFrame(
            border=True, titleColor=ttk.TTkColor.fg("#ffffff")
        )

        self.status_frame.setLayout(
            ttk.TTkGridLayout(columnMinHeight=2, columnMinWidth=2)
        )
        self.buttons_frame.setLayout(
            ttk.TTkGridLayout(columnMinHeight=2, columnMinWidth=2)
        )
        self.attributes_frame.setLayout(
            ttk.TTkGridLayout(columnMinHeight=2, columnMinWidth=2)
        )
        self.graph_frame.setLayout(
            ttk.TTkGridLayout(columnMinHeight=2, columnMinWidth=2)
        )

        self._buttons_row = 0
        self._buttons_col = 0
        self._buttons_max_row = 3
        self._buttons_max_col = 3

        self._button_groups = {}
        self._button_group_rows = {}
        self._button_group_cols = {}

        self._attributes_row = 0
        self._attributes_col = 0
        self._attributes_max_row = 3
        self._attributes_max_col = 3

        self._active_attribute: str = "[Motor] Position"

        self._attribute_groups = {}
        self._attribute_group_rows = {}
        self._attribute_group_cols = {}

        self._status_row = 0
        self._status_col = 0
        self._status_max_row = 3
        self._status_max_col = 3

        self._status_group_rows = {}
        self._status_group_cols = {}

        self.graph = ttk.TTkGraph(
            color=ttk.TTkColor.fg(
                "#00dddd", modifier=ttk.TTkColorGradient(increment=25)
            )
        )
        self._graph_value = 0.0
        self._graph_counter = 0

        self.left_frame.layout().addWidget(self.graph_frame)
        self.left_frame.layout().addWidget(self.attributes_frame)
        self.right_frame.layout().addWidget(self.status_frame)
        self.right_frame.layout().addWidget(self.buttons_frame)

        self.graph_frame.addWidget(self.graph)
        self.timer.timeout.connect(self.update)
        self.timer.start(1)

    @ttk.pyTTkSlot()
    def update(self):
        self._graph_value = [np.sin(self._graph_counter * np.pi / 40)]
        self.graph.addValue(self._graph_value)
        self.graph_frame.setTitle(
            ttk.TTkString(f"{self._active_attribute}: {self._graph_value[0]:0.3f}")
        )

        self._graph_counter += 1
        self.timer.start(self.dt)

    def run(self):
        self.root.mainloop()

    def set_max_columns(self, buttons: int = 3, attributes: int = 3, status: int = 3):
        self._buttons_max_col = buttons
        self._attributes_max_col = attributes
        self._status_max_col = status

    def add_button(
        self,
        name: str = "Button",
        group: str = "Buttons",
        color: str = "#ffffff",
        border: bool = True,
        show_group: bool = False,
    ):

        if group not in self._button_groups:
            self._button_groups[group] = ttk.TTkFrame(
                border=True,
                titleColor=ttk.TTkColor.BOLD,
                borderColor=ttk.TTkColor.fg(color),
            )

            if show_group:
                self._button_groups[group].setTitle(ttk.TTkString(group))

            self._button_groups[group].setLayout(
                ttk.TTkGridLayout(columnMinHeight=2, columnMinWidth=2)
            )

            self._button_group_rows[group] = 0
            self._button_group_cols[group] = 0

            self.buttons_frame.layout().addWidget(
                self._button_groups[group],
                self._buttons_row,
                self._buttons_col,
            )

            self._buttons_row += 1

            if self._buttons_row >= self._buttons_max_row:
                self._buttons_row = 0
                self._buttons_col += 1

        self._button_groups[group].layout().addWidget(
            a := ttk.TTkButton(
                text=name, color=ttk.TTkColor.fg(self.colors.white), border=border
            ),
            self._button_group_rows[group],
            self._button_group_cols[group],
        )

        self._button_group_cols[group] += 1

        if self._button_group_cols[group] >= self._buttons_max_col:
            self._button_group_cols[group] = 0
            self._button_group_rows[group] += 1

    def add_attribute(
        self,
        name: str = "Attribute",
        group: str = "Attributes",
        is_checked: bool = False,
    ):

        if group not in self._attribute_groups:
            self._attribute_groups[group] = ttk.TTkFrame(
                border=True, title=group, titleColor=ttk.TTkColor.BOLD
            )
            self._attribute_groups[group].setLayout(
                ttk.TTkGridLayout(columnMinHeight=1, columnMinWidth=1)
            )

            self._attribute_group_rows[group] = 0
            self._attribute_group_cols[group] = 0

            self.attributes_frame.layout().addWidget(
                self._attribute_groups[group],
                self._attributes_row,
                self._attributes_col,
            )

            self._attributes_row += 1

            if self._attributes_row >= self._attributes_max_row:
                self._attributes_row = 0
                self._attributes_col += 1

        self._attribute_groups[group].layout().addWidget(
            ttk.TTkRadioButton(text=" " + name, name=group, checked=is_checked),
            self._attribute_group_rows[group],
            self._attribute_group_cols[group],
        )

        self._attribute_group_cols[group] += 1

        if self._attribute_group_cols[group] >= self._attributes_max_col:
            self._attribute_group_cols[group] = 0
            self._attribute_group_rows[group] += 1

    def add_status(self, name: str = "Status", is_checked: bool = False):
        self.status_frame.layout().addWidget(
            ttk.TTkCheckbox(text=" " + name, checked=is_checked),
            self._status_row,
            self._status_col,
        )

        self._status_col += 1

        if self._status_col >= self._status_max_col:
            self._status_col = 0
            self._status_row += 1


def add_attributes(tui):

    tui.add_attribute(
        name="Knee",
        group="Joint",
        is_checked=True,
    )

    tui.add_attribute(
        name="Ankle",
        group="Joint",
        is_checked=False,
    )

    tui.add_attribute(
        name="[Motor] Position",
        group="Attributes",
        is_checked=True,
    )

    tui.add_attribute(
        name="[Motor] Velocity",
        group="Attributes",
        is_checked=False,
    )

    tui.add_attribute(
        name="[Motor] Current",
        group="Attributes",
        is_checked=False,
    )

    tui.add_attribute(
        name="[Joint] Position",
        group="Attributes",
        is_checked=False,
    )

    tui.add_attribute(
        name="[Joint] Velocity",
        group="Attributes",
        is_checked=False,
    )

    tui.add_attribute(
        name="[Joint] Torque",
        group="Attributes",
        is_checked=False,
    )

    tui.add_attribute(
        name="[Loadcell] Fx",
        group="Attributes",
        is_checked=False,
    )

    tui.add_attribute(
        name="[Loadcell] Fy",
        group="Attributes",
        is_checked=False,
    )

    tui.add_attribute(
        name="[Loadcell] Fz",
        group="Attributes",
        is_checked=False,
    )


if __name__ == "__main__":
    tui = TUI()

    add_attributes(tui)

    tui.add_button("Disable Knee", group="Utility", show_group=False)
    tui.add_button("Disable Ankle", group="Utility")
    tui.add_button("Safety Reset", group="Utility")
    tui.add_button("Home Knee", group="Utility")
    tui.add_button("Home Ankle", group="Utility")
    tui.add_button("Encoders", group="Calibrate", show_group=True)
    tui.add_button("Loadcell", group="Calibrate")
    tui.add_button("E-STOP", group="E-STOP", color=tui.colors.red)

    tui.run()
