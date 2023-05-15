#!/usr/bin/python3
import sys
import time

import numpy as np

sys.path.append("../")

from opensourceleg.joints import Joint
from opensourceleg.loadcell import Loadcell
from opensourceleg.logger import Logger
from opensourceleg.state_machine import StateMachine
from opensourceleg.units import DEFAULT_UNITS, UnitsDefinition
from opensourceleg.utilities import SoftRealtimeLoop

CURRENT_LIMIT = 8000


class OpenSourceLeg:
    """
    OSL class: This class is the main class for the Open Source Leg project. It
    contains all the necessary functions to control the leg.

    Returns:
        none: none
    """

    # This is a singleton class
    _instance = None

    @staticmethod
    def get_instance():
        if OpenSourceLeg._instance is None:
            OpenSourceLeg()
        return OpenSourceLeg._instance

    def __init__(self, frequency: int = 200, file_name: str = "./osl.log") -> None:

        self._frequency: int = frequency

        self._has_knee: bool = False
        self._has_ankle: bool = False
        self._has_loadcell: bool = False
        self._has_tui: bool = False

        self._knee: Joint = None
        self._ankle: Joint = None
        self._loadcell: Loadcell = None

        self.log = Logger(
            file_path=file_name, log_format="[%(asctime)s] %(levelname)s: %(message)s"
        )

        self.clock = SoftRealtimeLoop(dt=1.0 / self._frequency, report=False, fade=0.1)

        self._units: UnitsDefinition = DEFAULT_UNITS

        self.tui = None
        self.state_machine = None

    def __enter__(self):

        if self.has_knee:
            self._knee.start()

        if self.has_ankle:
            self._ankle.start()

        if self.has_loadcell:
            self._loadcell.initialize()

    def __exit__(self, type, value, tb):
        if self.has_knee:
            self._knee.stop()

        if self.has_ankle:
            self._ankle.stop()

        if self.has_tui:
            if self.tui.is_running:
                self.tui.quit()

    def __repr__(self) -> str:
        return f"OSL object with 0 joints"

    def get_attribute(self, object, attribute: str):
        return getattr(object, attribute)

    def add_tui(
        self,
        frequency: int = 30,
    ):
        from opensourceleg.tui import TUI

        self._has_tui = True
        self.tui = TUI(frequency=frequency)

        if self.has_loadcell:
            self.tui.set_loadcell(self._loadcell)

        self.initialize_tui()

    def add_joint(
        self,
        name: str = "knee",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        gear_ratio: float = 1.0,
        has_loadcell: bool = False,
        debug_level: int = 0,
        dephy_log: bool = False,
    ):

        if "knee" in name.lower():
            self._has_knee = True
            self._knee = Joint(
                name=name,
                port=port,
                baud_rate=baud_rate,
                frequency=self._frequency,
                gear_ratio=gear_ratio,
                has_loadcell=has_loadcell,
                logger=self.log,
                units=self.units,
                debug_level=debug_level,
                dephy_log=dephy_log,
            )

        elif "ankle" in name.lower():
            self._has_ankle = True
            self._ankle = Joint(
                name=name,
                port=port,
                baud_rate=baud_rate,
                frequency=self._frequency,
                gear_ratio=gear_ratio,
                has_loadcell=has_loadcell,
                logger=self.log,
                units=self.units,
                debug_level=debug_level,
                dephy_log=dephy_log,
            )
        else:
            self.log.warning("[OSL] Joint name is not recognized.")

    def add_loadcell(
        self,
        dephy_mode: bool = False,
        joint: Joint = None,
        amp_gain: float = 125.0,
        exc: float = 5.0,
        loadcell_matrix=None,
    ):
        self._has_loadcell = True
        self._loadcell = Loadcell(
            dephy_mode=dephy_mode,
            joint=joint,
            amp_gain=amp_gain,
            exc=exc,
            loadcell_matrix=loadcell_matrix,
            logger=self.log,
        )

    def add_state_machine(
        self,
    ):
        self.state_machine = StateMachine(osl=self)

    def update(
        self,
        current_limit: int = CURRENT_LIMIT,
    ):
        if self.has_knee:
            self._knee.update()

            if self.knee.motor_current > current_limit:
                self.log.warn("[KNEE] Current limit reached. Stopping motor.")
                self.__exit__()
                exit()

        if self.has_ankle:
            self._ankle.update()

            if self.ankle.motor_current > current_limit:
                self.log.warn("[ANKLE] Current limit () reached. Stopping motor.")
                self.__exit__()
                exit()

        if self.has_loadcell:
            self._loadcell.update()

    def run_tui(self):
        if self.has_knee:
            self.tui.add_knee(self.knee)

            if self.tui.joint is None:
                self.tui.set_active_joint(name="knee", parent="joint")

        if self.has_ankle:
            self.tui.add_ankle(self.ankle)

            if self.tui.joint is None:
                self.tui.set_active_joint(name="ankle", parent="joint")

        self.tui.add_update_callback(self.update)
        self.tui.run()

    def initialize_tui(self):

        from opensourceleg.tui import COLORS

        self.tui.add_group(
            name="left",
            parent="root",
            layout="vertical",
            border=True,
        )

        self.tui.add_group(
            name="control_panel",
            parent="root",
            layout="vertical",
            border=False,
            show_title=False,
        )

        self.tui.add_group(
            name="plot",
            parent="left",
            layout="vertical",
            border=True,
            show_title=True,
        )

        self.tui.add_group(
            name="plot_attributes",
            parent="left",
            layout="vertical",
            border=False,
            show_title=False,
        )

        self.tui.add_plot(
            parent="plot",
            color=COLORS.white,
        )

        self.tui.add_group(
            name="joint",
            parent="plot_attributes",
            layout="grid",
            border=True,
            show_title=True,
        )

        self.tui.add_group(
            name="attributes",
            parent="plot_attributes",
            layout="grid",
            border=True,
            show_title=True,
        )

        # ------------ RADIO BUTTONS ------------ #

        self.tui.add_radio_button(
            name="knee",
            category="joint",
            parent="joint",
            callback=self.tui.set_active_joint,
            color=COLORS.white,
            is_checked=True,
            row=0,
            col=0,
        )

        self.tui.add_radio_button(
            name="ankle",
            category="joint",
            parent="joint",
            callback=self.tui.set_active_joint,
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=1,
        )

        self.tui.add_radio_button(
            name="motor_position",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=True,
            row=0,
            col=0,
        )

        self.tui.add_radio_button(
            name="motor_velocity",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=1,
        )

        self.tui.add_radio_button(
            name="motor_current",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=0,
            col=2,
        )

        self.tui.add_radio_button(
            name="joint_position",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=0,
        )

        self.tui.add_radio_button(
            name="joint_velocity",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=1,
        )

        self.tui.add_radio_button(
            name="joint_torque",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=1,
            col=2,
        )

        self.tui.add_radio_button(
            name="loadcell_Fx",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=2,
            col=0,
        )

        self.tui.add_radio_button(
            name="loadcell_fy",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=2,
            col=1,
        )

        self.tui.add_radio_button(
            name="loadcell_fz",
            category="attributes",
            parent="attributes",
            callback=self.set_tui_attribute,
            color=COLORS.white,
            is_checked=False,
            row=2,
            col=2,
        )

        # ------------ CONTROL PANEL ------------ #

        self.tui.add_group(
            name="joint_control",
            parent="control_panel",
            layout="horizontal",
            border=False,
            show_title=False,
        )

        self.tui.add_group(
            name="knee",
            parent="joint_control",
            layout="grid",
            border=True,
            show_title=True,
        )

        self.tui.add_group(
            name="ankle",
            parent="joint_control",
            layout="grid",
            border=True,
            show_title=True,
        )

        self.tui.add_group(
            name="utility",
            parent="control_panel",
            layout="grid",
            border=False,
            show_title=False,
        )

        self.tui.add_group(
            name="estop",
            parent="control_panel",
            layout="grid",
            border=False,
            show_title=False,
        )

        # ------------ UTILITY BUTTONS ------------ #

        self.tui.add_button(
            name="emergency_stop",
            parent="estop",
            callback=self.estop,
            color=COLORS.maize,
            border=True,
        )

        self.tui.add_button(
            name="safety_reset",
            parent="utility",
            callback=self.reset,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_group(
            name="calibrate",
            parent="utility",
            layout="grid",
            border=True,
            show_title=True,
            row=0,
            col=1,
        )

        self.tui.add_button(
            name="encoders",
            parent="calibrate",
            # callback=self.calibrate_encoders,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_button(
            name="loadcell",
            parent="calibrate",
            # callback=self.calibrate_loadcell,
            color=COLORS.white,
            row=1,
            col=0,
        )

        # ------------ KNEE BUTTONS ------------ #

        self.tui.add_button(
            name="home",
            parent="knee",
            callback=self.home,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_group(
            name="knee_data",
            parent="knee",
            layout="grid",
            border=True,
            show_title=False,
            row=1,
            col=0,
        )

        # ------------ KNEE VALUES ------------ #

        self.tui.add_text(
            name="Voltage (mV): ",
            parent="knee_data",
            row=0,
            col=0,
        )

        self.tui.add_value(
            name="voltage",
            parent="knee_data",
            default=0,
            callback=self.set_motor_voltage_sp,
            row=0,
            col=1,
        )

        self.tui.add_text(
            name="Position (deg): ",
            parent="knee_data",
            row=1,
            col=0,
        )

        self.tui.add_value(
            name="position",
            parent="knee_data",
            default=0,
            callback=self.set_motor_position_sp,
            row=1,
            col=1,
        )

        self.tui.add_text(
            name="Current (mA): ",
            parent="knee_data",
            row=2,
            col=0,
        )

        self.tui.add_value(
            name="current",
            parent="knee_data",
            default=0,
            callback=self.set_motor_current_sp,
            row=2,
            col=1,
        )

        self.tui.add_text(
            name=" ",
            parent="knee_data",
            row=3,
            col=0,
        )

        self.tui.add_text(
            name="Stiffness: ",
            parent="knee_data",
            row=4,
            col=0,
        )

        self.tui.add_value(
            name="stiffness",
            parent="knee_data",
            default=200,
            callback=self.set_stiffness_sp,
            row=4,
            col=1,
        )

        self.tui.add_text(
            name="Damping: ",
            parent="knee_data",
            row=5,
            col=0,
        )

        self.tui.add_value(
            name="damping",
            parent="knee_data",
            default=400,
            callback=self.set_damping_sp,
            row=5,
            col=1,
        )

        self.tui.add_text(
            name="Equilibrium Position (deg): ",
            parent="knee_data",
            row=6,
            col=0,
        )

        self.tui.add_value(
            name="equilibrium_position",
            parent="knee_data",
            default=0,
            callback=self.set_equilibrium_position_sp,
            row=6,
            col=1,
        )

        self.tui.add_group(
            name="mode",
            parent="knee",
            layout="grid",
            border=True,
            show_title=True,
            row=2,
            col=0,
        )

        self.tui.add_text(
            name=" ",
            parent="mode",
            row=2,
            col=0,
        )

        self.tui.add_text(
            name=" Updates only the setpoints that are ",
            parent="mode",
            color=COLORS.turquoise,
            row=3,
            col=0,
        )

        self.tui.add_text(
            name=" relevant to the selected control mode.",
            parent="mode",
            color=COLORS.turquoise,
            row=4,
            col=0,
        )

        self.tui.add_group(
            name="kupdater",
            parent="knee",
            layout="grid",
            border=False,
            show_title=False,
            row=3,
            col=0,
        )

        self.tui.add_button(
            name="stop",
            parent="kupdater",
            callback=self.stop_joint,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_button(
            name="update",
            parent="kupdater",
            callback=self.update_joint,
            color=COLORS.white,
            row=0,
            col=1,
        )

        # ------------ KNEE DROPDOWN ------------ #

        self.tui.add_dropdown(
            name="knee",
            parent="mode",
            options=["Voltage (V)", "Position (P)", "Current (I)", "Impedance (Z)"],
            callback=self.set_control_mode_sp,
            row=1,
            col=0,
        )

        # ------------ ANKLE BUTTONS ------------ #

        self.tui.add_button(
            name="home",
            parent="ankle",
            callback=self.home,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_group(
            name="ankle_data",
            parent="ankle",
            layout="grid",
            border=True,
            show_title=False,
            row=1,
            col=0,
        )

        # ------------ ANKLE VALUES ------------ #

        self.tui.add_text(
            name="Voltage (mV): ",
            parent="ankle_data",
            row=0,
            col=0,
        )

        self.tui.add_value(
            name="voltage",
            parent="ankle_data",
            default=0,
            callback=self.set_motor_voltage_sp,
            row=0,
            col=1,
        )

        self.tui.add_text(
            name="Position (deg): ",
            parent="ankle_data",
            row=1,
            col=0,
        )

        self.tui.add_value(
            name="position",
            parent="ankle_data",
            default=0,
            callback=self.set_motor_position_sp,
            row=1,
            col=1,
        )

        self.tui.add_text(
            name="Current (mA): ",
            parent="ankle_data",
            row=2,
            col=0,
        )

        self.tui.add_value(
            name="current",
            parent="ankle_data",
            default=0,
            callback=self.set_motor_current_sp,
            row=2,
            col=1,
        )

        self.tui.add_text(
            name=" ",
            parent="ankle_data",
            row=3,
            col=0,
        )

        self.tui.add_text(
            name="Stiffness: ",
            parent="ankle_data",
            row=4,
            col=0,
        )

        self.tui.add_value(
            name="stiffness",
            parent="ankle_data",
            default=200,
            callback=self.set_stiffness_sp,
            row=4,
            col=1,
        )

        self.tui.add_text(
            name="Damping: ",
            parent="ankle_data",
            row=5,
            col=0,
        )

        self.tui.add_value(
            name="damping",
            parent="ankle_data",
            default=400,
            callback=self.set_damping_sp,
            row=5,
            col=1,
        )

        self.tui.add_text(
            name="Equilibrium Position (deg): ",
            parent="ankle_data",
            row=6,
            col=0,
        )

        self.tui.add_value(
            name="equilibrium_position",
            parent="ankle_data",
            default=0,
            callback=self.set_equilibrium_position_sp,
            row=6,
            col=1,
        )

        self.tui.add_group(
            name="mode",
            parent="ankle",
            layout="grid",
            border=True,
            show_title=True,
            row=2,
            col=0,
        )

        self.tui.add_text(
            name=" ",
            parent="mode",
            row=2,
            col=0,
        )

        self.tui.add_text(
            name=" Updates only the setpoints that are ",
            parent="mode",
            color=COLORS.turquoise,
            row=3,
            col=0,
        )

        self.tui.add_text(
            name=" relevant to the selected control mode.",
            parent="mode",
            color=COLORS.turquoise,
            row=4,
            col=0,
        )

        self.tui.add_group(
            name="aupdater",
            parent="ankle",
            layout="grid",
            border=False,
            show_title=False,
            row=3,
            col=0,
        )

        self.tui.add_button(
            name="stop",
            parent="aupdater",
            callback=self.stop_joint,
            color=COLORS.white,
            row=0,
            col=0,
        )

        self.tui.add_button(
            name="update",
            parent="aupdater",
            callback=self.update_joint,
            color=COLORS.white,
            row=0,
            col=1,
        )

        # ------------ ANKLE DROPDOWN ------------ #

        self.tui.add_dropdown(
            name="ankle",
            parent="mode",
            options=["Voltage (V)", "Position (P)", "Current (I)", "Impedance (Z)"],
            callback=self.set_control_mode_sp,
        )

    def set_motor_current_sp(self, **kwargs):
        self.log.debug("[OSL] Setting motor current setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        value = int(self.tui.values[parent + "_" + name].text())

        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._motor_current_sp = value
            self.log.debug("[OSL] Setting knee motor current setpoint to %s." % value)

        elif "ankle" in name and self.has_ankle:
            self.ankle._motor_current_sp = value

    def set_motor_position_sp(self, **kwargs):
        self.log.debug("[OSL] Setting motor position setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        _value = int(self.tui.values[parent + "_" + name].text())

        value = np.deg2rad(_value)
        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._motor_position_sp = value
            self.log.debug("[OSL] Setting knee motor position setpoint to %s." % value)

        elif "ankle" in name and self.has_ankle:
            self.ankle._motor_position_sp = value

    def set_motor_voltage_sp(self, **kwargs):
        self.log.debug("[OSL] Setting motor voltage setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        value = int(self.tui.values[parent + "_" + name].text())

        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._motor_voltage_sp = value
            self.log.debug("[OSL] Setting knee motor voltage setpoint to %s." % value)

        elif "ankle" in name and self.has_ankle:
            self.ankle._motor_voltage_sp = value

    def set_stiffness_sp(self, **kwargs):
        self.log.debug("[OSL] Setting stiffness setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        value = int(self.tui.values[parent + "_" + name].text())

        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._stiffness_sp = value
            self.log.debug("[OSL] Setting knee stiffness setpoint to %s." % value)

        elif "ankle" in name and self.has_ankle:
            self.ankle._stiffness_sp = value

    def set_damping_sp(self, **kwargs):
        self.log.debug("[OSL] Setting damping setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        value = int(self.tui.values[parent + "_" + name].text())

        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._damping_sp = value
            self.log.debug("[OSL] Setting knee damping setpoint to %s." % value)

        elif "ankle" in name and self.has_ankle:
            self.ankle._damping_sp = value

    def set_equilibrium_position_sp(self, **kwargs):
        self.log.debug("[OSL] Setting equilibrium position setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        _value = int(self.tui.values[parent + "_" + name].text())

        value = np.deg2rad(_value)

        _name = parent + name

        if "knee" in _name and self.has_knee:
            self.knee._equilibrium_position_sp = value
            self.log.debug(
                "[OSL] Setting knee equilibrium position setpoint to %s." % value
            )

        elif "ankle" in name and self.has_ankle:
            self.ankle._equilibrium_position_sp = value

    def set_control_mode_sp(self, **kwargs):
        self.log.debug("[OSL] Setting control mode setpoint.")

        name = kwargs["name"]
        parent = kwargs["parent"]
        _mode_id = self.tui.dropdowns[parent + "_" + name].currentIndex()
        _name = (parent + name).lower()

        if "knee" in _name and self.has_knee:
            self.knee._control_mode_sp = self.tui.control_modes[_mode_id]
            self.log.debug(
                "[OSL] Setting knee control mode setpoint to %s."
                % self.tui.control_modes[_mode_id]
            )

        elif "ankle" in _name and self.has_ankle:
            self.ankle._control_mode_sp = self.tui.control_modes[_mode_id]
            self.log.debug(
                "[OSL] Setting ankle control mode setpoint to %s."
                % self.tui.control_modes[_mode_id]
            )

    def estop(self, **kwargs):
        self.log.debug("[OSL] Emergency stop activated.")
        self.tui.quit()

    def set_tui_attribute(self, **kwargs):
        name = kwargs["name"]
        self.tui.set_active_attribute(name)

    def home(self, **kwargs):
        name = kwargs["name"]
        parent = kwargs["parent"]

        _name = (parent + name).lower()

        if "knee" in _name:
            self.log.debug("[OSL] Homing knee joint.")
            if self.has_knee:
                self.knee.home()

        elif "ankle" in _name:
            self.log.debug("[OSL] Homing ankle joint.")
            if self.has_ankle:
                self.ankle.home()

    def calibrate_loadcell(self, **kwargs):
        self.log.debug("[OSL] Calibrating loadcell.")
        if self.has_loadcell:
            self.loadcell.reset()

    def calibrate_encoders(self, **kwargs):
        self.log.debug("[OSL] Calibrating encoders.")

        if self.has_knee:
            self.knee.make_encoder_map()

        if self.has_ankle:
            self.ankle.make_encoder_map()

    def reset(self, **kwargs):
        self.log.debug("[OSL] Resetting OSL.")

        if self.has_knee:
            self.knee.set_mode("voltage")
            self.knee.set_voltage(0, force=True)

            time.sleep(0.1)

        if self.has_ankle:
            self.ankle.set_mode("voltage")
            self.ankle.set_voltage(0, force=True)

            time.sleep(0.1)

    def update_joint(self, **kwargs):
        name = kwargs["name"]
        parent = kwargs["parent"]

        _name = (parent + name).lower()

        if "kupdater" in _name:
            self.log.debug("[OSL] Updating knee joint.")

            if self.has_knee:

                if self.knee.control_mode_sp == "voltage":
                    self.knee.set_mode(self.knee.control_mode_sp)
                    self.knee.set_voltage(self.knee.motor_voltage_sp, force=True)

                elif self.knee.control_mode_sp == "current":
                    self.knee.set_mode(self.knee.control_mode_sp)
                    self.knee.set_current_gains()
                    self.knee.set_current(self.knee.motor_current_sp, force=True)

                elif self.knee.control_mode_sp == "position":
                    self.knee.set_mode(self.knee.control_mode_sp)
                    self.knee.set_position_gains()
                    self.knee.set_output_position(self.knee.motor_position_sp)

                else:
                    self.knee.set_mode(self.knee.control_mode_sp)

                    self.knee.set_impedance_gains(
                        K=self.knee.stiffness_sp,
                        B=self.knee.damping_sp,
                    )

                    self.knee.set_output_position(self.knee.equilibirum_position_sp)

        elif "aupdater" in _name:
            self.log.debug("[OSL] Updating ankle joint.")

            if self.has_ankle:

                if self.ankle.control_mode_sp == "voltage":
                    self.ankle.set_mode(self.ankle.control_mode_sp)
                    self.ankle.set_voltage(self.ankle.motor_voltage_sp, force=True)

                elif self.ankle.control_mode_sp == "current":
                    self.ankle.set_mode(self.ankle.control_mode_sp)
                    self.ankle.set_current_gains()
                    self.ankle.set_current(self.ankle.motor_current_sp, force=True)

                elif self.ankle.control_mode_sp == "position":
                    self.ankle.set_mode(self.ankle.control_mode_sp)
                    self.ankle.set_position_gains()
                    self.ankle.set_output_position(self.ankle.motor_position_sp)

                else:
                    self.ankle.set_mode(self.ankle.control_mode_sp)

                    self.ankle.set_impedance_gains(
                        K=self.ankle.stiffness_sp,
                        B=self.ankle.damping_sp,
                    )

                    self.ankle.set_output_position(self.ankle.equilibirum_position_sp)

    def stop_joint(self, **kwargs):
        name = kwargs["name"]
        parent = kwargs["parent"]

        _name = (parent + name).lower()

        if "kupdater" in _name:
            self.log.debug("[OSL] Stopping knee joint.")
            if self.has_knee:
                self.knee.set_mode("voltage")
                self.knee.set_voltage(0, force=True)

                time.sleep(0.1)

        elif "aupdater" in _name:
            self.log.debug("[OSL] Stopping ankle joint.")
            if self.has_ankle:
                self.ankle.set_mode("voltage")
                self.ankle.set_voltage(0, force=True)

                time.sleep(0.1)

    @property
    def knee(self):
        if self.has_knee:
            return self._knee
        else:
            self.log.warning("[OSL] Knee is not connected.")

    @property
    def ankle(self):
        if self.has_ankle:
            return self._ankle
        else:
            self.log.warning("[OSL] Ankle is not connected.")

    @property
    def loadcell(self):
        if self.has_loadcell:
            return self._loadcell
        else:
            self.log.warning("[OSL] Loadcell is not connected.")

    @property
    def units(self):
        return self._units

    @property
    def has_knee(self):
        return self._has_knee

    @property
    def has_ankle(self):
        return self._has_ankle

    @property
    def has_loadcell(self):
        return self._has_loadcell

    @property
    def loadcell(self):
        return self._loadcell

    @property
    def has_tui(self):
        return self._has_tui


if __name__ == "__main__":
    osl = OpenSourceLeg(frequency=200)

    osl.add_joint(
        name="knee",
        port="/dev/ttyACM0",
        baud_rate=230400,
        gear_ratio=41.4999,
    )

    osl.add_loadcell(
        dephy_mode=False,
    )

    osl.units["position"] = "deg"
    osl.log.info(f"Units: {osl.units}")

    with osl:
        for t in osl.clock:
            osl.loadcell.update()
            osl.log.info(f"[OSL] Loadcell: {osl.loadcell.fz}")
