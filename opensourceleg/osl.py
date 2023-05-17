#!/usr/bin/python3
import sys
import time

import numpy as np

sys.path.append("../")

from joints import Joint
from loadcell import Loadcell
from logger import Logger
from state_machine import State, StateMachine
from units import DEFAULT_UNITS, UnitsDefinition
from utilities import SoftRealtimeLoop
import utilities as utilities

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

    def __init__(
        self,
        frequency: int = 200,
        current_limit: float = 8000,
        file_name: str = "./osl.log",
    ) -> None:

        self._frequency: int = frequency

        self._has_knee: bool = False
        self._has_ankle: bool = False
        self._has_loadcell: bool = False
        self._has_tui: bool = False
        self._has_sm: bool = False

        self._current_limit: float = current_limit
        self._set_state_machine_parameters: bool = False

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

        if self.has_state_machine:
            self.state_machine.start()

            if self.has_knee:
                self.knee.set_mode("impedance")
                self.knee.set_impedance_gains()

            if self.has_ankle:
                self.ankle.set_mode("impedance")
                self.ankle.set_impedance_gains()

    def __exit__(self, type, value, tb):
        if self.has_state_machine:
            self.state_machine.stop()

        if self.has_knee:
            self._knee.stop()

        if self.has_ankle:
            self._ankle.stop()

        if self.has_tui:
            if self.tui.is_running:
                self.tui.quit()

    def __repr__(self) -> str:
        return f"OSL object. Frequency: {self._frequency} Hz"

    def add_tui(
        self,
        configuration: str = "state_machine",
        layout: str = "vertical",
    ):
        from tui import TUI

        self._has_tui = True
        self.tui = TUI(
            title="www.opensourceleg.com", frequency=self._frequency, layout=layout
        )

        if configuration == "state_machine":
            if self.has_state_machine:
                self.initialize_sm_tui()
            else:
                self.log.warn(
                    "State machine not initialized. Please initialize the state machine before adding the TUI."
                )

    def add_joint(
        self,
        name: str = "knee",
        port: str = None,
        baud_rate: int = 230400,
        gear_ratio: float = 1.0,
        has_loadcell: bool = False,
        debug_level: int = 0,
        dephy_log: bool = False,
    ):

        if "knee" in name.lower():
            self._has_knee = True

            if port is None:
                if "knee" in name.lower():
                    port = utilities.get_active_ports()[0]
                else:
                    port = utilities.get_active_ports()[1]

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
        self._has_sm = True

    def update(
        self,
    ):
        if self.has_knee:
            self._knee.update()

            if self.knee.motor_current > self.current_limit:
                self.log.warn("[KNEE] Current limit reached. Stopping motor.")
                self.__exit__()
                exit()

        if self.has_ankle:
            self._ankle.update()

            if self.ankle.motor_current > self.current_limit:
                self.log.warn("[ANKLE] Current limit () reached. Stopping motor.")
                self.__exit__()
                exit()

        if self.has_loadcell:
            self._loadcell.update()

        if self.has_state_machine:
            self.state_machine.update()

            if self._set_state_machine_parameters:
                if self.has_knee:
                    self.knee.set_impedance_gains(
                        K=self.state_machine.current_state.stiffness,
                        B=self.state_machine.current_state.damping,
                    )
                    
                    self.knee.set_output_position(
                        self.state_machine.current_state.equilibrium_angle
                    )

                elif self.has_ankle:
                    self.ankle.set_impedance_gains(
                        K=self.state_machine.current_state.stiffness,
                        B=self.state_machine.current_state.damping,
                    )
                    
                    self.ankle.set_output_position(
                        self.state_machine.current_state.equilibrium_angle
                    )

    def run(self, set_state_machine_parameters: bool = False):

        self._set_state_machine_parameters = set_state_machine_parameters
        self.update()

        time.sleep(0.1)

        if not self.has_tui:
            self.update()

        else:
            self.tui.add_update_callback(self.update)
            self.tui.run()

    def initialize_sm_tui(self):

        from tui import COLORS

        self.tui.add_panel(
            name="top",
            layout="horizontal",
            border=False,
        )

        self.tui.add_panel(
            name="bottom",
            layout="horizontal",
            border=False,
        )

        self.tui.add_panel(
            name="fz",
            parent="top",
            show_title=True,
        )

        self.tui.add_panel(
            name="joint_position",
            parent="top",
            show_title=True,
        )

        self.tui.add_plot(
            name="fz_plot",
            parent="fz",
            object=self.loadcell,
            attribute="fz",
            color=COLORS.cyan,
        )

        self.tui.add_plot(
            name="joint_position_plot",
            parent="joint_position",
            object=self.knee,
            attribute="output_position",
            color=COLORS.yellow,
        )

        self.tui.add_state_visualizer(
            name="state_machine",
            parent="bottom",
            states=self.state_machine.states,
            object=self.state_machine,
            attribute="current_state_name",
            layout="horizontal",
        )

        self.tui.add_button(
            name="emergency_stop",
            parent="bottom",
            callback=self.estop,
            color=COLORS.maize,
            border=True,
        )

    def estop(self, **kwargs):
        self.log.debug("[OSL] Emergency stop activated.")

        if self.has_tui:
            self.tui.quit()

    def home(self):
        if self.has_knee:
            self.log.debug("[OSL] Homing knee joint.")
            self.knee.home()

        if self.has_ankle:
            self.log.debug("[OSL] Homing ankle joint.")
            self.ankle.home()

    def calibrate_loadcell(self):
        self.log.debug("[OSL] Calibrating loadcell.")
        if self.has_loadcell:
            self.loadcell.reset()

    def calibrate_encoders(self):
        self.log.debug("[OSL] Calibrating encoders.")

        if self.has_knee:
            self.knee.make_encoder_map()

        if self.has_ankle:
            self.ankle.make_encoder_map()

    def reset(self):
        self.log.debug("[OSL] Resetting OSL.")

        if self.has_knee:
            self.knee.set_mode("voltage")
            self.knee.set_voltage(0, force=True)

            time.sleep(0.1)

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
    def current_limit(self):
        return self._current_limit

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
    def has_state_machine(self):
        return self._has_sm

    @property
    def loadcell(self):
        return self._loadcell

    @property
    def has_tui(self):
        return self._has_tui


if __name__ == "__main__":
    osl = OpenSourceLeg(frequency=200)

    osl.units["position"] = "deg"

    osl.add_joint(
        name="knee",
        gear_ratio=41.61,
    )


    # osl.add_state_machine()
    # osl.add_tui()

    # with osl:
    #     osl.run(set_state_machine_parameters=True)
