#!/usr/bin/python3
import sys
import time

import numpy as np

sys.path.append("../")

import opensourceleg.constants as constants
import opensourceleg.utilities as utilities
from opensourceleg.joints import Joint
from opensourceleg.loadcell import Loadcell
from opensourceleg.logger import Logger
from opensourceleg.state_machine import State, StateMachine
from opensourceleg.units import DEFAULT_UNITS, UnitsDefinition
from opensourceleg.utilities import SoftRealtimeLoop


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
        file_name: str = "osl",
    ) -> None:
        """
        Initialize the OSL class.

        Parameters
        ----------
        frequency : int, optional
            The frequency of the control loop, by default 200
        file_name : str, optional
            The name of the log file, by default "./osl.log"
        """

        self._frequency: int = frequency

        self._has_knee: bool = False
        self._has_ankle: bool = False
        self._has_loadcell: bool = False
        self._has_tui: bool = False
        self._has_sm: bool = False

        self._is_sm_running: bool = False
        self._is_homed: bool = False

        self._knee: Joint = None  # type: ignore
        self._ankle: Joint = None  # type: ignore
        self._loadcell: Loadcell = None  # type: ignore

        self._log_data: bool = False

        self.log = Logger(
            file_path=file_name, log_format="[%(asctime)s] %(levelname)s: %(message)s"
        )

        self.clock = SoftRealtimeLoop(dt=1.0 / self._frequency, report=False, fade=0.1)
        self._units: UnitsDefinition = DEFAULT_UNITS

        self.tui = None
        self.state_machine = None

        self._timestamp: float = time.time()

    def __enter__(self) -> None:

        if self.has_knee:
            self._knee.start()

        if self.has_ankle:
            self._ankle.start()

        if self.has_loadcell:
            self._loadcell.initialize()

    def __exit__(self, type, value, tb) -> None:
        if self.has_state_machine and self.is_sm_running:
            self.state_machine.stop()  # type: ignore

        if self.has_knee:
            self._knee.stop()

        if self.has_ankle:
            self._ankle.stop()

        if self.has_tui:
            if self.tui.is_running:  # type: ignore
                self.tui.quit()  # type: ignore

    def __repr__(self) -> str:
        return f"OSL object. Frequency: {self._frequency} Hz"

    def add_tui(
        self,
        configuration: str = "state_machine",
        layout: str = "vertical",
    ) -> None:
        """
        Add a Terminal User Interface (TUI) to the OSL object. The TUI is used
        to visualize the data from the OSL and to send commands to the OSL. Additionally,
        the TUI also has a timer built-in to govern the frequency of the control loop, which is
        less accurate than the "osl.clock" (SoftRealTimeLoop) object but is more convenient.

        Note
        ----
        The timer/interface runs in a separate thread, so it does not affect the control loop.
        This causes the code to not respond to keyboard interrupts (Ctrl+C) when the TUI is running.

        Parameters
        ----------
        configuration : str, optional
            The configuration of the TUI, by default "state_machine"
        layout : str, optional
            The layout of the TUI, by default "vertical"
        """
        from opensourceleg.tui import TUI

        self._has_tui = True
        self.tui = TUI(
            title="www.opensourceleg.com", frequency=self._frequency, layout=layout
        )

        if configuration == "state_machine":
            if self.has_state_machine:
                self.initialize_sm_tui()
            else:
                self.log.warn(
                    msg="State machine not initialized. Please initialize the state machine before adding the TUI."
                )

    def add_joint(
        self,
        name: str = "knee",
        port: str = None,  # type: ignore
        baud_rate: int = 230400,
        gear_ratio: float = 1.0,
        has_loadcell: bool = False,
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:
        """
        Add a joint to the OSL object.

        Parameters
        ----------
        name : str, optional
            The name of the joint, by default "knee"
        port : str, optional
            The serial port that the joint is connected to, by default None. If
            None, the first active port will be used.
        baud_rate : int, optional
            The baud rate of the serial communication, by default 230400
        gear_ratio : float, optional
            The gear ratio of the joint, by default 1.0
        has_loadcell : bool, optional
            Whether the joint has a loadcell, by default False
        debug_level : int, optional
            The debug level of the joint, by default 0
        dephy_log : bool, optional
            Whether to log the joint data to the dephy log, by default False
        """
        if port is None:
            ports = utilities.get_active_ports()

            if len(ports) == 0:
                self.log.warn(
                    msg="No active ports found, please ensure that the joint is connected and powered on."
                )

                exit()

            elif len(ports) == 1:
                port = ports[-1]

            else:
                port = ports[-1]
                port_a = ports[-2]

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

            if self.has_knee:
                port = port_a

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
            self.log.warning(msg="[OSL] Joint name is not recognized.")

    def add_loadcell(
        self,
        dephy_mode: bool = False,
        joint: Joint = None,  # type: ignore
        amp_gain: float = 125.0,
        exc: float = 5.0,
        loadcell_matrix=constants.LOADCELL_MATRIX,
    ) -> None:
        """
        Add a loadcell to the OSL object.

        Parameters
        ----------
        dephy_mode : bool, optional
            Whether the loadcell is in dephy mode ie. connected to the dephy actpack with an FFC cable, by default False
        joint : Joint, optional
            The joint that the loadcell is attached to, by default None
        amp_gain : float, optional
            The amplifier gain of the loadcell, by default 125.0
        exc : float, optional
            The excitation voltage of the loadcell, by default 5.0
        loadcell_matrix : np.ndarray, optional
            The loadcell calibration matrix, by default None
        """
        self._has_loadcell = True
        self._loadcell = Loadcell(
            dephy_mode=dephy_mode,
            joint=joint,
            amp_gain=amp_gain,
            exc=exc,
            loadcell_matrix=loadcell_matrix,  # type: ignore
            logger=self.log,
        )

    def add_state_machine(
        self,
        spoof: bool = False,
    ) -> None:
        """
        Add a state machine to the OSL object.

        Parameters
        ----------
        spoof : bool, optional
            If True, the state machine will spoof the state transitions--ie, it will not
            check the criteria for transitioning but will instead transition after the
            minimum time spent in state has elapsed. This is useful for testing.
            Defaults to False.
        """
        self.state_machine = StateMachine(osl=self, spoof=spoof)
        self._has_sm = True

    def update(
        self,
        set_state_machine_parameters: bool = False,
    ) -> None:
        if self.has_knee:
            self._knee.update()

            if self.knee.case_temperature > self.knee.max_temperature:  # type: ignore
                self.log.warn(
                    msg="[KNEE] Thermal limit {self.knee.max_temperature} reached. Stopping motor."
                )
                self.__exit__(type=None, value=None, tb=None)
                exit()

        if self.has_ankle:
            self._ankle.update()

            if self.ankle.case_temperature > self.ankle.max_temperature:  # type: ignore
                self.log.warn(
                    msg="[ANKLE] Thermal limit {self.ankle.max_temperature} reached. Stopping motor."
                )
                self.__exit__(type=None, value=None, tb=None)
                exit()

        if self.has_loadcell:
            self._loadcell.update()

        if self._log_data:
            self.log.data()

        if self.has_state_machine:

            if self.is_homed and not self.is_sm_running:
                self.state_machine.start()
                self._is_sm_running = True

            if self.is_sm_running:
                self.state_machine.update()  # type: ignore

            if set_state_machine_parameters:
                if self.has_knee and self.state_machine.current_state.is_knee_active:  # type: ignore

                    if self.knee.mode != self.knee.modes["impedance"]:  # type: ignore
                        self.knee.set_mode(mode="impedance")  # type: ignore
                        self.knee.set_impedance_gains()  # type: ignore

                    self.knee.set_joint_impedance(  # type: ignore
                        K=self.state_machine.current_state.knee_stiffness,  # type: ignore
                        B=self.state_machine.current_state.knee_damping,  # type: ignore
                    )

                    self.knee.set_output_position(  # type: ignore
                        position=self.state_machine.current_state.knee_theta  # type: ignore
                    )

                if self.has_ankle and self.state_machine.current_state.is_ankle_active:  # type: ignore

                    if self.ankle.mode != self.ankle.modes["impedance"]:  # type: ignore
                        self.ankle.set_mode(mode="impedance")  # type: ignore
                        self.ankle.set_impedance_gains()  # type: ignore

                    self.ankle.set_joint_impedance(  # type: ignore
                        K=self.state_machine.current_state.ankle_stiffness,  # type: ignore
                        B=self.state_machine.current_state.ankle_damping,  # type: ignore
                    )

                    self.ankle.set_output_position(  # type: ignore
                        position=self.state_machine.current_state.ankle_theta  # type: ignore
                    )

            self._timestamp = time.time()

    def run(
        self, set_state_machine_parameters: bool = False, log_data: bool = False
    ) -> None:
        """
        Run the OpenSourceLeg instance: update the joints, loadcell, and state machine.
        If the instance has a TUI, run the TUI.
        If the instance has a state machine and if set_state_machine_parameters is True,
        set the joint impedance gains and equilibrium angles to the current state's values.

        Parameters
        ----------
        set_state_machine_parameters : bool, optional
            Whether to set the joint impedance gains and equilibrium angles to the current state's values, by default False
        """

        if self.is_homed:
            if not self.is_sm_running:
                self.state_machine.start()
                self._is_sm_running = True
        else:
            osl.log.warn(
                f"[OSL] Please run the homing routine by calling `osl.home()` before starting the state-machine."
            )
            exit()

        if log_data:
            self._log_data = True

        self.update(set_state_machine_parameters=set_state_machine_parameters)

        time.sleep(0.1)

        if not self.has_tui:
            self.update(set_state_machine_parameters=set_state_machine_parameters)

        else:
            self.tui.add_update_callback(callback=self.update, args=set_state_machine_parameters)  # type: ignore
            self.tui.run()  # type: ignore

    def initialize_sm_tui(self) -> None:

        from opensourceleg.tui import COLORS

        assert self.tui is not None
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
            name="knee_joint_position",
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
            parent="knee_joint_position",
            object=self.knee,
            attribute="output_position",
            color=COLORS.yellow,
        )

        self.tui.add_state_visualizer(
            name="state_machine",
            parent="bottom",
            states=self.state_machine.states,  # type: ignore
            object=self.state_machine,
            attribute="current_state_name",
            layout="horizontal",
        )

        self.tui.add_button(
            name="emergency_stop",
            parent="bottom",
            callback=self.estop,
            color=COLORS.red,
            border=True,
        )

    def estop(self, **kwargs):
        self.log.debug("[OSL] Emergency stop activated.")

        if self.has_tui:
            self.tui.quit()  # type: ignore

    def home(self) -> None:
        if self.has_knee:
            self.log.info(msg="[OSL] Homing knee joint.")
            self.knee.home()  # type: ignore

        if self.has_ankle:
            self.log.info(msg="[OSL] Homing ankle joint.")
            self.ankle.home()  # type: ignore

        self._is_homed = True

    def calibrate_loadcell(self) -> None:
        self.log.debug(msg="[OSL] Calibrating loadcell.")
        if self.has_loadcell:
            self.loadcell.reset()  # type: ignore

    def calibrate_encoders(self) -> None:
        self.log.debug(msg="[OSL] Calibrating encoders.")

        if self.has_knee:
            self.knee.make_encoder_map()  # type: ignore

        if self.has_ankle:
            self.ankle.make_encoder_map()  # type: ignore

    def reset(self) -> None:
        self.log.debug(msg="[OSL] Resetting OSL.")

        if self.has_knee:
            self.knee.set_mode(mode="voltage")  # type: ignore
            self.knee.set_voltage(value=0, force=True)  # type: ignore

            time.sleep(0.1)

        if self.has_ankle:
            self.ankle.set_mode(mode="voltage")  # type: ignore
            self.ankle.set_voltage(value=0, force=True)  # type: ignore

            time.sleep(0.1)

    def log_data(self):
        self._log_data = True

    @property
    def timestamp(self) -> float:
        return self._timestamp

    @property
    def knee(self):
        if self.has_knee:
            return self._knee
        else:
            self.log.warning(msg="[OSL] Knee is not connected.")

    @property
    def ankle(self):
        if self.has_ankle:
            return self._ankle
        else:
            self.log.warning(msg="[OSL] Ankle is not connected.")

    @property
    def loadcell(self):
        if self.has_loadcell:
            return self._loadcell
        else:
            self.log.warning(msg="[OSL] Loadcell is not connected.")

    @property
    def units(self) -> UnitsDefinition:
        return self._units

    @property
    def has_knee(self) -> bool:
        return self._has_knee

    @property
    def has_ankle(self) -> bool:
        return self._has_ankle

    @property
    def has_loadcell(self) -> bool:
        return self._has_loadcell

    @property
    def has_state_machine(self) -> bool:
        return self._has_sm

    @property
    def has_tui(self) -> bool:
        return self._has_tui

    @property
    def is_homed(self) -> bool:
        return self._is_homed

    @property
    def is_sm_running(self) -> bool:
        return self._is_sm_running


if __name__ == "__main__":
    osl = OpenSourceLeg(frequency=200)

    osl.units["position"] = "deg"  # type: ignore

    osl.add_joint(
        name="knee",
        gear_ratio=41.61,
    )
