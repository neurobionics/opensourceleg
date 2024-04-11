#!/usr/bin/python3
import sys
import time

import numpy as np

sys.path.append("../")

from .hardware.joints import Joint, MockJoint
from .hardware.sensors import Loadcell, MockLoadcell
from .tools import utilities
from .tools.logger import Logger
from .tools.utilities import SoftRealtimeLoop


class OpenSourceLeg:
    """
    This is a singleton class that represents the Open-Source Leg. It contains various methods and attributes to
    control the Open-Source Leg.

    Parameters:
        file_name (str): Name of the file to be used for logging. Defaults to osl.
        frequency (int): Frequency of the Open-Source Leg. This is the frequency at which the control loop runs
        and the actuators are updated. Defaults to 100.

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

        self._is_homed: bool = False

        self._knee: Joint = None  # type: ignore
        self._ankle: Joint = None  # type: ignore
        self._loadcell: Loadcell = None  # type: ignore

        self.log = Logger(
            file_path=file_name, log_format="[%(asctime)s] %(levelname)s: %(message)s"
        )

        self.clock = SoftRealtimeLoop(dt=1.0 / self._frequency, report=False, fade=0.1)

        self._timestamp: float = time.time()

    def __enter__(self) -> None:

        if self.has_knee:
            self._knee.start()

        if self.has_ankle:
            self._ankle.start()

        if self.has_loadcell:
            self._loadcell.initialize()

    def __exit__(self, type=None, value=None, tb=None) -> None:

        if self.has_knee:
            self._knee.stop()

        if self.has_ankle:
            self._ankle.stop()

    def __repr__(self) -> str:
        return f"OSL"

    def add_joint(
        self,
        name: str = "knee",
        firmware_version: str = "7.2.0",
        port: str = None,
        baud_rate: int = 230400,
        gear_ratio: float = 1.0,
        has_loadcell: bool = False,
        debug_level: int = 0,
        dephy_log: bool = False,
        offline_mode: bool = False,
    ) -> None:
        """
        Adds a knee or ankle joint to the OSL.

        Parameters:
            name (str): Name of the joint to be added. Defaults to knee.
            firmware_version (str): Current version of the actpack being used. Defaults to 7.2.0
            port (str): Port to which the actpack is connected. Defaults to None.
            baud_rate (int): Baud rate of the actpack. Defaults to 230400.
            gear_ratio (float): Gear ratio of the joint. Defaults to 1.0.
            has_loadcell (bool): Is the load cell conneced to the corresponding actpack? If True, the load cell data will be read from the
                                 actpack and not from the RPi's GPIO pins. Defaults to False
            debug_level (int): Debug level to be used for the actpack's logging routine. Defaults to 0.
            dephy_log (bool): If True, Dephy Actpack's logging routine will be enabled in addition to the default opensourceleg's logging
                              routine. Defaults to False.
            offline_mode (bool): If True, the actpack will be in offline mode. This is useful for testing the OSL without connecting the
                                 actpacks. Defaults to False.
        """
        if offline_mode:
            if "knee" in name.lower():
                self._knee = MockJoint(
                    name=name,
                    firmware_version=firmware_version,
                    port=port if port is not None else "",
                    baud_rate=baud_rate,
                    frequency=self._frequency,
                    gear_ratio=gear_ratio,
                    has_loadcell=has_loadcell,
                    logger=self.log,
                    debug_level=debug_level,
                    dephy_log=dephy_log,
                )
                self._has_knee = True

            elif "ankle" in name.lower():
                self._ankle = MockJoint(
                    name=name,
                    firmware_version=firmware_version,
                    port=port if port is not None else "",
                    baud_rate=baud_rate,
                    frequency=self._frequency,
                    gear_ratio=gear_ratio,
                    has_loadcell=has_loadcell,
                    logger=self.log,
                    debug_level=debug_level,
                    dephy_log=dephy_log,
                )
                self._has_ankle = True

            else:
                self.log.warning(msg="[OSL] Joint name is not recognized.")

        else:
            if port is None:
                ports = utilities.get_active_ports()

                port_1: str = ""
                port_2: str = ""

                if len(ports) == 0:
                    self.log.warning(
                        msg="No active ports found, please ensure that the motor is connected and powered on."
                    )

                    exit()

                elif len(ports) == 1:
                    port_1 = ports[-1]

                else:
                    port_1 = ports[-1]
                    port_2 = ports[-2]

                if "knee" in name.lower():
                    if self.has_ankle:
                        if self.ankle.port == port_1:
                            port = port_2
                        else:
                            port = port_1
                    else:
                        port = port_1

                    self._knee = Joint(
                        name=name,
                        firmware_version=firmware_version,
                        port=port,
                        baud_rate=baud_rate,
                        frequency=self._frequency,
                        gear_ratio=gear_ratio,
                        has_loadcell=has_loadcell,
                        logger=self.log,
                        debug_level=debug_level,
                        dephy_log=dephy_log,
                    )
                    self._has_knee = True

                elif "ankle" in name.lower():
                    if self.has_knee:
                        if self.knee.port == port_1:
                            port = port_2
                        else:
                            port = port_1
                    else:
                        port = port_1

                    self._ankle = Joint(
                        name=name,
                        firmware_version=firmware_version,
                        port=port,
                        baud_rate=baud_rate,
                        frequency=self._frequency,
                        gear_ratio=gear_ratio,
                        has_loadcell=has_loadcell,
                        logger=self.log,
                        debug_level=debug_level,
                        dephy_log=dephy_log,
                    )
                    self._has_ankle = True
                else:
                    self.log.warning(msg="[OSL] Joint name is not recognized.")

            else:

                if "knee" in name.lower():
                    if self.has_ankle:
                        if self.ankle.port == port:
                            self.log.warning(
                                msg="[OSL] Knee and Ankle joints cant have the same port. Please specify a different port for the knee joint."
                            )

                            exit()

                    self._knee = Joint(
                        name=name,
                        firmware_version=firmware_version,
                        port=port,
                        baud_rate=baud_rate,
                        frequency=self._frequency,
                        gear_ratio=gear_ratio,
                        has_loadcell=has_loadcell,
                        logger=self.log,
                        debug_level=debug_level,
                        dephy_log=dephy_log,
                    )
                    self._has_knee = True

                elif "ankle" in name.lower():
                    if self.has_knee:
                        if self.knee.port == port:
                            self.log.warning(
                                msg="[OSL] Knee and Ankle joints cant have the same port. Please specify a different port for the ankle joint."
                            )

                            exit()

                    self._ankle = Joint(
                        name=name,
                        firmware_version=firmware_version,
                        port=port,
                        baud_rate=baud_rate,
                        frequency=self._frequency,
                        gear_ratio=gear_ratio,
                        has_loadcell=has_loadcell,
                        logger=self.log,
                        debug_level=debug_level,
                        dephy_log=dephy_log,
                    )
                    self._has_ankle = True

                else:
                    self.log.warning(msg="[OSL] Joint name is not recognized.")

    def add_loadcell(
        self,
        dephy_mode: bool = False,
        joint: Joint = None,
        amp_gain: float = 125.0,
        exc: float = 5.0,
        loadcell_matrix=None,
        offline_mode: bool = False,
    ) -> None:
        """
        Adds a load cell to the OSL.

        Parameters:
            dephy_mode (bool): Is the load cell connected to the actpack? If True, the load cell data will be read from the
                               actpack and not from the RPi's GPIO pins. Defaults to False.
            joint (Joint): Joint to which the load cell is connected. Defaults to None.
            amp_gain (float): Amplifier gain of the load cell strain amplifier. efaults to 125.0.
            exc (float): Excitation voltage of the load cell strain amplifier. Defaults to 5.0.
            loadcell_matrix (np.ndarray): Calibration matrix for the load cell. This matrix is used to convert the
                                                load cell's raw data to forces and moments. Defaults to None.
            offline_mode (bool): If True, the load cell will be in offline mode. This is useful for testing the OSL without
                                 connecting the load cell. Defaults to False.
        """

        if loadcell_matrix is None:
            self.log.warning(
                msg="[OSL] Loadcell matrix is not specified. Please provide the loadcell calibration matrix to the add_loadcell method."
            )

            exit()

        else:
            if offline_mode:
                self._loadcell = MockLoadcell(
                    dephy_mode=dephy_mode,
                    joint=joint,
                    amp_gain=amp_gain,
                    exc=exc,
                    loadcell_matrix=loadcell_matrix,
                    logger=self.log,
                )
            else:
                self._loadcell = Loadcell(
                    dephy_mode=dephy_mode,
                    joint=joint,
                    amp_gain=amp_gain,
                    exc=exc,
                    loadcell_matrix=loadcell_matrix,
                    logger=self.log,
                )

            self._has_loadcell = True

    def update(
        self,
        log_data: bool = False,
    ) -> None:
        """
        Updates the joints and load cells connected to the OSL. It also checks if any of the joints are
        overheating, and if so, it will safely exit the program.

        Parameters:
            log_data (bool): If True, all the variables configured with osl.logger will be automatically
                             logged when the update method is called. Defaults to False.


        """
        if self.has_knee:
            self._knee.update()

            if self.knee.case_temperature > self.knee.max_temperature:
                self.log.warning(
                    msg=f"[KNEE] Thermal limit {self.knee.max_temperature} reached. Stopping motor."
                )
                self.__exit__()
                exit()

        if self.has_ankle:
            self._ankle.update()

            if self.ankle.case_temperature > self.ankle.max_temperature:
                self.log.warning(
                    msg=f"[ANKLE] Thermal limit {self.ankle.max_temperature} reached. Stopping motor."
                )
                self.__exit__()
                exit()

        if self.has_loadcell:
            self._loadcell.update()

        if log_data:
            self.log.data()

        self._timestamp = time.time()

    def home(self) -> None:
        """
        Initiates the homing routine for all the joints connected to the OSL. This routine homes the knee joint first and then the ankle joint regardless of the order in which the joints were added to the OSL.
        """
        if self.has_knee:
            self.log.info(msg="[OSL] Homing knee joint.")
            self.knee.home()

        if self.has_ankle:
            self.log.info(msg="[OSL] Homing ankle joint.")
            self.ankle.home()

        self._is_homed = True

    def calibrate_loadcell(self) -> None:
        """
        Resets the load cell's zero offset.

        Parameters:
            None
        """
        self.log.debug(msg="[OSL] Calibrating loadcell.")
        if self.has_loadcell:
            self.loadcell.reset()

    def calibrate_encoders(self) -> None:
        """
        Makes an encoder map for all the connected joints.

        Parameters:
            None
        """
        self.log.debug(msg="[OSL] Calibrating encoders.")

        if self.has_knee:
            self.knee.make_encoder_map()

        if self.has_ankle:
            self.ankle.make_encoder_map()

    def reset(self) -> None:
        """
        Resets the OSL. This method will stop the OSL, transition all the joints to voltage mode, and set the voltage to 0mV.

        Parameters:
            None
        """
        self.log.debug(msg="[OSL] Resetting OSL.")

        if self.has_knee:
            self.knee.set_mode(mode=self.knee.control_modes.voltage)
            self.knee.set_voltage(value=0)

            time.sleep(0.1)

        if self.has_ankle:
            self.ankle.set_mode(mode=self.ankle.control_modes.voltage)
            self.ankle.set_voltage(value=0)

            time.sleep(0.1)

    @property
    def timestamp(self) -> float:
        """timestamp (float): Timestamp of the last update."""
        return self._timestamp

    @property
    def knee(self):
        """knee (Joint): Knee joint connected to the OSL. Returns None if the knee joint is not added to the OSL."""
        if self.has_knee:
            return self._knee
        else:
            self.log.warning(msg="[OSL] Knee is not connected.")

    @property
    def ankle(self):
        """ankle (Joint): Ankle joint connected to the OSL. Returns None if the ankle joint is not added to the OSL."""
        if self.has_ankle:
            return self._ankle
        else:
            self.log.warning(msg="[OSL] Ankle is not connected.")

    @property
    def loadcell(self):
        """loadcell (float): Loadcell connected to the OSL."""
        if self.has_loadcell:
            return self._loadcell
        else:
            self.log.warning(msg="[OSL] Loadcell is not connected.")

    @property
    def has_knee(self) -> bool:
        """has_ankle (bool): True if the OSL has an ankle joint."""
        return self._has_knee

    @property
    def has_ankle(self) -> bool:
        """has_ankle (bool): True if the OSL has an ankle joint."""
        return self._has_ankle

    @property
    def has_loadcell(self) -> bool:
        """has_loadcell (bool): True if the OSL has a load cell."""
        return self._has_loadcell

    @property
    def is_homed(self) -> bool:
        """is_homed (bool): True if all the joints connected to the OSL are homed."""
        return self._is_homed


if __name__ == "__main__":
    osl = OpenSourceLeg(frequency=200)

    osl.add_joint(
        name="knee",
        gear_ratio=41.61,
        offline_mode=True,
    )
