import os
import time

import numpy as np

from opensourceleg.actuators import DephyActpack
from opensourceleg.constants import Constants
from opensourceleg.logger import Logger
from opensourceleg.units import DEFAULT_UNITS, UnitsDefinition


class Joint(DephyActpack):
    def __init__(
        self,
        name: str = "knee",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        gear_ratio: float = 1.0,
        has_loadcell: bool = False,
        logger: Logger = None,
        units: UnitsDefinition = DEFAULT_UNITS,
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:

        super().__init__(
            port=port,
            baud_rate=baud_rate,
            frequency=frequency,
            logger=logger,
            units=units,
            debug_level=debug_level,
            dephy_log=dephy_log,
        )

        self._gear_ratio: float = gear_ratio
        self._is_homed: bool = False
        self._has_loadcell: bool = has_loadcell
        self._encoder_map = None
        self._zero_pos = 0.0

        self._motor_zero_pos = 0.0
        self._joint_zero_pos = 0.0

        self._motor_voltage_sp = 0.0
        self._motor_current_sp = 0.0
        self._motor_position_sp = 0.0

        self._stiffness_sp: int = 200
        self._damping_sp: int = 400
        self._equilibrium_position_sp = 0.0

        self._control_mode_sp: str = "voltage"

        if "knee" in name.lower() or "ankle" in name.lower():
            self._name = name
        else:
            self._log.warning(f"Invalid joint name: {name}")
            return

        if os.path.isfile(f"./{self._name}_encoder_map.npy"):
            coefficients = np.load(f"./{self._name}_encoder_map.npy")
            self._encoder_map = np.polynomial.polynomial.Polynomial(coefficients)
        else:
            self._log.debug(
                f"[{self._name}] No encoder map found. Please run the calibration routine."
            )

    def home(
        self,
        homing_voltage: int = 2000,
        homing_frequency: int = 100,
    ):

        is_homing = True

        CURRENT_THRESHOLD = 6000
        VELOCITY_THRESHOLD = 0.001

        self.set_mode("voltage")
        self.set_voltage(-1 * homing_voltage)  # mV, negative for counterclockwise

        _motor_encoder_array = []
        _joint_encoder_array = []

        time.sleep(0.1)

        try:
            while is_homing:
                self.update()
                time.sleep(1 / homing_frequency)

                _motor_encoder_array.append(self.motor_position)
                _joint_encoder_array.append(self.joint_position)

                if (
                    abs(self.output_velocity) <= VELOCITY_THRESHOLD
                    or abs(self.motor_current) >= CURRENT_THRESHOLD
                ):
                    self.set_voltage(0)
                    is_homing = False

        except KeyboardInterrupt:
            self.set_voltage(0)
            self._log.warning("Homing interrupted.")
            return

        _motor_zero_pos = self.motor_position
        _joint_zero_pos = self.joint_position

        time.sleep(0.1)

        if np.std(_motor_encoder_array) < 1e-6:
            self._log.warning(
                f"[{self._name}] Motor encoder not working. Please check the wiring."
            )
            return

        elif np.std(_joint_encoder_array) < 1e-6:
            self._log.warning(
                f"[{self._name}] Joint encoder not working. Please check the wiring."
            )
            return

        if "ankle" in self._name.lower():
            self._zero_pos = np.deg2rad(30)
            self.set_motor_zero_position(_motor_zero_pos)
            self.set_joint_zero_position(_joint_zero_pos)

        else:
            self._zero_pos = 0.0
            self.set_motor_zero_position(_motor_zero_pos)
            self.set_joint_zero_position(_joint_zero_pos)

        self._is_homed = True

        # if self.encoder_map is None:
        #     if (
        #         input(f"[{self._name}] Would you like to make an encoder map? (y/n): ")
        #         == "y"
        #     ):
        #         self.make_encoder_map()

    def make_encoder_map(self):
        """
        This method makes a lookup table to calculate the position measured by the joint encoder.
        This is necessary because the magnetic output encoders are nonlinear.
        By making the map while the joint is unloaded, joint position calculated by motor position * gear ratio
        should be the same as the true joint position.

        Output from this function is a file containing a_i values parameterizing the map

        Eqn: position = sum from i=0^5 (a_i*counts^i)

        Author: Kevin Best, PhD
                U-M Neurobionics Lab
                Gitub: tkevinbest, https://github.com/tkevinbest
        """

        if not self.is_homed:
            self._log.warning(
                f"[{self._name}] Please home the joint before making the encoder map."
            )
            return

        self.set_mode("current")
        self.set_current_gains()
        time.sleep(0.1)
        self.set_current_gains()

        self.set_output_torque(0.0)
        time.sleep(0.1)
        self.set_output_torque(0.0)

        _joint_position_array = []
        _output_position_array = []

        self._log.info(
            f"[{self._name}] Please manually move the joint numerous times \
                through its full range of motion for 10 seconds.\
                   \n Press any key to continue."
        )

        _start_time = time.time()

        try:
            while time.time() - _start_time < 10:
                self.update()
                _joint_position_array.append(self.joint_position)
                _output_position_array.append(self.output_position)

                time.sleep(1 / self.frequency)

        except KeyboardInterrupt:
            self._log.warning("Encoder map interrupted.")
            return

        self._log.info(f"[{self._name}] You may now stop moving the joint.")

        _power = np.arange(4.0)
        _a_mat = np.array(_joint_position_array).reshape(-1, 1) ** _power
        _beta = np.linalg.lstsq(_a_mat, _output_position_array, rcond=None)[0]
        _coeffs = _beta[0]

        self._encoder_map = np.polynomial.polynomial.Polynomial(_coeffs)

        np.save(f"./{self._name}_encoder_map.npy", _coeffs)
        self._log.info(f"[{self._name}] Encoder map saved.")

    def set_output_torque(self, torque: float):
        """
        Set the output torque of the joint.
        This is the torque that is applied to the joint, not the motor.

        Args:
            torque (float): torque in user-defined units
        """
        self.set_motor_torque(torque / self.gear_ratio)

    def set_output_position(self, position: float):
        """
        Set the output position of the joint.
        This is the desired position of the joint, not the motor.

        Args:
            position (float): position in user-defined units
        """
        self.set_motor_position(position * self.gear_ratio)

    def set_motor_impedance(
        self,
        kp: int = 40,
        ki: int = 400,
        K: float = 0.08922,
        B: float = 0.0038070,
        ff: int = 128,
    ):
        """
        Set the impedance gains of the motor in real units: Nm/rad and Nm/rad/s.

        Args:
            kp (int): Proportional gain. Defaults to 40.
            ki (int): Integral gain. Defaults to 400.
            K (float): Spring constant. Defaults to 0.08922 Nm/rad.
            B (float): Damping constant. Defaults to 0.0038070 Nm/rad/s.
            ff (int): Feedforward gain. Defaults to 128.
        """
        self.set_impedance_gains(
            kp=kp,
            ki=ki,
            K=int(K * Constants.NM_PER_RAD_TO_K),
            B=int(B * Constants.NM_S_PER_RAD_TO_B),
            ff=ff,
        )

    def set_joint_impedance(
        self,
        kp: int = 40,
        ki: int = 400,
        K: float = 0.08922,
        B: float = 0.0038070,
        ff: int = 128,
    ):
        """
        Set the impedance gains of the joint in real units: Nm/rad and Nm/rad/s.

        Conversion:
            K_motor = K_joint / (gear_ratio ** 2)
            B_motor = B_joint / (gear_ratio ** 2)

        Args:
            kp (int): Proportional gain. Defaults to 40.
            ki (int): Integral gain. Defaults to 400.
            K (float): Spring constant. Defaults to 0.08922 Nm/rad.
            B (float): Damping constant. Defaults to 0.0038070 Nm/rad/s.
            ff (int): Feedforward gain. Defaults to 128.
        """
        self.set_motor_impedance(
            kp=kp,
            ki=ki,
            K=K / (self.gear_ratio**2),
            B=B / (self.gear_ratio**2),
            ff=ff,
        )

    def update_set_points(self):
        self.set_mode(self.control_mode_sp)

    @property
    def zero_position(self):
        return self._zero_pos

    @zero_position.setter
    def zero_position(self, value):
        self._zero_pos = value

    @property
    def name(self):
        return self._name

    @property
    def gear_ratio(self):
        return self._gear_ratio

    @property
    def is_homed(self):
        return self._is_homed

    @property
    def encoder_map(self):
        return self._encoder_map

    @property
    def output_position(self):
        return self.motor_position / self.gear_ratio

    @property
    def output_velocity(self):
        return self.motor_velocity / self.gear_ratio

    @property
    def joint_torque(self):
        return self.motor_torque * self.gear_ratio

    @property
    def motor_current_sp(self):
        return self._motor_current_sp

    @property
    def motor_voltage_sp(self):
        return self._motor_voltage_sp

    @property
    def motor_position_sp(self):
        return self._motor_position_sp

    @property
    def stiffness_sp(self):
        return self._stiffness_sp

    @property
    def damping_sp(self):
        return self._damping_sp

    @property
    def equilibirum_position_sp(self):
        return self._equilibrium_position_sp

    @property
    def control_mode_sp(self):
        return self._control_mode_sp
