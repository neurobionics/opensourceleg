import os
import time

import numpy as np

from ..tools.logger import Logger
from .actuators import (
    MAX_CASE_TEMPERATURE,
    NM_PER_RAD_TO_K,
    NM_S_PER_RAD_TO_B,
    RAD_PER_COUNT,
    DephyActpack,
    MockDephyActpack,
)

"""
Module Overview:

This module provides classes for controlling robotic joints using DephyActpack.
It includes the `Joint` and `MockJoint` classes, facilitating joint homing,
encoder mapping, impedance setting, and parameter conversion. The module is designed
for robotic systems, offering both real and mock joint implementations.

Usage Guide:

1. Create a `Joint` instance by providing necessary parameters.
2. Optionally, create an encoder map using the `make_encoder_map` method.
3. Set impedance gains with the `set_joint_impedance` method.
4. Home the joint using the `home` method.
5. Access joint information using properties such as name, gear ratio, and temperature.
6. For testing without hardware, create a `MockJoint` instance.'
"""


class Joint(DephyActpack):
    def __init__(
        self,
        name: str = "knee",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        gear_ratio: float = 41.4999,
        has_loadcell: bool = False,
        logger: Logger = Logger(),
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:

        super().__init__(
            name=name,
            port=port,
            baud_rate=baud_rate,
            frequency=frequency,
            logger=logger,
            debug_level=debug_level,
            dephy_log=dephy_log,
        )

        self._gear_ratio: float = gear_ratio
        self._is_homed: bool = False
        self._has_loadcell: bool = has_loadcell
        self._encoder_map = None

        self._motor_zero_pos = 0.0
        self._joint_zero_pos = 0.0

        self._max_temperature: float = MAX_CASE_TEMPERATURE

        if "knee" in name.lower() or "ankle" in name.lower():
            self._name: str = name
        else:
            self._log.warning(msg=f"[{self.__repr__()}] Invalid joint name: {name}")
            return

        if os.path.isfile(path=f"./{self._name}_encoder_map.npy"):
            coefficients = np.load(file=f"./{self._name}_encoder_map.npy")
            self._encoder_map = np.polynomial.polynomial.Polynomial(coef=coefficients)
        else:
            self._log.debug(
                msg=f"[{self._name}] No encoder map found. Please run the calibration routine."
            )

    def home(
        self,
        homing_voltage: int = 2000,
        homing_frequency: int = 100,
    ) -> None:
        """

        This method homes the joint by moving it to the zero position.
        The zero position is defined as the position where the joint is fully extended.
        This method will also make an encoder map if one does not exist.

        Args:
            homing_voltage (int): voltage in mV to use for homing
            homing_frequency (int): frequency in Hz to use for homing
        """

        is_homing = True

        CURRENT_THRESHOLD = 5000
        VELOCITY_THRESHOLD = 0.001

        self.set_mode(mode=self.control_modes.voltage)
        homing_direction = -1.0

        self.set_voltage(
            value=homing_direction * homing_voltage
        )  # mV, negative for counterclockwise

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
                    self.set_voltage(value=0)
                    is_homing = False

        except KeyboardInterrupt:
            self.set_voltage(value=0)
            self._log.warning(msg="Homing interrupted.")
            return

        _motor_zero_pos = self.motor_encoder_counts
        _joint_zero_pos = self.joint_encoder_counts

        time.sleep(0.1)

        _zero_pos: int = 0
        _zero_pos_joint: int = 0

        if "ankle" in self._name.lower():
            _zero_pos = int((np.deg2rad(30) * self.gear_ratio) / RAD_PER_COUNT)
            _zero_pos_joint = int(np.deg2rad(30) / RAD_PER_COUNT)

        self.set_motor_zero_position(position=(_motor_zero_pos + _zero_pos))
        self.set_joint_zero_position(position=(_joint_zero_pos + _zero_pos_joint))

        self._is_homed = True
        self._log.info(f"[{self._name}] Homing complete.")

    def make_encoder_map(self) -> None:
        """
        This method makes a lookup table to calculate the position measured by the joint encoder.
        This is necessary because the magnetic output encoders are nonlinear.
        By making the map while the joint is unloaded, joint position calculated by motor position * gear ratio
        should be the same as the true joint position.

        Output from this function is a file containing a_i values parameterizing the map

        Eqn: position = sum from i=0^5 (a_i*counts^i)

        Author: Kevin Best
                U-M Locolab | Neurobionics Lab
                Gitub: tkevinbest, https://github.com/tkevinbest
        """

        if not self.is_homed:
            self._log.warning(
                msg=f"[{self.__repr__()}] Please home the joint before making the encoder map."
            )
            return

        self.set_mode(mode=self.control_modes.current)
        self.set_current_gains()
        time.sleep(0.1)
        self.set_current_gains()

        self.set_output_torque(torque=0.0)
        time.sleep(0.1)
        self.set_output_torque(torque=0.0)

        _joint_position_array = []
        _output_position_array = []

        self._log.info(
            msg=f"[{self.__repr__()}] Please manually move the joint numerous times through its full range of motion for 10 seconds. \nPress any key to continue."
        )

        _start_time: float = time.time()

        try:
            while time.time() - _start_time < 10:
                self.update()
                _joint_position_array.append(self.joint_position)
                _output_position_array.append(self.output_position)

                time.sleep(1 / self.frequency)

        except KeyboardInterrupt:
            self._log.warning(msg="Encoder map interrupted.")
            return

        self._log.info(msg=f"[{self.__repr__()}] You may now stop moving the joint.")

        _power = np.arange(4.0)
        _a_mat = np.array(_joint_position_array).reshape(-1, 1) ** _power
        _beta = np.linalg.lstsq(_a_mat, _output_position_array, rcond=None)[0]
        _coeffs = _beta[0]

        self._encoder_map = np.polynomial.polynomial.Polynomial(coef=_coeffs)

        np.save(file=f"./{self._name}_encoder_map.npy", arr=_coeffs)
        self._log.info(msg=f"[{self.__repr__()}] Encoder map saved.")

    def set_max_temperature(self, temperature: float) -> None:
        """
        Set the maximum temperature of the motor.

        Args:
            temperature (float): temperature in degrees Celsius
        """
        self._max_temperature = temperature

    def set_output_torque(self, torque: float) -> None:
        """
        Set the output torque of the joint.
        This is the torque that is applied to the joint, not the motor.

        Args:
            torque (float): torque in N_m
        """
        self.set_motor_torque(torque=torque / self.gear_ratio)

    def set_output_position(self, position: float) -> None:
        """
        Set the output position of the joint.
        This is the desired position of the joint, not the motor.
        This method automatically handles scaling by the gear raito.

        Args:
            position (float): position in radians
        """
        self.set_motor_position(position=position * self.gear_ratio)

    def set_motor_impedance(
        self,
        kp: int = 40,
        ki: int = 400,
        K: float = 0.08922,
        B: float = 0.0038070,
        ff: int = 128,
    ) -> None:
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
            K=int(K * NM_PER_RAD_TO_K),
            B=int(B * NM_S_PER_RAD_TO_B),
            ff=ff,
        )

    def set_joint_impedance(
        self,
        kp: int = 40,
        ki: int = 400,
        K: float = 100.0,
        B: float = 3.0,
        ff: int = 128,
    ) -> None:
        """
        Set the impedance gains of the joint in real units: Nm/rad and Nm/rad/s.
        This sets the impedance at the output and automatically scales based on gear raitos.

        Conversion:
            K_motor = K_joint / (gear_ratio ** 2)
            B_motor = B_joint / (gear_ratio ** 2)

        Args:
            kp (int): Proportional gain. Defaults to 40.
            ki (int): Integral gain. Defaults to 400.
            K (float): Spring constant. Defaults to 100 Nm/rad.
            B (float): Damping constant. Defaults to 3.0 Nm/rad/s.
            ff (int): Feedforward gain. Defaults to 128.
        """
        self.set_motor_impedance(
            kp=kp,
            ki=ki,
            K=K / (self.gear_ratio**2),
            B=B / (self.gear_ratio**2),
            ff=ff,
        )

    def convert_to_joint_impedance(
        self,
        K: float = 100,
        B: float = 40,
    ):
        joint_stiffness = (K / NM_PER_RAD_TO_K) * self.gear_ratio**2
        joint_damping = (B / NM_S_PER_RAD_TO_B) * self.gear_ratio**2

        return joint_stiffness, joint_damping

    def convert_to_motor_impedance(
        self,
        K: float = 100,
        B: float = 40,
    ):
        motor_stiffness = K / NM_PER_RAD_TO_K
        motor_damping = B / NM_S_PER_RAD_TO_B

        return motor_stiffness, motor_damping

    def convert_to_pid_impedance(
        self,
        K: float = 0.08922,
        B: float = 0.0038070,
    ):
        pid_stiffness = (K / self.gear_ratio**2) * NM_PER_RAD_TO_K
        pid_damping = (B / self.gear_ratio**2) * NM_S_PER_RAD_TO_B

        return pid_stiffness, pid_damping

    @property
    def name(self) -> str:
        return self._name

    @property
    def gear_ratio(self) -> float:
        return self._gear_ratio

    @property
    def max_temperature(self) -> float:
        """Max allowed temperature of the actuator case in celsius."""
        return self._max_temperature

    @property
    def is_homed(self) -> bool:
        """Indicates if the homing routine has been called yet."""
        return self._is_homed

    @property
    def encoder_map(self):
        """Polynomial coefficients defining the joint encoder map from counts to radians."""
        return self._encoder_map

    @property
    def output_position(self) -> float:
        """
        Position of the output in radians.
        This is calculated by scaling the motor angle with the gear ratio.
        Note that this method does not consider compliance from an SEA.
        """
        return self.motor_position / self.gear_ratio

    @property
    def output_velocity(self) -> float:
        """
        Velocity of the output in radians.
        This is calculated by scaling the motor angle with the gear ratio.
        Note that this method does not consider compliance from an SEA.
        """
        return self.motor_velocity / self.gear_ratio

    @property
    def joint_torque(self) -> float:
        """
        Torque at the joint output in Nm.
        This is calculated using motor current, k_t, and the gear ratio.
        """
        return self.motor_torque * self.gear_ratio


class MockJoint(Joint, MockDephyActpack):
    """
    Mock Joint class for testing the Joint class\n
    Inherits everything from the Joint class and the MockDephyActpack class
    except for the Joint constructor.
    """

    def __init__(
        self,
        name: str = "knee",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        gear_ratio: float = 41.4999,
        has_loadcell: bool = False,
        logger: Logger = Logger(),
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:

        MockDephyActpack.__init__(self, name, port)
        self._gear_ratio: float = gear_ratio
        self._is_homed: bool = False
        self._has_loadcell: bool = has_loadcell
        self._encoder_map = None

        self._motor_zero_pos = 0.0
        self._joint_zero_pos = 0.0

        self._max_temperature: float = MAX_CASE_TEMPERATURE

        if "knee" in name.lower() or "ankle" in name.lower():
            self._name: str = name
        else:
            self._log.warning(msg=f"[{self.__repr__()}] Invalid joint name: {name}")
            return


if __name__ == "__main__":
    joint = MockJoint()
    print(joint)
