#!/usr/bin/python3
from typing import Any, Callable, Dict, List, Optional

import logging
import os
import sys
import threading
import time
import traceback
from enum import Enum
from logging.handlers import RotatingFileHandler
from math import isfinite
from ntpath import join
from socket import setdefaulttimeout
from time import strftime

import numpy as np
import scipy.signal
from flexsea import flexsea as flex
from flexsea import fxEnums as fxe
from flexsea import fxUtils as fxu

# TODO: Support for TMotor driver with similar structure


class Actpack:
    """A class that contains and works with all the sensor data from a Dephy Actpack.

    Attributes:
        motor_angle (double) : Motor angle
        motor_velocity (double : Motor velocity

    """

    MOTOR_COUNT_PER_REV = 16384
    NM_PER_AMP = 0.146
    RAD_PER_COUNT = 2 * np.pi / MOTOR_COUNT_PER_REV
    RAD_PER_DEG = np.pi / 180
    MOTOR_COUNT_TO_RADIANS = lambda x: x * (np.pi / 180.0 / 45.5111)
    RADIANS_TO_MOTOR_COUNTS = lambda q: q * (180 * 45.5111 / np.pi)

    RAD_PER_SEC_GYROLSB = np.pi / 180 / 32.8
    G_PER_ACCLSB = 1.0 / 8192

    def __init__(
        self,
        fxs,
        port,
        baud_rate,
        frequency,
        logger: logging.Logger = None,
        debug_level=0,
    ) -> None:
        """
        Initializes the Actpack class

        Args:
            fxs (Fle): _description_
            port (_type_): _description_
            baud_rate (_type_): _description_
            has_loadcell (bool, optional): _description_. Defaults to False.
            debug_level (int, optional): _description_. Defaults to 0.
        """

        self._fxs = fxs
        self._dev_id = None
        self._app_type = None

        self._port = port
        self._baud_rate = baud_rate
        self._debug_level = debug_level

        self._frequency = frequency

        self._streaming = False
        self._data = None

        self.log = logger
        self._start()

    def update(self) -> None:
        """
        Updates/Reads data from the Actpack
        """
        if self.is_streaming:
            self._data = self._fxs.read_device(self._dev_id)

    def _start(self):
        try:
            self._dev_id = self._fxs.open(
                self._port, self._baud_rate, self._debug_level
            )
            self._app_type = self._fxs.get_app_type(self._dev_id)

        except OSError:
            self.log.error("Actpack is not powered ON.")
            sys.exit()

    def _start_streaming_data(self, log_en=False):
        """Starts streaming dataa

        Args:
            frequency (int, optional): _description_. Defaults to 500.
            log_en (bool, optional): _description_. Defaults to False.
        """

        self._fxs.start_streaming(self._dev_id, freq=self._frequency, log_en=log_en)
        self._streaming = True
        time.sleep(1 / self._frequency)

    def _stop_streaming_data(self):
        """
        Stops streaming data
        """
        if self.is_streaming:
            self._fxs.stop_streaming(self._dev_id)
            self._streaming = False
            time.sleep(1 / self._frequency)

    def _shutdown(self):
        """
        Shuts down the Actpack
        """
        self._stop_streaming_data()
        self._fxs.send_motor_command(self._dev_id, fxe.FX_NONE, 0)
        time.sleep(1 / self._frequency)
        self._fxs.close(self._dev_id)
        time.sleep(1 / self._frequency)

    def _set_position_gains(self, kp: int = 100, ki: int = 0, kd: int = 0):
        """Sets Position Gains

        Args:
            kp (int, optional): _description_. Defaults to 200.
            ki (int, optional): _description_. Defaults to 50.
            kd (int, optional): _description_. Defaults to 0.
        """
        assert isfinite(kp) and kp >= 0 and kp <= 1000
        assert isfinite(ki) and ki >= 0 and ki <= 1000
        assert isfinite(kd) and kd >= 0 and kd <= 1000

        self._set_qaxis_voltage(0)
        self._fxs.set_gains(self._dev_id, kp, ki, kd, 0, 0, 0)
        self._set_motor_angle_counts(self.motor_angle)

    def _set_current_gains(self, kp: int = 40, ki: int = 400, ff: int = 128):
        """Sets Current Gains

        Args:
            kp (int, optional): _description_. Defaults to 40.
            ki (int, optional): _description_. Defaults to 400.
            ff (int, optional): _description_. Defaults to 128.
        """
        assert isfinite(kp) and kp >= 0 and kp <= 80
        assert isfinite(ki) and ki >= 0 and ki <= 800
        assert isfinite(ff) and ff >= 0 and ff <= 128

        self._fxs.set_gains(self._dev_id, kp, ki, 0, 0, 0, ff)
        time.sleep(1 / self._frequency)

    def _set_impedance_gains(
        self, K: int = 300, B: int = 1600, kp: int = 40, ki: int = 400, ff: int = 128
    ):
        """Sets Impedance Gains

        Args:
            kp (int, optional): _description_. Defaults to 40.
            ki (int, optional): _description_. Defaults to 400.
            K (int, optional): _description_. Defaults to 300.
            B (int, optional): _description_. Defaults to 1600.
            ff (int, optional): _description_. Defaults to 128.
        """
        assert isfinite(kp) and kp >= 0 and kp <= 80
        assert isfinite(ki) and ki >= 0 and ki <= 800
        assert isfinite(ff) and ff >= 0 and ff <= 128
        assert isfinite(K) and K >= 0
        assert isfinite(B) and B >= 0

        self._set_qaxis_voltage(0)
        self._fxs.set_gains(self._dev_id, kp, ki, 0, K, B, ff)
        self._set_motor_angle_counts(self.motor_angle)

    def _set_motor_angle_counts(self, position: int = 0):
        """Sets Motor Angle

        Args:
            position (int, optional): Angular position in motor counts. Defaults to 0.
        """
        assert isfinite(position)
        self._fxs.send_motor_command(self._dev_id, fxe.FX_POSITION, position)

    def _set_motor_angle_radians(self, theta: float = 0.0):
        assert isfinite(theta)
        self._fxs.send_motor_command(
            self._dev_id, fxe.FX_POSITION, int(theta / self.RAD_PER_COUNT)
        )

    def _set_equilibrium_angle(self, theta: int = 0):
        assert isfinite(theta)
        self._fxs.send_motor_command(self._dev_id, fxe.FX_IMPEDANCE, theta)

    def _set_voltage(self, volt: int = 0):
        """Sets Q-Axis Voltage

        Args:
            volt (int, optional): Voltage in mV. Defaults to 0. Maximum limit is set to 36V.
        """
        assert isfinite(volt) and abs(volt) <= 36000
        self._fxs.send_motor_command(self._dev_id, fxe.FX_VOLTAGE, volt)

    def _set_qaxis_voltage(self, volt: int = 0):
        """Sets Q-Axis Voltage

        Args:
            volt (int, optional): Voltage in mV. Defaults to 0. Maximum limit is set to 36V.
        """
        assert isfinite(volt) and abs(volt) <= 36000
        self._fxs.send_motor_command(self._dev_id, fxe.FX_NONE, volt)

    def _set_qaxis_current(self, current: int = 0):
        """Sets Q-Axis Current

        Args:
            current (int, optional): Current in mA. Defaults to 0. Maximum limit is set to 22A.
        """
        assert isfinite(current) and abs(current) <= 22000
        self._fxs.send_motor_command(self._dev_id, fxe.FX_CURRENT, current)

    def _set_motor_torque(self, torque: float = 0.0):
        self._set_qaxis_current(torque / self.NM_PER_AMP)

    @property
    def fxs(self):
        return self._fxs

    @property
    def port(self):
        return self._port

    @property
    def baud_rate(self):
        return self._baud_rate

    @property
    def is_streaming(self):
        return self._streaming

    @property
    def state_time(self):
        return self._data.state_time

    @property
    def battery_current(self):
        return self._data.batt_curr

    @property
    def battery_voltage(self):
        return self._data.batt_volt

    @property
    def motor_temperature(self):
        return self._data.temp

    @property
    def motor_current(self):
        return self._data.mot_cur

    @property
    def motor_angle_counts(self):
        return self._data.mot_ang

    @property
    def motor_angle(self):
        return self._data.mot_ang * self.RAD_PER_COUNT

    @property
    def motor_velocity(self):
        return self._data.mot_vel * self.RAD_PER_DEG

    @property
    def motor_torque(self):
        return self.motor_current * 1e-3 * self.NM_PER_AMP

    @property
    def motor_acceleration(self):
        return self._data.mot_acc

    @property
    def joint_angle(self):
        return self._data.ank_ang

    @property
    def joint_velocity(self):
        return self._data.ank_vel

    @property
    def acceleration_x(self):
        return self._data.accelx * self.G_PER_ACCLSB

    @property
    def acceleration_y(self):
        return self._data.accely * self.G_PER_ACCLSB

    @property
    def acceleration_z(self):
        return self._data.accelz * self.G_PER_ACCLSB

    @property
    def gyro_x(self):
        return self._data.gyro_x * self.RAD_PER_SEC_GYROLSB

    @property
    def gyro_y(self):
        return self._data.gyro_y * self.RAD_PER_SEC_GYROLSB

    @property
    def gyro_z(self):
        return self._data.gyro_z * self.RAD_PER_SEC_GYROLSB

    @property
    def gyroscope_vector(self):
        return (
            np.array([self._data.gyro_x, self._data.gyro_y, self._data.gyro_z]).T
            * self.RAD_PER_SEC_GYROLSB
        )

    @property
    def accelerometer_vector(self):
        return (
            np.array([self._data.accelx, self._data.accely, self._data.accelz]).T
            * self.G_PER_ACCLSB
        )

    @property
    def genvars(self):
        return np.array(
            [
                self._data.genvar_0,
                self._data.genvar_1,
                self._data.genvar_2,
                self._data.genvar_3,
                self._data.genvar_4,
                self._data.genvar_5,
            ]
        )


class JointState(Enum):
    NEUTRAL = 0
    VOLTAGE = 1
    POSITION = 2
    CURRENT = 3
    IMPEDANCE = 4


class Joint(Actpack):
    def __init__(
        self, name, fxs, port, baud_rate, frequency, logger, debug_level=0
    ) -> None:
        super().__init__(fxs, port, baud_rate, frequency, logger, debug_level)

        self._name = name
        self._filename = "./encoder_map_" + self._name + ".txt"

        self._count2deg = 360 / 2**14
        self._joint_angle_array = None
        self._motor_count_array = None

        self._state = JointState.NEUTRAL

        self._k: int = 0
        self._b: int = 0
        self._theta: int = 0

    def home(self, save=True, homing_voltage=2500, homing_rate=0.001):

        # TODO Logging module
        self.log.info(f"[{self._name}] Initiating Homing Routine.")

        minpos_motor, minpos_joint, min_output = self._homing_routine(
            direction=1.0, hvolt=homing_voltage, hrate=homing_rate
        )
        self.log.info(
            f"[{self._name}] Minimum Motor angle: {minpos_motor}, Minimum Joint angle: {minpos_joint}"
        )
        time.sleep(0.5)
        maxpos_motor, maxpos_joint, max_output = self._homing_routine(
            direction=-1.0, hvolt=homing_voltage, hrate=homing_rate
        )
        self.log.info(
            f"[{self.name}] Maximum Motor angle: {maxpos_motor}, Maximum Joint angle: {maxpos_joint}"
        )

        max_output = np.array(max_output).reshape((len(max_output), 2))
        output_motor_count = max_output[:, 1]

        _, ids = np.unique(output_motor_count, return_index=True)

        if save:
            self._save_encoder_map(data=max_output[ids])

        self.log.info(f"[{self.name}] Homing Successfull.")

    def _homing_routine(self, direction, hvolt=2500, hrate=0.001):
        """Homing Routine

        Args:
            direction (_type_): _description_
            hvolt (int, optional): _description_. Defaults to 2500.
            hrate (float, optional): _description_. Defaults to 0.001.

        Returns:
            _type_: _description_
        """
        output = []
        velocity_threshold = 0
        go_on = True

        self.update()
        current_motor_position = self.motor_angle
        current_joint_position = self.joint_angle

        self.switch_state(JointState.VOLTAGE)
        self.set_voltage(direction * hvolt)
        time.sleep(0.05)
        self.update()
        cpos_motor = self.motor_angle
        initial_velocity = self.joint_velocity
        output.append([self.joint_angle * self._count2deg] + [cpos_motor])
        velocity_threshold = abs(initial_velocity / 2.0)

        while go_on:
            time.sleep(hrate)
            self.update()
            cpos_motor = self.motor_angle
            cvel_joint = self.joint_velocity
            output.append([self.joint_angle * self._count2deg] + [cpos_motor])

            if abs(cvel_joint) <= velocity_threshold:
                self.set_voltage(0)
                current_motor_position = self.motor_angle
                current_joint_position = self.joint_angle

                go_on = False

        return current_motor_position, current_joint_position, output

    def get_motor_count(self, desired_joint_angle):
        """Returns Motor Count corresponding to the passed Joint angle value

        Args:
            desired_joint_angle (_type_): _description_

        Returns:
            _type_: _description_
        """
        if self._joint_angle_array is None:
            self._load_encoder_map()

        desired_motor_count = np.interp(
            np.array(desired_joint_angle),
            self._joint_angle_array,
            self._motor_count_array,
        )
        return desired_motor_count

    def switch_state(self, to_state: JointState = JointState.NEUTRAL):
        self._state = to_state

    def set_voltage(self, volt):
        if self.state == JointState.VOLTAGE:
            self._set_voltage(volt)

    def set_current(self, current):
        if self.state == JointState.CURRENT:
            self._set_current_gains()
            self._set_qaxis_current(current)
        else:
            self.log.warning("Joint State is incorrect.")

    def set_position(self, position):
        if self.state == JointState.POSITION:
            self._set_position_gains()
            self._set_motor_angle_counts(position)

    def set_impedance(self, k: int = 300, b: int = 1600, theta: int = None):
        self._k = k
        self._b = b
        self._theta = theta

        if self.state == JointState.IMPEDANCE:
            self._set_impedance_gains(K=k, B=b)
            self._set_equilibrium_angle(theta=theta)

    def _save_encoder_map(self, data):
        """
        Saves encoder_map: [Joint angle, Motor count] to a text file
        """
        np.savetxt(self._filename, data, fmt="%.5f")

    def _load_encoder_map(self):
        """
        Loads Joint angle array, Motor count array, Min Joint angle, and Max Joint angle
        """
        data = np.loadtxt(self._filename, dtype=np.float64)
        self._joint_angle_array = data[:, 0]
        self._motor_count_array = np.array(data[:, 1], dtype=np.int32)

        self._min_joint_angle = np.min(self._joint_angle_array)
        self._max_joint_angle = np.max(self._joint_angle_array)

        self._joint_angle_array = self._max_joint_angle - self._joint_angle_array

        # Applying a median filter with a kernel size of 3
        self._joint_angle_array = scipy.signal.medfilt(
            self._joint_angle_array, kernel_size=3
        )
        self._motor_count_array = scipy.signal.medfilt(
            self._motor_count_array, kernel_size=3
        )

    @property
    def name(self):
        return self._name

    @property
    def state(self):
        return self._state

    @property
    def stiffness(self):
        return self._k

    @property
    def damping(self):
        return self._b

    @property
    def equilibrium_angle(self):
        return self._theta


class Loadcell:
    def __init__(
        self,
        joint: Joint,
        amp_gain: float = 125.0,
        exc: float = 5.0,
        loadcell_matrix=None,
        logger: logging.Logger = None,
    ) -> None:
        self._joint = joint
        self._amp_gain = 125.0
        self._exc = 5.0

        if not loadcell_matrix:
            self._loadcell_matrix = np.array(
                [
                    (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
                    (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
                    (
                        -1047.16800,
                        8.63900,
                        -1047.28200,
                        -20.70000,
                        -1073.08800,
                        -8.92300,
                    ),
                    (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
                    (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
                    (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
                ]
            )
        else:
            self._loadcell_matrix = loadcell_matrix

        self._loadcell_data = None
        self._loadcell_zero = np.zeros((1, 6), dtype=np.double)
        self._zeroed = False
        self.log = logger

    def reset(self):
        self._zeroed = False
        self._loadcell_zero = np.zeros((1, 6), dtype=np.double)

    def update(self, loadcell_zero=None):
        """
        Computes Loadcell data

        """
        loadcell_signed = (self._joint.genvars - 2048) / 4095 * self._exc
        loadcell_coupled = loadcell_signed * 1000 / (self._exc * self._amp_gain)

        if loadcell_zero is None:
            self._loadcell_data = (
                np.transpose(self._loadcell_matrix.dot(np.transpose(loadcell_coupled)))
                - self._loadcell_zero
            )
        else:
            self._loadcell_data = (
                np.transpose(self._loadcell_matrix.dot(np.transpose(loadcell_coupled)))
                - loadcell_zero
            )

    def initialize(self, number_of_iterations: int = 2000):
        """
        Obtains the initial loadcell reading (aka) loadcell_zero
        """
        ideal_loadcell_zero = np.zeros((1, 6), dtype=np.double)
        if not self._zeroed:
            if self._joint.is_streaming:
                self._joint.update()
                self.update()
                self._loadcell_zero = self._loadcell_data

                for _ in range(number_of_iterations):
                    self.update(ideal_loadcell_zero)
                    loadcell_offset = self._loadcell_data
                    self._loadcell_zero = (loadcell_offset + self._loadcell_zero) / 2.0

        elif input("Do you want to re-initialize loadcell? (Y/N)") == "Y":
            self.reset()
            self.initialize()

    @property
    def is_zeroed(self):
        return self._zeroed

    @property
    def fx(self):
        return self._loadcell_data[0][0]

    @property
    def fy(self):
        return self._loadcell_data[0][1]

    @property
    def fz(self):
        return self._loadcell_data[0][2]

    @property
    def mx(self):
        return self._loadcell_data[3]

    @property
    def my(self):
        return self._loadcell_data[4]

    @property
    def mz(self):
        return self._loadcell_data[5]


class OSLV2:
    """
    The OSL class
    """

    def __init__(self, frequency: int = 200, log_data=False) -> None:

        self._fxs = flex.FlexSEA()
        self._loadcell = None

        self.joints: list[Joint] = []

        self._knee_id = None
        self._ankle_id = None

        self._frequency = frequency

        # --------------------------------------------

        self._log_data = log_data
        self._log_filename = "osl.log"
        self.log = logging.getLogger(__name__)

        if log_data:
            self.log.setLevel(logging.DEBUG)
        else:
            self.log.setLevel(logging.INFO)

        self._std_formatter = logging.Formatter(
            "[%(asctime)s] %(levelname)s: %(message)s"
        )

        self._file_handler = RotatingFileHandler(
            self._log_filename, mode="w", maxBytes=2000, backupCount=10
        )
        self._file_handler.setLevel(logging.DEBUG)
        self._file_handler.setFormatter(self._std_formatter)

        self._stream_handler = logging.StreamHandler()
        self._stream_handler.setLevel(logging.INFO)
        self._stream_handler.setFormatter(self._std_formatter)

        self.log.addHandler(self._stream_handler)
        self.log.addHandler(self._file_handler)

        # ----------------------------------------------

    def __enter__(self):
        for joint in self.joints:
            joint._start_streaming_data()

        if self.loadcell is not None:
            self.loadcell.initialize()

    def __exit__(self, type, value, tb):
        for joint in self.joints:
            joint.switch_state()
            joint._shutdown()

    def add_joint(self, name: str, port, baud_rate, debug_level=0):

        if "knee" in name.lower():
            self._knee_id = len(self.joints)
        elif "ankle" in name.lower():
            self._ankle_id = len(self.joints)
        else:
            sys.exit("Joint can't be identified, kindly check the given name.")

        self.joints.append(
            Joint(
                name=name,
                fxs=self._fxs,
                port=port,
                baud_rate=baud_rate,
                frequency=self._frequency,
                logger=self.log,
                debug_level=debug_level,
            )
        )

    def add_loadcell(
        self,
        joint: Joint,
        amp_gain: float = 125.0,
        exc: float = 5.0,
        loadcell_matrix=None,
    ):
        self._loadcell = Loadcell(
            joint=joint,
            amp_gain=amp_gain,
            exc=exc,
            loadcell_matrix=loadcell_matrix,
            logger=self.log,
        )

    def update(self):
        for joint in self.joints:
            joint.update()

        if self.loadcell is not None:
            self.loadcell.update()

    def home(self):
        for joint in self.joints:
            joint.home()

    @property
    def loadcell(self):
        if self._loadcell is not None:
            return self._loadcell
        else:
            sys.exit("Loadcell not connected.")

    @property
    def knee(self):
        if self._knee_id is not None:
            return self.joints[self._knee_id]
        else:
            sys.exit("Knee is not connected.")

    @property
    def ankle(self):
        if self._ankle_id is not None:
            return self.joints[self._ankle_id]
        else:
            sys.exit("Ankle is not connected.")


if __name__ == "__main__":
    start = time.perf_counter()

    osl = OSLV2(log_data=False)
    osl.add_joint(name="Knee", port="/dev/ttyACM0", baud_rate=230400)
    osl.add_loadcell(osl.knee, amp_gain=125, exc=5)

    with osl:
        osl.loadcell.initialize()
        osl.update()
        osl.knee.home()
        osl.log.info(f"{osl.knee.motor_angle}")

    finish = time.perf_counter()
    osl.log.info(f"Script ended at {finish-start:0.4f}")
