#!/usr/bin/python3
from typing import Any, Callable, List, Optional

import threading
import time

import numpy as np
import scipy.signal
from flexsea import flexsea as flex
from flexsea import fxEnums as fxe
from flexsea import fxUtils as fxu


class ActpackData:
    """A class that contains and works with all the sensor data from a Dephy Actpack.

    Attributes:
        motor_angle (double) : Motor angle
        motor_velocity (double : Motor velocity

    """

    def __init__(self) -> None:
        self.motor_angle: np.double
        self.motor_velocity: np.double
        self.motor_acceleration: np.double

        self.joint_angle: np.double
        self.joint_velocity: np.double

        self.fx: np.double
        self.fy: np.double
        self.fz: np.double

    def update(self, motor, joint, loadcell):
        """_summary_

        Args:
            motor (_type_): _description_
            joint (_type_): _description_
            loadcell (_type_): _description_
        """
        self.motor_angle = motor[0]
        self.motor_velocity = motor[1]
        self.motor_acceleration = motor[2]

        self.joint_angle = joint[0]
        self.joint_velocity = joint[1]

        self.fx = loadcell[0]
        self.fy = loadcell[1]
        self.fz = loadcell[2]


class Joint:
    def __init__(
        self, name, fxs, dev_id, homing_voltage=2500, homing_rate=0.001
    ) -> None:
        self._name = name
        self._filename = "./encoder_map_" + self._name + ".txt"

        self._fxs = fxs
        self._dev_id = dev_id

        self._homing_voltage = homing_voltage
        self._homing_rate = homing_rate

        self._count2deg = 360 / 2**14
        self._deg2count = 2**14 / 360

        self._bit_2_degree_per_sec = 1 / 32.8
        self._bit_2_g = 1 / 8192

        # Sensor data
        self.data = ActpackData()

    def home(self, save=True):

        print("*** Initiating Homing Routine ***")

        minpos_motor, minpos_joint, min_output = self._homing_routine(direction=1.0)
        print(
            f"\nMinimum Motor angle: {minpos_motor}, Minimum Joint angle: {minpos_joint}"
        )
        time.sleep(0.5)
        maxpos_motor, maxpos_joint, max_output = self._homing_routine(direction=-1.0)
        print(
            f"\nMaximum Motor angle: {maxpos_motor}, Maximum Joint angle: {maxpos_joint}"
        )

        max_output = np.array(max_output).reshape((len(max_output), 2))
        output_motor_count = max_output[:, 1]

        _, ids = np.unique(output_motor_count, return_index=True)

        if save:
            self.save_encoder_map(data=max_output[ids])

        print("*** Homing Successfull ***")

    def _homing_routine(self, direction):
        """
        Private function to aid homing process
        """
        output = []
        velocity_threshold = 0
        go_on = True

        data = self._fxs.read_device(self._dev_id)
        current_motor_position = data.mot_ang
        current_joint_position = data.ank_ang

        try:
            self._fxs.send_motor_command(
                self._dev_id, fxe.FX_VOLTAGE, direction * self._homing_voltage
            )
            time.sleep(0.05)

            data = self._fxs.read_device(self._dev_id)
            cpos_motor = data.mot_ang
            initial_velocity = data.ank_vel
            output.append([data.ank_ang * self._count2deg] + [cpos_motor])

            velocity_threshold = abs(initial_velocity / 2.0)

            while go_on:
                time.sleep(self._homing_rate)
                data = self._fxs.read_device(self._dev_id)
                cpos_motor = data.mot_ang
                cvel_joint = data.ank_vel

                output.append([data.ank_ang * self._count2deg] + [cpos_motor])

                if abs(cvel_joint) <= velocity_threshold:
                    self._fxs.send_motor_command(self._dev_id, fxe.FX_VOLTAGE, 0)
                    current_motor_position = data.mot_ang
                    current_joint_position = data.ank_ang

                    go_on = False

        except KeyboardInterrupt:
            print("Stopping homing routine!")
            self._fxs.send_motor_command(self._dev_id, fxe.FX_NONE, 0)

        return current_motor_position, current_joint_position, output

    def save_encoder_map(self, data):
        """
        Saves encoder_map: [Joint angle, Motor count] to a text file
        """
        np.savetxt(self._filename, data, fmt="%.5f")

    def load_encoder_map(self):
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

    def joint_angle_2_motor_count(self, desired_joint_angle):
        desired_joint_angle_array = np.array(desired_joint_angle)
        desired_motor_count = np.interp(
            desired_joint_angle_array, self._joint_angle_array, self._motor_count_array
        )
        return desired_motor_count

    def get_orientation(self, raw_acc, raw_g):
        acc = raw_acc * self._bit_2_g
        gyro = raw_g * self._bit_2_degree_per_sec
        return acc, gyro

    @property
    def name(self):
        return self._name


class OSL:
    """
    The OSL class
    """

    def __init__(self, port, baud_rate, debug_level=0) -> None:

        self.port = port
        self.baud_rate = baud_rate

        # Actuator Variables
        self.fxs = flex.FlexSEA()
        self.dev_id = self.fxs.open(self.port, self.baud_rate, debug_level)
        self.app_type = self.fxs.get_app_type(self.dev_id)

        # Joint Variables
        self.knee = Joint(name="knee", fxs=self.fxs, dev_id=self.dev_id)

        # Loadcell Variables
        self.body_weight = 155
        self.loadcell_matrix = None
        self.loadcell_zero = None

    def _start_streaming_data(self, frequency=500, log_en=False):
        """_summary_

        Args:
            frequency (int, optional): _description_. Defaults to 500.
            log_en (bool, optional): _description_. Defaults to False.
        """

        # If loadcell hasn't been initialized, assign default values.
        if not self.loadcell_matrix:
            self.initialize_loadcell()

        self.fxs.start_streaming(self.dev_id, freq=frequency, log_en=log_en)
        time.sleep(2)

        if input("Do you want to initiate homing process? (y/n) ") == "y":
            self.knee.home()

        self.knee.load_encoder_map()

    def _stop_streaming_data(self):
        """
        Shuts down the actuator
        """
        self.fxs.send_motor_command(self.dev_id, fxe.FX_NONE, 0)
        time.sleep(1)
        self.fxs.stop_streaming(self.dev_id)
        time.sleep(1)
        self.fxs.close(self.dev_id)

    def initialize_loadcell(self, amp_gain=125.0, exc=5.0, loadcell_matrix=None):
        """_summary_

        Args:
            amp_gain (float, optional): _description_. Defaults to 125.0.
            exc (float, optional): _description_. Defaults to 5.0.
            loadcell_matrix (_type_, optional): _description_. Defaults to None.
        """
        self.amp_gain = amp_gain
        self.exc = exc

        if not loadcell_matrix:
            self.loadcell_matrix = np.array(
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
            self.loadcell_matrix = loadcell_matrix

    def _get_loadcell_data(self, loadcell_raw, loadcell_zero):
        """
        Computes Loadcell data

        """
        loadcell_signed = (loadcell_raw - 2048) / 4095 * self.exc
        loadcell_coupled = loadcell_signed * 1000 / (self.exc * self.amp_gain)

        return (
            np.transpose(self.loadcell_matrix.dot(np.transpose(loadcell_coupled)))
            - loadcell_zero
        )

    def _get_loadcell_zero(self, loadcell_raw, number_of_iterations=2000):
        """
        Obtains the initial loadcell reading (aka) loadcell_zero
        """
        initial_loadcell_zero = np.zeros((1, 6), dtype=np.double)
        loadcell_zero = loadcell_offset = self._get_loadcell_data(
            loadcell_raw, initial_loadcell_zero
        )

        for i in range(number_of_iterations):
            loadcell_offset = self._get_loadcell_data(
                loadcell_raw, initial_loadcell_zero
            )
            loadcell_zero = (loadcell_offset + loadcell_zero) / 2.0

        return loadcell_zero

    def _get_sensor_data(self, i, dt):
        """
        Streams data from the actuator.

        Parameters:
        -----------

        Returns:
        --------
        acc:
        gyro:
        loadcell_raw:

        """
        data = self.fxs.read_device(self.dev_id)

        motor = np.array([data.mot_ang, data.mot_vel, data.mot_acc])
        joint = np.array([data.ank_ang, data.ank_vel])
        loadcell_raw = np.array(
            [
                data.genvar_0,
                data.genvar_1,
                data.genvar_2,
                data.genvar_3,
                data.genvar_4,
                data.genvar_5,
            ]
        )

        if i == 0:
            self.loadcell_zero = self._get_loadcell_zero(loadcell_raw)

        loadcell = self._get_loadcell_data(loadcell_raw, self.loadcell_zero)

        self.knee.data.update(motor, joint, loadcell[0])

    def read(self, duration=15, time_step=0.01):
        """
        Reads data from the actuator for 'n' number of seconds.
        """
        number_of_iterations = int(duration / time_step)

        self._start_streaming_data()

        now = then = time.time()

        for i in range(number_of_iterations):
            now = time.time()
            dt = now - then

            time.sleep(time_step)
            fxu.clear_terminal()

            motor, joint, acc, gyro, loadcell_raw = self._get_sensor_data()

            if i == 0:
                print("*** Obtaining initial value of loadcell ***")
                loadcell_zero = self._get_loadcell_zero(loadcell_raw)

            loadcell = self._get_loadcell_data(loadcell_raw, loadcell_zero)
            np.set_printoptions(suppress=True)
            print(self.knee.get_orientation(acc, gyro))

            then = now

        time.sleep(1)
        self._stop_streaming_data()

    def walk(self, duration=15, time_step=0.001):
        """
        Walks for 'n' number of seconds.
        """
        number_of_iterations = int(duration / time_step)

        self._start_streaming_data()

        kp_I = 40
        ki_I = 400
        k_FF = 128

        self.fxs.set_gains(self.dev_id, kp_I, ki_I, 0, 0, 0, k_FF)
        time.sleep(0.5)

        try:
            now = then = time.time()
            for i in range(number_of_iterations):
                now = time.time()
                dt = now - then

                time.sleep(time_step)
                fxu.clear_terminal()

                self._get_sensor_data(i, dt)
                np.set_printoptions(suppress=True)

                self.fxs.send_motor_command(
                    self.dev_id,
                    fxe.FX_IMPEDANCE,
                    0.0,
                )
                self.fxs.set_gains(
                    self.dev_id,
                    kp_I,
                    ki_I,
                    int(self.knee.joint_angle_2_motor_count(30)),
                    int(30.0),
                    int(120.0),
                    k_FF,
                )

                then = now

            print("Exiting the program.")
            time.sleep(1)
            self._stop_streaming_data()

        except KeyboardInterrupt:
            print("Keyboard Interrupt detected, exiting the program.")
            time.sleep(1)
            self._stop_streaming_data()


if __name__ == "__main__":
    start = time.perf_counter()

    osl = OSL(port="/dev/ttyACM0", baud_rate=230400)
    walking_thread = threading.Thread(target=osl.walk, args=(45,))
    walking_thread.start()

    finish = time.perf_counter()
    print(f"Script ended at {finish-start:0.4f}")
