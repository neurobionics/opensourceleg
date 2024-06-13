from typing import Any

from dataclasses import dataclass

import numpy as np

import opensourceleg.hardware.actuators.dephy as dephy
import opensourceleg.hardware.sensor.base as base
from opensourceleg.hardware.actuators.base import MecheConsts
from opensourceleg.hardware.thermal import ThermalModel


class DephySensor(base.SensorIMU):

    def __init__(self, device: "dephy.DephyActpack"):
        self._device = device

        self._kinematics = DephyKinematics(self)
        self._motor = DephyMotor(self)
        self._thermal = DephyThermal(self)
        self._joint_encoder = JointEncoder(self)
        self._battery = DephyBattery(self)
        self._data: Any = None
        # self.status_ex = 0b00000000

    def start_streaming(self):

        # self.is_streaming = True
        # print("Please ensure that the actpack is connected and streaming.")
        pass

    def stop_streaming(self):
        # self.is_streaming = False
        pass

    @property
    def is_streaming(self):
        return self._device.is_streaming

    def get_data(self):

        if self._device.is_streaming:
            self._data = self._device.read()
            self.status_ex = self._data.status_ex

        else:
            pass
            # TODO: should raise an error here
            # self._data = MockData()
        return self._data


class DephyKinematics:
    def __init__(self, Sensor: DephySensor) -> None:
        self._device = Sensor
        self._data: Any = None

    def update(self):
        self._data = self._device.data

    @property
    def genvars(self):
        """Dephy's 'genvars' object."""
        if self._data is not None:
            return np.array(
                object=[
                    self._data.genvar_0,
                    self._data.genvar_1,
                    self._data.genvar_2,
                    self._data.genvar_3,
                    self._data.genvar_4,
                    self._data.genvar_5,
                ]
            )
        else:
            return np.zeros(shape=6)

    @property
    def accelx(self) -> float:
        """
        Acceleration in x direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.accelx * MecheConsts.M_PER_SEC_SQUARED_ACCLSB)
        else:
            return 0.0

    @property
    def accely(self) -> float:
        """
        Acceleration in y direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.accely * MecheConsts.M_PER_SEC_SQUARED_ACCLSB)
        else:
            return 0.0

    @property
    def accelz(self) -> float:
        """
        Acceleration in z direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.accelz * MecheConsts.M_PER_SEC_SQUARED_ACCLSB)
        else:
            return 0.0

    @property
    def gyrox(self) -> float:
        """
        Angular velocity in x direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.gyrox * MecheConsts.RAD_PER_SEC_GYROLSB)
        else:
            return 0.0

    @property
    def gyroy(self) -> float:
        """
        Angular velocity in y direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.gyroy * MecheConsts.RAD_PER_SEC_GYROLSB)
        else:
            return 0.0

    @property
    def gyroz(self) -> float:
        """
        Angular velocity in z direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.gyroz * MecheConsts.RAD_PER_SEC_GYROLSB)
        else:
            return 0.0


class DephyMotor:
    def __init__(self, Sensor: DephySensor) -> None:
        self._device = Sensor
        self._data: Any = None
        self._motor_zero_position = 0.0
        self._motor_offset = 0.0
        self._encoder_map = None

    def update(self):
        self._data = self._device.data

    @property
    def encoder_map(self):
        """Polynomial coefficients defining the joint encoder map from counts to radians."""
        return self._encoder_map

    @property
    def motor_zero_position(self) -> float:
        """Motor encoder zero position in radians."""
        return self._motor_zero_position

    @property
    def motor_offset(self) -> float:
        """Motor encoder offset in radians."""
        return self._motor_offset

    @property
    def motor_voltage(self) -> float:
        """Q-axis motor voltage in mV."""
        if self._data is not None:
            return float(self._data.mot_volt)
        else:
            return 0.0

    @property
    def motor_current(self) -> float:
        """Q-axis motor current in mA."""
        if self._data is not None:
            return float(self._data.mot_cur)
        else:
            return 0.0

    @property
    def motor_torque(self) -> float:
        """
        Torque at motor output in Nm.
        This is calculated using the motor current and torque constant.
        """
        if self._data is not None:
            return float(self._data.mot_cur * MecheConsts.NM_PER_MILLIAMP)
        else:
            return 0.0

    @property
    def motor_position(self) -> float:
        """Angle of the motor in radians."""
        if self._data is not None:
            return (
                float(self._data.mot_ang * MecheConsts.RAD_PER_COUNT)
                - self._motor_zero_position
                - self.motor_offset
            )
        else:
            return 0.0

    @property
    def motor_encoder_counts(self) -> int:
        """Raw reading from motor encoder in counts."""
        return int(self._data.mot_ang)

    @property
    def motor_velocity(self) -> float:
        """Motor velocity in rad/s."""
        if self._data is not None:
            return int(self._data.mot_vel) * MecheConsts().RAD_PER_DEG
        else:
            return 0.0

    @property
    def motor_acceleration(self) -> float:
        """Motor acceleration in rad/s^2."""
        if self._data is not None:
            return float(self._data.mot_acc)
        else:
            return 0.0


class DephyThermal:
    def __init__(self, Sensor: DephySensor) -> None:
        self._device = Sensor
        self._data: Any = None
        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=80,
            soft_border_C_windings=10,
            temp_limit_case=70,
            soft_border_C_case=10,
        )
        self._thermal_scale: float = 1.0

    def update(self):
        self._data = self._device.data

    @property
    def case_temperature(self) -> float:
        """Case temperature in celsius."""
        if self._data is not None:
            return float(self._data.temperature)
        else:
            return 0.0

    @property
    def winding_temperature(self) -> float:
        """
        ESTIMATED temperature of the windings in celsius.
        This is calculated based on the thermal model using motor current.
        """
        if self._data is not None:
            return float(self._thermal_model.T_w)
        else:
            return 0.0

    @property
    def thermal_scaling_factor(self) -> float:
        """
        Scale factor to use in torque control, in [0,1].
        If you scale the torque command by this factor, the motor temperature will never exceed max allowable temperature.
        For a proof, see paper referenced in thermal model.
        """
        return float(self._thermal_scale)


class JointEncoder:
    def __init__(self, Sensor: DephySensor) -> None:
        self._device = Sensor
        self._data: Any = None
        self._joint_offset = 0.0
        self._joint_zero_position = 0.0
        self._joint_direction = 1.0

    def update(self):
        self._data = self._device.data

    @property
    def joint_zero_position(self) -> float:
        """Joint encoder zero position in radians."""
        return self._joint_zero_position

    @property
    def joint_offset(self) -> float:
        """Joint encoder offset in radians."""
        return self._joint_offset

    @property
    def joint_direction(self) -> float:
        """Joint direction: 1 or -1"""
        return self._joint_direction

    @property
    def joint_encoder_counts(self) -> int:
        """Raw reading from joint encoder in counts."""
        return int(self._data.ank_ang)

    @property
    def joint_position(self) -> float:
        """Measured angle from the joint encoder in radians."""
        if self._data is not None:
            if self._device._motor.encoder_map is not None:
                return float(self._device._motor.encoder_map(self._data.ank_ang))
            else:
                return (
                    float(self._data.ank_ang * MecheConsts().RAD_PER_COUNT)
                    - self.joint_zero_position
                    - self.joint_offset
                ) * self.joint_direction
        else:
            return 0.0

    @property
    def joint_velocity(self) -> float:
        """Measured velocity from the joint encoder in rad/s."""
        if self._data is not None:
            return float(self._data.ank_vel * MecheConsts().RAD_PER_COUNT)
        else:
            return 0.0


class DephyBattery:

    def __init__(self, Sensor: DephySensor) -> None:
        self._device = Sensor
        self._data: Any = None

    def update(self):
        self._data = self._device.data

    @property
    def battery_voltage(self) -> float:
        """Battery voltage in mV."""
        if self._data is not None:
            return float(self._data.batt_volt)
        else:
            return 0.0

    @property
    def battery_current(self) -> float:
        """Battery current in mA."""
        if self._data is not None:
            return float(self._data.batt_curr)
        else:
            return 0.0


class MockData:
    def __init__(
        self,
        batt_volt=0,
        batt_curr=0,
        mot_volt=0,
        mot_cur=0,
        mot_ang=0,
        ank_ang=0,
        mot_vel=0,
        mot_acc=0,
        ank_vel=0,
        temperature=0,
        genvar_0=0,
        genvar_1=0,
        genvar_2=0,
        genvar_3=0,
        genvar_4=0,
        genvar_5=0,
        accelx=0,
        accely=0,
        accelz=0,
        gyrox=0,
        gyroy=0,
        gyroz=0,
    ):
        self.batt_volt = batt_volt
        self.batt_curr = batt_curr
        self.mot_volt = mot_volt
        self.mot_cur = mot_cur
        self.mot_ang = mot_ang
        self.ank_ang = ank_ang
        self.mot_vel = mot_vel
        self.mot_acc = mot_acc
        self.ank_vel = ank_vel
        self.temperature = temperature
        self.genvar_0 = genvar_0
        self.genvar_1 = genvar_1
        self.genvar_2 = genvar_2
        self.genvar_3 = genvar_3
        self.genvar_4 = genvar_4
        self.genvar_5 = genvar_5
        self.accelx = accelx
        self.accely = accely
        self.accelz = accelz
        self.gyrox = gyrox
        self.gyroy = gyroy
        self.gyroz = gyroz
        self.status_ex = 0b00000000

    def __repr__(self):
        return f"MockData"


if __name__ == "__main__":
    pass
    # dephy_actpack = DephyActpack(port="/dev/ttyUSB0", baud_rate=115200)
    # my_dephy_imu = DephyIMU(dephy_actpack)
