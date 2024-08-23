import csv
import os
import time
import traceback
import warnings
from collections import namedtuple
from enum import Enum
from math import isfinite

import can
import numpy as np

from opensourceleg.actuators.base import (
    CONTROL_MODE_CONFIGS,
    CONTROL_MODES,
    MOTOR_CONSTANTS,
    ActuatorBase,
    ControlGains,
    ControlModeConfig,
)
from opensourceleg.actuators.decorators import (
    check_actuator_connection,
    check_actuator_open,
    check_actuator_stream,
)
from opensourceleg.logging import LOGGER
from opensourceleg.math import ThermalModel
from opensourceleg.time import SoftRealtimeLoop

TMOTOR_ACTUATOR_CONSTANTS = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=16384,
    NM_PER_AMP=0.1133,
    NM_PER_RAD_TO_K=0.0,  # TODO: Find value
    NM_S_PER_RAD_TO_B=0.0,  # TODO: Find value
    MAX_CASE_TEMPERATURE=80,
    MAX_WINDING_TEMPERATURE=110,
)

# Parameter dictionary for each specific motor that can be controlled with this library
# Thresholds are in the datasheet for the motor on cubemars.com

MIT_Params = {
    "ERROR_CODES": {
        0: "No Error",
        1: "Over temperature fault",
        2: "Over current fault",
        3: "Over voltage fault",
        4: "Under voltage fault",
        5: "Encoder fault",
        6: "Phase current unbalance fault (The hardware may be damaged)",
    },
    "AK80-9": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -18.0,
        "T_max": 18.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.091,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # to correct the qaxis current
        "Kt_actual": 0.115,  # Need to use the right constant -- 0.115 by our calcs, 0.091 by theirs. At output leads to 1.31 by them and 1.42 by us.
        "GEAR_RATIO": 9.0,  # hence the 9 in the name
        "Use_derived_torque_constants": True,  # true if you have a better model
        "a_hat": [0.0, 1.15605006e00, 4.17389589e-04, 2.68556072e-01, 4.90424140e-02]
        #'a_hat' : [0.0,  8.23741648e-01, 4.57963164e-04,     2.96032614e-01, 9.31279510e-02]# [7.35415941e-02, 6.26896231e-01, 2.65240487e-04,     2.96032614e-01,  7.08736309e-02]# [-5.86860385e-02,6.50840079e-01,3.47461078e-04,8.58635580e-01,2.93809281e-01]
    },
    "AK10-9": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -65.0,
        "T_max": 65.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.16,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # UNTESTED CONSTANT!
        "Kt_actual": 0.206,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 9.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK60-6": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -15.0,
        "T_max": 15.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.068,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # # UNTESTED CONSTANT!
        "Kt_actual": 0.087,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 6.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK70-10": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -25.0,
        "T_max": 25.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.095,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # # UNTESTED CONSTANT!
        "Kt_actual": 0.122,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 10.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK80-6": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -76.0,
        "V_max": 76.0,
        "T_min": -12.0,
        "T_max": 12.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.091,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # # UNTESTED CONSTANT!
        "Kt_actual": 0.017,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 6.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK80-64": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -8.0,
        "V_max": 8.0,
        "T_min": -144.0,
        "T_max": 144.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.119,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # # UNTESTED CONSTANT!
        "Kt_actual": 0.153,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 80.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
}
"""
A Dictionary containing the parameters of each type of motor, as well as the error
code definitions for the AK-series TMotor actuators.
You could use an optional torque model that accounts for friction losses if one is available.
So far, such a model is only available for the AK80-9.

This model comes from a linear regression with the following constants:
    - a_hat[0] = bias
    - a_hat[1] = standard torque constant multiplier
    - a_hat[2] = nonlinear torque constant multiplier
    - a_hat[3] = coloumb friction
    - a_hat[4] = gearbox friction

The model has the form:
τ = a_hat[0] + gr*(a_hat[1]*kt - a_hat[2]*abs(i))*i - (v/(ϵ + np.abs(v)) )*(a_hat[3] + a_hat[4]*np.abs(i))

with the following values:
    - τ = approximated torque
    - gr = gear ratio
    - kt = nominal torque constant
    - i = current
    - v = velocity
    - ϵ = signum velocity threshold
"""


class motor_state:
    """Data structure to store and update motor states"""

    def __init__(self, position, velocity, current, temperature, error, acceleration):
        """
        Sets the motor state to the input.

        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            current: current in amps
            temperature: temperature in degrees C
            error: error code, 0 means no error
        """
        self.set_state(position, velocity, current, temperature, error, acceleration)

    def set_state(self, position, velocity, current, temperature, error, acceleration):
        """
        Sets the motor state to the input.

        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            current: current in amps
            temperature: temperature in degrees C
            error: error code, 0 means no error
        """
        self.position = position
        self.velocity = velocity
        self.current = current
        self.temperature = temperature
        self.error = error
        self.acceleration = acceleration

    def set_state_obj(self, other_motor_state):
        """
        Sets this motor state object's values to those of another motor state object.

        Args:
            other_motor_state: The other motor state object with values to set this motor state object's values to.
        """
        self.position = other_motor_state.position
        self.velocity = other_motor_state.velocity
        self.current = other_motor_state.current
        self.temperature = other_motor_state.temperature
        self.error = other_motor_state.error
        self.acceleration = other_motor_state.acceleration


# Data structure to store MIT_command that will be sent upon update
class MIT_command:
    """Data structure to store MIT_command that will be sent upon update"""

    def __init__(self, position, velocity, kp, kd, current):
        """
        Sets the motor state to the input.

        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            kp: Position gain
            kd: Velocity gain
            current: Current in amps
        """
        self.position = position
        self.velocity = velocity
        self.kp = kp
        self.kd = kd
        self.current = current


# motor state from the controller, uneditable named tuple
MIT_motor_state = namedtuple(
    "motor_state", "position velocity current temperature error"
)
"""
Motor state from the controller, uneditable named tuple
"""

# python-can listener object, with handler to be called upon reception of a message on the CAN bus
class motorListener(can.Listener):
    """Python-can listener object, with handler to be called upon reception of a message on the CAN bus"""

    def __init__(self, canman, motor):
        """
        Sets stores can manager and motor object references

        Args:
            canman: The CanManager object to get messages from
            motor: The TMotorCANManager object to update
        """
        self.canman = canman
        self.bus = canman.bus
        self.motor = motor

    def on_message_received(self, msg):
        """
        Updates this listener's motor with the info contained in msg, if that message was for this motor.

        args:
            msg: A python-can CAN message
        """
        data = bytes(msg.data)
        ID = data[0]
        if ID == self.motor.ID:
            self.motor._update_state_async(
                self.canman.parse_MIT_message(data, self.motor.type)
            )


# A class to manage the low level CAN communication protocols
class CAN_Manager(object):
    """A class to manage the low level CAN communication protocols"""

    debug = False
    """
    Set to true to display every message sent and recieved for debugging.
    """
    # Note, defining singletons in this way means that you cannot inherit
    # from this class, as apparently __init__ for the subclass will be called twice
    _instance = None
    """
    Used to keep track of one instantation of the class to make a singleton object
    """

    def __new__(cls):
        """
        Makes a singleton object to manage a socketcan_native CAN bus.
        """
        if not cls._instance:
            cls._instance = super(CAN_Manager, cls).__new__(cls)
            print("Initializing CAN Manager")
            # verify the CAN bus is currently down
            os.system("sudo /sbin/ip link set can0 down")
            # start the CAN bus back up
            os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
            # create a python-can bus object
            cls._instance.bus = can.interface.Bus(
                channel="can0", bustype="socketcan"
            )  # bustype='socketcan_native')
            # create a python-can notifier object, which motors can later subscribe to
            cls._instance.notifier = can.Notifier(bus=cls._instance.bus, listeners=[])
            print("Connected on: " + str(cls._instance.bus))

        return cls._instance

    def __init__(self):
        """
        ALl initialization happens in __new__
        """
        pass

    def __del__(self):
        """
        # shut down the CAN bus when the object is deleted
        # This may not ever get called, so keep a reference and explicitly delete if this is important.
        """
        os.system("sudo /sbin/ip link set can0 down")

    # subscribe a motor object to the CAN bus to be updated upon message reception
    def add_motor(self, motor):
        """
        Subscribe a motor object to the CAN bus to be updated upon message reception

        Args:
            motor: The TMotorManager object to be subscribed to the notifier
        """
        self.notifier.add_listener(motorListener(self, motor))

    # Locks value between min and max
    @staticmethod
    def limit_value(value, min, max):
        """
        Limits value to be between min and max

        Args:
            value: The value to be limited.
            min: The lowest number allowed (inclusive) for value
            max: The highest number allowed (inclusive) for value
        """
        if value >= max:
            return max
        elif value <= min:
            return min
        else:
            return value

    # interpolates a floating point number to fill some amount of the max size of unsigned int,
    # as specified with the num_bits
    @staticmethod
    def float_to_uint(x, x_min, x_max, num_bits):
        """
        Interpolates a floating point number to an unsigned integer of num_bits length.
        A number of x_max will be the largest integer of num_bits, and x_min would be 0.

        args:
            x: The floating point number to convert
            x_min: The minimum value for the floating point number
            x_max: The maximum value for the floating point number
            num_bits: The number of bits for the unsigned integer
        """
        span = x_max - x_min
        bitratio = float((1 << num_bits) / span)
        x = CAN_Manager.limit_value(x, x_min, x_max - (2 / bitratio))
        # (x - x_min)*(2^num_bits)/span

        return CAN_Manager.limit_value(
            int((x - x_min) * (bitratio)), 0, int((x_max - x_min) * bitratio)
        )

    # undoes the above method
    @staticmethod
    def uint_to_float(x, x_min, x_max, num_bits):
        """
        Interpolates an unsigned integer of num_bits length to a floating point number between x_min and x_max.

        args:
            x: The floating point number to convert
            x_min: The minimum value for the floating point number
            x_max: The maximum value for the floating point number
            num_bits: The number of bits for the unsigned integer
        """
        span = x_max - x_min
        # (x*span/(2^num_bits -1)) + x_min
        return float(x * span / ((1 << num_bits) - 1) + x_min)

    # sends a message to the motor (when the motor is in MIT mode)
    def send_MIT_message(self, motor_id, data):
        """
        Sends an MIT Mode message to the motor, with a header of motor_id and data array of data

        Args:
            motor_id: The CAN ID of the motor to send to.
            data: An array of integers or bytes of data to send.
        """
        DLC = len(data)
        assert DLC <= 8, "Data too long in message for motor " + str(motor_id)

        if self.debug:
            print(
                "ID: "
                + str(hex(motor_id))
                + "   Data: "
                + "[{}]".format(", ".join(hex(d) for d in data))
            )

        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
        try:
            self.bus.send(message)
            if self.debug:
                print("    Message sent on " + str(self.bus.channel_info))
        except can.CanError:
            if self.debug:
                print("    Message NOT sent")

    # send the power on code
    def power_on(self, motor_id):
        """
        Sends the power on code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_MIT_message(
            motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        )

    # send the power off code
    def power_off(self, motor_id):
        """
        Sends the power off code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_MIT_message(
            motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
        )

    # send the zeroing code. Like a scale, it takes about a second to zero the position
    def zero(self, motor_id):
        """
        Sends the zeroing code to motor_id. This code will shut off communication with the motor for about a second.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_MIT_message(
            motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
        )

    # send an MIT control signal, consisting of desired position, velocity, and current, and gains for position and velocity control
    # basically an impedance controller
    def MIT_controller(self, motor_id, motor_type, position, velocity, Kp, Kd, I):
        """
        Sends an MIT style control signal to the motor. This signal will be used to generate a
        current for the field-oriented controller on the motor control chip, given by this expression:

            q_control = Kp*(position - current_position) + Kd*(velocity - current_velocity) + I

        Args:
            motor_id: The CAN ID of the motor to send the message to
            motor_type: A string noting the type of motor, ie 'AK80-9'
            position: The desired position in rad
            velocity: The desired velocity in rad/s
            Kp: The position gain
            Kd: The velocity gain
            I: The additional current
        """
        position_uint16 = CAN_Manager.float_to_uint(
            position,
            MIT_Params[motor_type]["P_min"],
            MIT_Params[motor_type]["P_max"],
            16,
        )
        velocity_uint12 = CAN_Manager.float_to_uint(
            velocity,
            MIT_Params[motor_type]["V_min"],
            MIT_Params[motor_type]["V_max"],
            12,
        )
        Kp_uint12 = CAN_Manager.float_to_uint(
            Kp, MIT_Params[motor_type]["Kp_min"], MIT_Params[motor_type]["Kp_max"], 12
        )
        Kd_uint12 = CAN_Manager.float_to_uint(
            Kd, MIT_Params[motor_type]["Kd_min"], MIT_Params[motor_type]["Kd_max"], 12
        )
        I_uint12 = CAN_Manager.float_to_uint(
            I, MIT_Params[motor_type]["T_min"], MIT_Params[motor_type]["T_max"], 12
        )

        data = [
            position_uint16 >> 8,
            position_uint16 & 0x00FF,
            (velocity_uint12) >> 4,
            ((velocity_uint12 & 0x00F) << 4) | (Kp_uint12) >> 8,
            (Kp_uint12 & 0x0FF),
            (Kd_uint12) >> 4,
            ((Kd_uint12 & 0x00F) << 4) | (I_uint12) >> 8,
            (I_uint12 & 0x0FF),
        ]
        # print(data)
        self.send_MIT_message(motor_id, data)

    # convert data recieved from motor in byte format back into floating point numbers in real units
    def parse_MIT_message(self, data, motor_type):
        """
        Takes a RAW MIT message and formats it into readable floating point numbers.

        Args:
            data: the bytes of data from a python-can message object to be parsed
            motor_type: A string noting the type of motor, ie 'AK80-9'

        Returns:
            An MIT_Motor_State namedtuple that contains floating point values for the
            position, velocity, current, temperature, and error in rad, rad/s, amps, and *C.
            0 means no error.

            Notably, the current is converted to amps from the reported
            'torque' value, which is i*Kt. This allows control based on actual q-axis current,
            rather than estimated torque, which doesn't account for friction losses.
        """
        assert (
            len(data) == 8 or len(data) == 6
        ), "Tried to parse a CAN message that was not Motor State in MIT Mode"
        temp = None
        error = None
        position_uint = data[1] << 8 | data[2]
        velocity_uint = ((data[3] << 8) | (data[4] >> 4) << 4) >> 4
        current_uint = (data[4] & 0x0F) << 8 | data[5]

        if len(data) == 8:
            temp = int(data[6])
            error = int(data[7])

        position = CAN_Manager.uint_to_float(
            position_uint,
            MIT_Params[motor_type]["P_min"],
            MIT_Params[motor_type]["P_max"],
            16,
        )
        velocity = CAN_Manager.uint_to_float(
            velocity_uint,
            MIT_Params[motor_type]["V_min"],
            MIT_Params[motor_type]["V_max"],
            12,
        )
        current = CAN_Manager.uint_to_float(
            current_uint,
            MIT_Params[motor_type]["T_min"],
            MIT_Params[motor_type]["T_max"],
            12,
        )

        if self.debug:
            print("  Position: " + str(position))
            print("  Velocity: " + str(velocity))
            print("  Current: " + str(current))
            if (temp is not None) and (error is not None):
                print("  Temp: " + str(temp))
                print("  Error: " + str(error))

        # returns the Tmotor "current" which is really a torque estimate
        return MIT_motor_state(position, velocity, current, temp, error)


# defualt variables to be logged
LOG_VARIABLES = [
    "output_angle",
    "output_velocity",
    "output_acceleration",
    "current",
    "output_torque",
]

# # possible states for the controller
# class _TMotorManState(Enum):
#     """
#     An Enum to keep track of different control states
#     """
#     IDLE = 0
#     IMPEDANCE = 1
#     CURRENT = 2
#     FULL_STATE = 3
#     SPEED = 4


def _tmotor_impedance_mode_exit(tmotor_actuator: "TMotorMITCANActuator") -> None:
    tmotor_actuator.stop_motor()


def _tmotor_current_mode_exit(tmotor_actuator: "TMotorMITCANActuator") -> None:
    tmotor_actuator.stop_motor()


def _tmotor_velocity_mode_exit(tmotor_actuator: "TMotorMITCANActuator") -> None:
    tmotor_actuator.stop_motor()


TMOTOR_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    IMPEDANCE=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=_tmotor_impedance_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
    CURRENT=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=_tmotor_current_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
    VELOCITY=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=_tmotor_velocity_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
)


# the user-facing class that manages the motor.
class TMotorMITCANActuator(ActuatorBase):
    """
    The user-facing class that manages the motor. This class should be
    used in the context of a with as block, in order to safely enter/exit
    control of the motor.
    """

    def __init__(
        self,
        tag: str = "TMotorActuator",
        motor_type="AK80-9",
        motor_ID=41,
        gear_ratio: float = 1.0,
        frequency: int = 500,
        offline: bool = False,
        max_mosfett_temp=50,
        # CSV_file=None,
        # log_vars = LOG_VARIABLES
    ):
        """
        Sets up the motor manager. Note the device will not be powered on by this method! You must
        call __enter__, mostly commonly by using a with block, before attempting to control the motor.

        Args:
            motor_type: The type of motor being controlled, ie AK80-9.
            motor_ID: The CAN ID of the motor.
            max_mosfett_temp: temperature of the mosfett above which to throw an error, in Celsius
            CSV_file: A CSV file to output log info to. If None, no log will be recorded.
            log_vars: The variables to log as a python list. The full list of possibilities is
                - "output_angle"
                - "output_velocity"
                - "output_acceleration"
                - "current"
                - "output_torque"
                - "motor_angle"
                - "motor_velocity"
                - "motor_acceleration"
                - "motor_torque"
        """
        ActuatorBase.__init__(
            self,
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=TMOTOR_ACTUATOR_CONSTANTS,
            frequency=frequency,
            offline=offline,
        )
        self.type = motor_type
        self.ID = motor_ID
        # self.csv_file_name = CSV_file
        print("Initializing device: " + self.device_info_string())

        self._motor_state = motor_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._motor_state_async = motor_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._command = MIT_command(0.0, 0.0, 0.0, 0.0, 0.0)
        # self._control_state = _TMotorManState.IDLE
        self._times_past_position_limit = 0
        self._times_past_current_limit = 0
        self._times_past_velocity_limit = 0
        self._angle_threshold = (
            MIT_Params[self.type]["P_max"] - 2.0
        )  # radians, only really matters if the motor's going super fast
        self._current_threshold = (
            self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"]) - 3.0
        )  # A, only really matters if the current changes quick
        self._velocity_threshold = (
            MIT_Params[self.type]["V_max"] - 2.0
        )  # radians, only really matters if the motor's going super fast
        self._old_pos = None
        self._old_curr = 0.0
        self._old_vel = 0.0
        self._old_current_zone = 0
        self.max_temp = max_mosfett_temp  # max temp in deg C, can update later

        self._entered = False
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._last_command_time = None
        self._updated = False
        self.SF = 1.0

        self._thermal_model: ThermalModel = ThermalModel(
            temp_limit_windings=self.max_winding_temperature,
            soft_border_C_windings=10,
            temp_limit_case=self.max_case_temperature,
            soft_border_C_case=10,
        )
        self._thermal_scale: float = 1.0

        # self.log_vars = log_vars
        # self.LOG_FUNCTIONS = {
        #     "output_angle" : self.get_output_angle_radians,
        #     "output_velocity" : self.get_output_velocity_radians_per_second,
        #     "output_acceleration" : self.get_output_acceleration_radians_per_second_squared,
        #     "current" : self.get_current_qaxis_amps,
        #     "output_torque": self.get_output_torque_newton_meters,
        #     "motor_angle" : self.get_motor_angle_radians,
        #     "motor_velocity" : self.get_motor_velocity_radians_per_second,
        #     "motor_acceleration" : self.get_motor_acceleration_radians_per_second_squared,
        #     "motor_torque": self.get_motor_torque_newton_meters
        # }

        self._canman = CAN_Manager()
        self._canman.add_motor(self)

    @property
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        return TMOTOR_CONTROL_MODE_CONFIGS

    @check_actuator_connection
    def start(self):
        """
        Used to safely power the motor on and begin the log file (if specified).
        """
        print("Turning on control for device: " + self.device_info_string())
        # if self.csv_file_name is not None:
        #     with open(self.csv_file_name,'w') as fd:
        #         writer = csv.writer(fd)
        #         writer.writerow(["pi_time"]+self.log_vars)
        #     self.csv_file = open(self.csv_file_name,'a').__enter__()
        #     self.csv_writer = csv.writer(self.csv_file)

        self.power_on()
        self._send_command()
        self._entered = True
        self.is_streaming = True
        if not self.check_can_connection():
            raise RuntimeError(
                "Device not connected: " + str(self.device_info_string())
            )
        return self

    @check_actuator_stream
    @check_actuator_open
    def stop(self):
        """
        Used to safely power the motor off and close the log file (if specified).
        """
        print("Turning off control for device: " + self.device_info_string())
        self.power_off()

        # if self.csv_file_name is not None:
        #     self.csv_file.__exit__(etype, value, tb)

        # if not (etype is None):
        #     traceback.print_exception(etype, value, tb)

    def TMotor_current_to_qaxis_current(self, iTM):
        """
        Try to convert TMotor reported torque to q-axis current
        """
        return (
            MIT_Params[self.type]["Current_Factor"]
            * iTM
            / (MIT_Params[self.type]["GEAR_RATIO"] * MIT_Params[self.type]["Kt_TMotor"])
        )

    def qaxis_current_to_TMotor_current(self, iq):
        """
        Try to convert q-axis current to TMotor reported torque
        """
        return (
            iq
            * (MIT_Params[self.type]["GEAR_RATIO"] * MIT_Params[self.type]["Kt_TMotor"])
            / MIT_Params[self.type]["Current_Factor"]
        )

    # this method is called by the handler every time a message is recieved on the bus
    # from this motor, to store the most recent state information for later
    def _update_state_async(self, MIT_state):
        """
        This method is called by the handler every time a message is recieved on the bus
        from this motor, to store the most recent state information for later

        Args:
            MIT_state: The MIT_Motor_State namedtuple with the most recent motor state.

        Raises:
            RuntimeError when device sends back an error code that is not 0 (0 meaning no error)
        """
        if MIT_state.error != 0:
            raise RuntimeError(
                "Driver board error for device: "
                + self.device_info_string()
                + ": "
                + MIT_Params["ERROR_CODES"][MIT_state.error]
            )

        now = time.time()
        dt = self._last_update_time - now
        self._last_update_time = now
        acceleration = (MIT_state.velocity - self._motor_state_async.velocity) / dt

        # The "Current" supplied by the controller is actually current*Kt, which approximates torque.
        self._motor_state_async.set_state(
            MIT_state.position,
            MIT_state.velocity,
            self.TMotor_current_to_qaxis_current(MIT_state.current),
            MIT_state.temperature,
            MIT_state.error,
            acceleration,
        )

        self._updated = True

    def home(self):
        pass

    # this method is called by the user to synchronize the current state used by the controller
    # with the most recent message recieved
    def update(self):
        """
        This method is called by the user to synchronize the current state used by the controller
        with the most recent message recieved, as well as to send the current command.
        """

        # check that the motor is safely turned on
        if not self._entered:
            raise RuntimeError(
                "Tried to update motor state before safely powering on for device: "
                + self.device_info_string()
            )

        if self.case_temperature > self.max_temp:
            raise RuntimeError(
                "Temperature greater than {}C for device: {}".format(
                    self.max_temp, self.device_info_string()
                )
            )

        # check that the motor data is recent
        # print(self._command_sent)
        now = time.time()
        if (now - self._last_command_time) < 0.25 and (
            (now - self._last_update_time) > 0.1
        ):
            # print("State update requested but no data recieved from motor. Delay longer after zeroing, decrease frequency, or check connection.")
            warnings.warn(
                "State update requested but no data from motor. Delay longer after zeroing, decrease frequency, or check connection. "
                + self.device_info_string(),
                RuntimeWarning,
            )
        else:
            self._command_sent = False

        # artificially extending the range of the position, current, and velocity that we track
        P_max = MIT_Params[self.type]["P_max"] + 0.01
        I_max = (
            self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"]) + 1.0
        )
        V_max = MIT_Params[self.type]["V_max"] + 0.01

        if self._old_pos is None:
            self._old_pos = self._motor_state_async.position
        old_pos = self._old_pos
        old_curr = self._old_curr
        old_vel = self._old_vel

        new_pos = self._motor_state_async.position
        new_curr = self._motor_state_async.current
        new_vel = self._motor_state_async.velocity

        thresh_pos = self._angle_threshold
        thresh_curr = self._current_threshold
        thresh_vel = self._velocity_threshold

        curr_command = self._command.current

        actual_current = new_curr

        # The TMotor will wrap around to -max at the limits for all values it returns!! Account for this
        if (thresh_pos <= new_pos and new_pos <= P_max) and (
            -P_max <= old_pos and old_pos <= -thresh_pos
        ):
            self._times_past_position_limit -= 1
        elif (thresh_pos <= old_pos and old_pos <= P_max) and (
            -P_max <= new_pos and new_pos <= -thresh_pos
        ):
            self._times_past_position_limit += 1

        # current is basically the same as position, but if you instantly command a switch it can actually change fast enough
        # to throw this off, so that is accounted for too. We just put a hard limit on the current to solve current jitter problems.
        if (thresh_curr <= new_curr and new_curr <= I_max) and (
            -I_max <= old_curr and old_curr <= -thresh_curr
        ):
            # self._old_current_zone = -1
            # if (thresh_curr <= curr_command and curr_command <= I_max):
            #     self._times_past_current_limit -= 1
            if curr_command > 0:
                actual_current = self.TMotor_current_to_qaxis_current(
                    MIT_Params[self.type]["T_max"]
                )
            elif curr_command < 0:
                actual_current = -self.TMotor_current_to_qaxis_current(
                    MIT_Params[self.type]["T_max"]
                )
            else:
                actual_current = -self.TMotor_current_to_qaxis_current(
                    MIT_Params[self.type]["T_max"]
                )
            new_curr = actual_current
        elif (thresh_curr <= old_curr and old_curr <= I_max) and (
            -I_max <= new_curr and new_curr <= -thresh_curr
        ):
            # self._old_current_zone = 1
            # if not (-I_max <= curr_command and curr_command <= -thresh_curr):
            #     self._times_past_current_limit += 1
            if curr_command > 0:
                actual_current = self.TMotor_current_to_qaxis_current(
                    MIT_Params[self.type]["T_max"]
                )
            elif curr_command < 0:
                actual_current = -self.TMotor_current_to_qaxis_current(
                    MIT_Params[self.type]["T_max"]
                )
            else:
                actual_current = self.TMotor_current_to_qaxis_current(
                    MIT_Params[self.type]["T_max"]
                )
            new_curr = actual_current

        # velocity should work the same as position
        if (thresh_vel <= new_vel and new_vel <= V_max) and (
            -V_max <= old_vel and old_vel <= -thresh_vel
        ):
            self._times_past_velocity_limit -= 1
        elif (thresh_vel <= old_vel and old_vel <= V_max) and (
            -V_max <= new_vel and new_vel <= -thresh_vel
        ):
            self._times_past_velocity_limit += 1

        # update expanded state variables
        self._old_pos = new_pos
        self._old_curr = new_curr
        self._old_vel = new_vel

        self._motor_state.set_state_obj(self._motor_state_async)
        self._motor_state.position += (
            self._times_past_position_limit * 2 * MIT_Params[self.type]["P_max"]
        )
        self._motor_state.current = actual_current
        self._motor_state.velocity += (
            self._times_past_velocity_limit * 2 * MIT_Params[self.type]["V_max"]
        )

        # send current motor command
        self._send_command()

        # # writing to log file
        # if self.csv_file_name is not None:
        #     self.csv_writer.writerow([self._last_update_time - self._start_time] + [self.LOG_FUNCTIONS[var]() for var in self.log_vars])

        self._updated = False

    # sends a command to the motor depending on whats controlm mode the motor is in
    def _send_command(self):
        """
        Sends a command to the motor depending on whats controlm mode the motor is in. This method
        is called by update(), and should only be called on its own if you don't want to update the motor state info.

        Notably, the current is converted to amps from the reported 'torque' value, which is i*Kt.
        This allows control based on actual q-axis current, rather than estimated torque, which
        doesn't account for friction losses.
        """
        # if self._control_state == _TMotorManState.FULL_STATE:
        #     self._canman.MIT_controller(self.ID,self.type, self._command.position, self._command.velocity, self._command.kp, self._command.kd, self.qaxis_current_to_TMotor_current(self._command.current))
        if self.mode == CONTROL_MODES.IMPEDANCE:
            self._canman.MIT_controller(
                self.ID,
                self.type,
                self._command.position,
                self._command.velocity,
                self._command.kp,
                self._command.kd,
                0.0,
            )
        elif self.mode == CONTROL_MODES.CURRENT:
            self._canman.MIT_controller(
                self.ID,
                self.type,
                0.0,
                0.0,
                0.0,
                0.0,
                self.qaxis_current_to_TMotor_current(self._command.current),
            )
        elif self.mode == CONTROL_MODES.IDLE:
            self._canman.MIT_controller(self.ID, self.type, 0.0, 0.0, 0.0, 0.0, 0.0)
        elif self.mode == CONTROL_MODES.VELOCITY:
            self._canman.MIT_controller(
                self.ID,
                self.type,
                0.0,
                self._command.velocity,
                0.0,
                self._command.kd,
                0.0,
            )
        else:
            raise RuntimeError(
                "UNDEFINED STATE for device " + self.device_info_string()
            )
        self._last_command_time = time.time()

    # Basic Motor Utility Commands
    def power_on(self):
        """Powers on the motor. You may hear a faint hiss."""
        self._canman.power_on(self.ID)
        self._updated = True

    def power_off(self):
        """Powers off the motor."""
        self._canman.power_off(self.ID)

    # zeros the position, like a scale you have to wait about a second before you can
    # use the motor again. This responsibility is on the user!!
    def set_zero_position(self):
        """Zeros the position--like a scale you have to wait about a second before you can
        use the motor again. This responsibility is on the user!!"""
        self._canman.zero(self.ID)
        self._last_command_time = time.time()

    # getters for motor state
    @property
    def case_temperature(self):
        """
        Returns:
        The most recently updated motor temperature in degrees C.
        """
        return self._motor_state.temperature

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

    def get_motor_error_code(self):
        """
        Returns:
        The most recently updated motor error code.
        Note the program should throw a runtime error before you get a chance to read
        this value if it is ever anything besides 0.

        Codes:
        - 0 : 'No Error',
        - 1 : 'Over temperature fault',
        - 2 : 'Over current fault',
        - 3 : 'Over voltage fault',
        - 4 : 'Under voltage fault',
        - 5 : 'Encoder fault',
        - 6 : 'Phase current unbalance fault (The hardware may be damaged)'
        """
        return self._motor_state.error

    @property
    def motor_current(self):
        """
        Returns:
        The most recently updated qaxis current in amps
        """
        return self._motor_state.current

    @property
    def motor_voltage(self):
        # Not implemented
        return 0.0

    @property
    def output_position(self):
        """
        Returns:
        The most recently updated output angle in radians
        """
        return self._motor_state.position

    @property
    def output_velocity(self):
        """
        Returns:
            The most recently updated output velocity in radians per second
        """
        return self._motor_state.velocity

    @property
    def output_acceleration(self):
        """
        Returns:
            The most recently updated output acceleration in radians per second per second
        """
        return self._motor_state.acceleration

    @property
    def output_torque(self):
        """
        Returns:
            the most recently updated output torque in Nm
        """
        return (
            self.motor_current
            * MIT_Params[self.type]["Kt_actual"]
            * MIT_Params[self.type]["GEAR_RATIO"]
        )

    # uses plain impedance mode, will send 0.0 for current command.
    def set_impedance_gains(self, kp=0, ki=0, K=0.08922, B=0.0038070, ff=0):
        """
        Uses plain impedance mode, will send 0.0 for current command in addition to position request.

        Args:
            kp: A dummy argument for backward compatibility with the dephy library.
            ki: A dummy argument for backward compatibility with the dephy library.
            K: The stiffness in Nm/rad
            B: The damping in Nm/(rad/s)
            ff: A dummy argument for backward compatibility with the dephy library.
        """
        assert (
            isfinite(K)
            and MIT_Params[self.type]["Kp_min"] <= K
            and K <= MIT_Params[self.type]["Kp_max"]
        )
        assert (
            isfinite(B)
            and MIT_Params[self.type]["Kd_min"] <= B
            and B <= MIT_Params[self.type]["Kd_max"]
        )
        self._command.kp = K
        self._command.kd = B
        self._command.velocity = 0.0
        # self._control_state = _TMotorManState.IMPEDANCE

    # # uses full MIT mode, will send whatever current command is set.
    # def set_impedance_gains_real_unit_full_state_feedback(self, kp=0, ki=0, K=0.08922, B=0.0038070, ff=0):
    #     """"
    #     Uses full state feedback mode, will send whatever current command is set in addition to position request.

    #     Args:
    #         kp: A dummy argument for backward compatibility with the dephy library.
    #         ki: A dummy argument for backward compatibility with the dephy library.
    #         K: The stiffness in Nm/rad
    #         B: The damping in Nm/(rad/s)
    #         ff: A dummy argument for backward compatibility with the dephy library."""
    #     assert(isfinite(K) and MIT_Params[self.type]["Kp_min"] <= K and K <= MIT_Params[self.type]["Kp_max"])
    #     assert(isfinite(B) and MIT_Params[self.type]["Kd_min"] <= B and B <= MIT_Params[self.type]["Kd_max"])
    #     self._command.kp = K
    #     self._command.kd = B
    #     self._control_state = _TMotorManState.FULL_STATE

    # uses plain current mode, will send 0.0 for position gains.
    def set_current_gains(self, kp=40, ki=400, ff=128, spoof=False):
        """
        Uses plain current mode, will send 0.0 for position gains in addition to requested current.

        Args:
            kp: A dummy argument for backward compatibility with the dephy library.
            ki: A dummy argument for backward compatibility with the dephy library.
            ff: A dummy argument for backward compatibility with the dephy library.
            spoof: A dummy argument for backward compatibility with the dephy library.
        """
        pass
        # self._control_state = _TMotorManState.CURRENT

    def set_velocity_gains(self, kd=1.0):
        """
        Uses plain speed mode, will send 0.0 for position gain and for feed forward current.

        Args:
            kd: The gain for the speed controller. Control law will be (v_des - v_actual)*kd = iq
        """
        self._command.kd = kd
        # self._control_state = _TMotorManState.SPEED

    def set_position_gains(self):
        # Not implemented
        pass

    # used for either impedance or MIT mode to set output angle
    def set_output_position(self, value):
        """
        Used for either impedance or full state feedback mode to set output angle command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            pos: The desired output position in rads
        """
        # position commands must be within a certain range :/
        # pos = (np.abs(pos) % MIT_Params[self.type]["P_max"])*np.sign(pos) # this doesn't work because it will unwind itself!
        # CANNOT Control using impedance mode for angles greater than 12.5 rad!!
        if np.abs(value) >= MIT_Params[self.type]["P_max"]:
            raise RuntimeError(
                "Cannot control using impedance mode for angles with magnitude greater than "
                + str(MIT_Params[self.type]["P_max"])
                + "rad!"
            )

        # if self._control_state not in [_TMotorManState.IMPEDANCE, _TMotorManState.FULL_STATE]:
        #     raise RuntimeError("Attempted to send position command without gains for device " + self.device_info_string())
        self._command.position = value

    def set_output_velocity(self, value):
        """
        Used for either speed or full state feedback mode to set output velocity command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            vel: The desired output speed in rad/s
        """
        if np.abs(value) >= MIT_Params[self.type]["V_max"]:
            raise RuntimeError(
                "Cannot control using speed mode for angles with magnitude greater than "
                + str(MIT_Params[self.type]["V_max"])
                + "rad/s!"
            )

        # if self._control_state not in [_TMotorManState.SPEED, _TMotorManState.FULL_STATE]:
        #     raise RuntimeError("Attempted to send speed command without gains for device " + self.device_info_string())
        self._command.velocity = value

    # used for either current MIT mode to set current

    def set_motor_current(self, value):
        """
        Used for either current or full state feedback mode to set current command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            current: the desired current in amps.
        """
        # if self._control_state not in [_TMotorManState.CURRENT, _TMotorManState.FULL_STATE]:
        #     raise RuntimeError("Attempted to send current command before entering current mode for device " + self.device_info_string())
        self._command.current = value

    # used for either current or MIT Mode to set current, based on desired torque
    def set_joint_torque(self, value):
        """
        Used for either current or MIT Mode to set current, based on desired torque.
        If a more complicated torque model is available for the motor, that will be used.
        Otherwise it will just use the motor's torque constant.

        Args:
            torque: The desired output torque in Nm.
        """
        self.set_motor_current(
            (
                value
                / MIT_Params[self.type]["Kt_actual"]
                / MIT_Params[self.type]["GEAR_RATIO"]
            )
        )

    # motor-side functions to account for the gear ratio
    def set_motor_torque(self, value):
        """
        Version of set_output_torque that accounts for gear ratio to control motor-side torque

        Args:
            torque: The desired motor-side torque in Nm.
        """
        self.set_output_torque(value * MIT_Params[self.type]["Kt_actual"])

    def set_motor_position(self, value):
        """
        Wrapper for set_output_angle that accounts for gear ratio to control motor-side angle

        Args:
            pos: The desired motor-side position in rad.
        """
        self.set_output_position(value / (MIT_Params[self.type]["GEAR_RATIO"]))

    def set_motor_velocity(self, value):
        """
        Wrapper for set_output_velocity that accounts for gear ratio to control motor-side velocity

        Args:
            vel: The desired motor-side velocity in rad/s.
        """
        self.set_output_velocity(value / (MIT_Params[self.type]["GEAR_RATIO"]))

    def set_motor_voltage(self, value):
        # Not implemented
        pass

    @property
    def motor_position(self):
        """
        Wrapper for get_output_angle that accounts for gear ratio to get motor-side angle

        Returns:
            The most recently updated motor-side angle in rad.
        """
        return self._motor_state.position * MIT_Params[self.type]["GEAR_RATIO"]

    @property
    def motor_velocity(self):
        """
        Wrapper for get_output_velocity that accounts for gear ratio to get motor-side velocity

        Returns:
            The most recently updated motor-side velocity in rad/s.
        """
        return self._motor_state.velocity * MIT_Params[self.type]["GEAR_RATIO"]

    @property
    def motor_acceleration(self):
        """
        Wrapper for get_output_acceleration that accounts for gear ratio to get motor-side acceleration

        Returns:
            The most recently updated motor-side acceleration in rad/s/s.
        """
        return self._motor_state.acceleration * MIT_Params[self.type]["GEAR_RATIO"]

    @property
    def motor_torque(self):
        """
        Wrapper for get_output_torque that accounts for gear ratio to get motor-side torque

        Returns:
            The most recently updated motor-side torque in Nm.
        """
        return self.output_torque * MIT_Params[self.type]["GEAR_RATIO"]

    # Pretty stuff
    def __str__(self):
        """Prints the motor's device info and current"""
        return (
            self.device_info_string()
            + " | Position: "
            + "{: 1f}".format(round(self.output_angle, 3))
            + " rad | Velocity: "
            + "{: 1f}".format(round(self.output_velocity, 3))
            + " rad/s | current: "
            + "{: 1f}".format(round(self.motor_current, 3))
            + " A | torque: "
            + "{: 1f}".format(round(self.output_torque, 3))
            + " Nm"
        )

    def device_info_string(self):
        """Prints the motor's ID and device type."""
        return str(self.type) + "  ID: " + str(self.ID)

    # Checks the motor connection by sending a 10 commands and making sure the motor responds.
    def check_can_connection(self):
        """
        Checks the motor's connection by attempting to send 10 startup messages.
        If it gets 10 replies, then the connection is confirmed.

        Returns:
            True if a connection is established and False otherwise.
        """
        if not self._entered:
            raise RuntimeError(
                "Tried to check_can_connection before entering motor control! Enter control using the __enter__ method, or instantiating the TMotorManager in a with block."
            )
        Listener = can.BufferedReader()
        self._canman.notifier.add_listener(Listener)
        for i in range(10):
            self.power_on()
            time.sleep(0.001)
        success = True
        self.is_open = True
        time.sleep(0.1)
        for i in range(10):
            if Listener.get_message(timeout=0.1) is None:
                success = False
                self.is_open = False
        self._canman.notifier.remove_listener(Listener)
        return success

    # controller variables
    temperature = property(case_temperature, doc="temperature_degrees_C")
    """Temperature in Degrees Celsius"""

    error = property(get_motor_error_code, doc="temperature_degrees_C")
    """Motor error code. 0 means no error."""

    # electrical variables
    current_qaxis = property(
        motor_current, set_motor_current, doc="current_qaxis_amps_current_only"
    )
    """Q-axis current in amps"""

    # output-side variables
    position = property(
        output_position, set_output_position, doc="output_angle_radians_impedance_only"
    )
    """Output angle in rad"""

    velocity = property(
        output_velocity, set_output_velocity, doc="output_velocity_radians_per_second"
    )
    """Output velocity in rad/s"""

    acceleration = property(
        output_acceleration, doc="output_acceleration_radians_per_second_squared"
    )
    """Output acceleration in rad/s/s"""

    torque = property(
        output_torque, set_joint_torque, doc="output_torque_newton_meters"
    )
    """Output torque in Nm"""

    # motor-side variables
    position_motorside = property(
        motor_position, set_motor_position, doc="motor_angle_radians_impedance_only"
    )
    """Motor-side angle in rad"""

    velocity_motorside = property(
        motor_velocity, set_motor_velocity, doc="motor_velocity_radians_per_second"
    )
    """Motor-side velocity in rad/s"""

    acceleration_motorside = property(
        motor_acceleration, doc="motor_acceleration_radians_per_second_squared"
    )
    """Motor-side acceleration in rad/s/s"""

    torque_motorside = property(
        motor_torque, set_motor_torque, doc="motor_torque_newton_meters"
    )
    """Motor-side torque in Nm"""


if __name__ == "__main__":
    with TMotorMITCANActuator(motor_type="AK80-9", motor_ID=41) as dev:
        dev.set_zero_position()  # has a delay!
        time.sleep(1.5)
        dev.set_control_mode(CONTROL_MODES.IMPEDANCE)
        dev.set_impedance_gains(K=10, B=0.5)

        print("Starting position step demo. Press ctrl+C to quit.")

        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)
        for t in loop:
            dev.update()
            if t < 1.0:
                dev.set_motor_position(0.0)
            else:
                dev.set_motor_position(10)

            print(
                "Actual: ",
                round(dev.output_position, 3),
                "Desired: ",
                dev._command.position,
            )

        del loop
