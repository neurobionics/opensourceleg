import time
import warnings
from math import isfinite
from typing import Optional

import can
import numpy as np
from TMotorCANControl.mit_can import (
    CAN_Manager,
    MIT_command,
    MIT_Params,
    TMotorManager_mit_can,
    motor_state,
)

from opensourceleg.actuators.base import (
    CONTROL_MODE_CONFIGS,
    CONTROL_MODES,
    MOTOR_CONSTANTS,
    ActuatorBase,
    ControlModeConfig,
)
from opensourceleg.actuators.decorators import (
    check_actuator_connection,
    check_actuator_open,
    check_actuator_stream,
)
from opensourceleg.math import ThermalModel
from opensourceleg.utilities import SoftRealtimeLoop

TMOTOR_ACTUATOR_CONSTANTS = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=16384,
    NM_PER_AMP=0.1133,
    NM_PER_RAD_TO_K=0.0,  # TODO: Find value
    NM_S_PER_RAD_TO_B=0.0,  # TODO: Find value
    MAX_CASE_TEMPERATURE=80,
    MAX_WINDING_TEMPERATURE=110,
)


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
class TMotorMITCANActuator(ActuatorBase, TMotorManager_mit_can):
    """
    The user-facing class that manages the motor. This class should be
    used in the context of a with as block, in order to safely enter/exit
    control of the motor.
    """

    def __init__(
        self,
        tag: str = "TMotorActuator",
        motor_type: str = "AK80-9",
        motor_ID: int = 41,
        gear_ratio: float = 1.0,
        frequency: int = 500,
        offline: bool = False,
        max_mosfett_temp: float = 50,
    ):
        """
        Sets up the motor manager. Note the device will not be powered on by this method! You must
        call __enter__, mostly commonly by using a with block, before attempting to control the motor.

        Args:
            tag: A string tag to identify the motor
            motor_type: The type of motor to control. Must be a key in MIT_Params.
            motor_ID: The ID of the motor to control.
            gear_ratio: The gear ratio of the motor. Default is 1.0.
            frequency: The frequency at which to send commands to the motor. Default is 500.
            offline: Whether to run the motor in offline mode. Default is False.
            max_mosfett_temp: The maximum temperature of the mosfet in degrees C. Default is 50.
        """
        ActuatorBase.__init__(
            self,
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=TMOTOR_ACTUATOR_CONSTANTS,
            frequency=frequency,
            offline=offline,
        )
        TMotorManager_mit_can.__init__(
            self,
            motor_type=motor_type,
            motor_ID=motor_ID,
            max_mosfett_temp=max_mosfett_temp,
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

        self.power_on()
        self._send_command()
        self._entered = True
        self.is_streaming = True
        if not self.check_can_connection():
            raise RuntimeError("Device not connected: " + str(self.device_info_string()))
        return self

    @check_actuator_stream
    @check_actuator_open
    def stop(self):
        """
        Used to safely power the motor off and close the log file (if specified).
        """
        print("Turning off control for device: " + self.device_info_string())
        self.power_off()

    def home(
        self,
        homing_voltage: int = 2000,
        homing_frequency: Optional[int] = None,
        homing_direction: int = -1,
        output_position_offset: float = 0.0,
        current_threshold: int = 5000,
        velocity_threshold: float = 0.001,
    ):
        pass

    def update(self):  # noqa: C901
        """
        This method is called by the user to synchronize the current state used by the controller
        with the most recent message recieved, as well as to send the current command.
        """

        # check that the motor is safely turned on
        if not self._entered:
            raise RuntimeError(
                "Tried to update motor state before safely powering on for device: " + self.device_info_string()
            )

        if self.case_temperature > self.max_temp:
            raise RuntimeError(f"Temperature greater than {self.max_temp}C for device: {self.device_info_string()}")

        # check that the motor data is recent
        # print(self._command_sent)
        now = time.time()
        if (now - self._last_command_time) < 0.25 and ((now - self._last_update_time) > 0.1):
            warnings.warn(
                "State update requested but no data from motor. Delay longer after zeroing, \
                decrease frequency, or check connection. "
                + self.device_info_string(),
                RuntimeWarning,
                stacklevel=2,
            )
        else:
            self._command_sent = False

        # artificially extending the range of the position, current, and velocity that we track
        P_max = MIT_Params[self.type]["P_max"] + 0.01
        I_max = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"]) + 1.0
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
        if (thresh_pos <= new_pos and new_pos <= P_max) and (-P_max <= old_pos and old_pos <= -thresh_pos):
            self._times_past_position_limit -= 1
        elif (thresh_pos <= old_pos and old_pos <= P_max) and (-P_max <= new_pos and new_pos <= -thresh_pos):
            self._times_past_position_limit += 1

        # current is basically the same as position, but if you instantly
        # command a switch it can actually change fast enough
        # to throw this off, so that is accounted for too. We just put a hard limit on the current
        # to solve current jitter problems.
        if (thresh_curr <= new_curr and new_curr <= I_max) and (-I_max <= old_curr and old_curr <= -thresh_curr):
            # self._old_current_zone = -1
            # if (thresh_curr <= curr_command and curr_command <= I_max):
            #     self._times_past_current_limit -= 1
            if curr_command > 0:
                actual_current = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            elif curr_command < 0:
                actual_current = -self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            else:
                actual_current = -self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            new_curr = actual_current
        elif (thresh_curr <= old_curr and old_curr <= I_max) and (-I_max <= new_curr and new_curr <= -thresh_curr):
            # self._old_current_zone = 1
            # if not (-I_max <= curr_command and curr_command <= -thresh_curr):
            #     self._times_past_current_limit += 1
            if curr_command > 0:
                actual_current = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            elif curr_command < 0:
                actual_current = -self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            else:
                actual_current = self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"])
            new_curr = actual_current

        # velocity should work the same as position
        if (thresh_vel <= new_vel and new_vel <= V_max) and (-V_max <= old_vel and old_vel <= -thresh_vel):
            self._times_past_velocity_limit -= 1
        elif (thresh_vel <= old_vel and old_vel <= V_max) and (-V_max <= new_vel and new_vel <= -thresh_vel):
            self._times_past_velocity_limit += 1

        # update expanded state variables
        self._old_pos = new_pos
        self._old_curr = new_curr
        self._old_vel = new_vel

        self._motor_state.set_state_obj(self._motor_state_async)
        self._motor_state.position += self._times_past_position_limit * 2 * MIT_Params[self.type]["P_max"]
        self._motor_state.current = actual_current
        self._motor_state.velocity += self._times_past_velocity_limit * 2 * MIT_Params[self.type]["V_max"]

        # send current motor command
        self._send_command()
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
            raise RuntimeError("UNDEFINED STATE for device " + self.device_info_string())
        self._last_command_time = time.time()

    # getters for motor state
    @property
    def case_temperature(self) -> float:
        """
        Returns:
        The most recently updated motor temperature in degrees C.
        """
        return float(self._motor_state.temperature)

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
    def motor_current(self) -> float:
        """
        Returns:
        The most recently updated qaxis current in amps
        """
        return float(self._motor_state.current)

    @property
    def motor_voltage(self) -> float:
        # Not implemented
        return 0.0

    @property
    def output_position(self) -> float:
        """
        Returns:
        The most recently updated output angle in radians
        """
        return float(self._motor_state.position)

    @property
    def output_velocity(self) -> float:
        """
        Returns:
            The most recently updated output velocity in radians per second
        """
        return float(self._motor_state.velocity)

    @property
    def output_acceleration(self) -> float:
        """
        Returns:
            The most recently updated output acceleration in radians per second per second
        """
        return float(self._motor_state.acceleration)

    @property
    def output_torque(self) -> float:
        """
        Returns:
            the most recently updated output torque in Nm
        """
        return float(self.motor_current * MIT_Params[self.type]["Kt_actual"] * MIT_Params[self.type]["GEAR_RATIO"])

    # uses plain impedance mode, will send 0.0 for current command.
    def set_impedance_gains(
        self,
        kp: float = 0,
        ki: float = 0,
        K: float = 0.08922,
        B: float = 0.0038070,
        ff: float = 0,
    ) -> None:
        """
        Uses plain impedance mode, will send 0.0 for current command in addition to position request.

        Args:
            kp: A dummy argument for backward compatibility with the dephy library.
            ki: A dummy argument for backward compatibility with the dephy library.
            K: The stiffness in Nm/rad
            B: The damping in Nm/(rad/s)
            ff: A dummy argument for backward compatibility with the dephy library.
        """
        if not (isfinite(K) and MIT_Params[self.type]["Kp_min"] <= K and MIT_Params[self.type]["Kp_max"] >= K):
            raise ValueError(
                f"K must be finite and between \
                {MIT_Params[self.type]['Kp_min']} and {MIT_Params[self.type]['Kp_max']}"
            )

        if not (isfinite(B) and MIT_Params[self.type]["Kd_min"] <= B and MIT_Params[self.type]["Kd_max"] >= B):
            raise ValueError(
                f"B must be finite and between \
                {MIT_Params[self.type]['Kd_min']} and {MIT_Params[self.type]['Kd_max']}"
            )

        self._command.kp = K
        self._command.kd = B
        self._command.velocity = 0.0

    def set_current_gains(
        self,
        kp: float = 40,
        ki: float = 400,
        ff: float = 128,
        spoof: bool = False,
    ) -> None:
        """
        Uses plain current mode, will send 0.0 for position gains in addition to requested current.

        Args:
            kp: A dummy argument for backward compatibility with the dephy library.
            ki: A dummy argument for backward compatibility with the dephy library.
            ff: A dummy argument for backward compatibility with the dephy library.
            spoof: A dummy argument for backward compatibility with the dephy library.
        """
        pass

    def set_velocity_gains(
        self,
        kd: float = 1.0,
    ) -> None:
        """
        Uses plain speed mode, will send 0.0 for position gain and for feed forward current.

        Args:
            kd: The gain for the speed controller. Control law will be (v_des - v_actual)*kd = iq
        """
        self._command.kd = kd

    def set_position_gains(self) -> None:
        # Not implemented
        pass

    # used for either impedance or MIT mode to set output angle
    def set_output_position(self, value: float) -> None:
        """
        Used for either impedance or full state feedback mode to set output angle command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            value: The desired output position in rads

        Raises:
            RuntimeError: If the position command is outside the range of the motor.
        """
        if np.abs(value) >= MIT_Params[self.type]["P_max"]:
            raise RuntimeError(
                "Cannot control using impedance mode for angles with magnitude greater than "
                + str(MIT_Params[self.type]["P_max"])
                + "rad!"
            )

        self._command.position = value

    def set_output_velocity(self, value: float) -> None:
        """
        Used for either speed or full state feedback mode to set output velocity command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            value: The desired output speed in rad/s

        Raises:
            RuntimeError: If the velocity command is outside the range of the motor.
        """
        if np.abs(value) >= MIT_Params[self.type]["V_max"]:
            raise RuntimeError(
                "Cannot control using speed mode for angles with magnitude greater than "
                + str(MIT_Params[self.type]["V_max"])
                + "rad/s!"
            )

        self._command.velocity = value

    # used for either current MIT mode to set current

    def set_motor_current(self, value: float) -> None:
        """
        Used for either current or full state feedback mode to set current command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            value: the desired current in amps.
        """
        self._command.current = value

    # used for either current or MIT Mode to set current, based on desired torque
    def set_output_torque(self, value: float) -> None:
        """
        Used for either current or MIT Mode to set current, based on desired torque.
        If a more complicated torque model is available for the motor, that will be used.
        Otherwise it will just use the motor's torque constant.

        Args:
            value: The desired output torque in Nm.
        """
        self.set_motor_current(value / MIT_Params[self.type]["Kt_actual"] / MIT_Params[self.type]["GEAR_RATIO"])

    # motor-side functions to account for the gear ratio
    def set_motor_torque(self, value: float) -> None:
        """
        Version of set_output_torque that accounts for gear ratio to control motor-side torque

        Args:
            value: The desired motor torque in Nm.
        """
        self.set_output_torque(value * MIT_Params[self.type]["Kt_actual"])

    def set_motor_position(self, value: float) -> None:
        """
        Wrapper for set_output_angle that accounts for gear ratio to control motor-side angle

        Args:
            value: The desired motor position in rad.
        """
        self.set_output_position(value / (MIT_Params[self.type]["GEAR_RATIO"]))

    def set_motor_velocity(self, value: float) -> None:
        """
        Wrapper for set_output_velocity that accounts for gear ratio to control motor-side velocity

        Args:
            value: The desired motor velocity in rad/s.
        """
        self.set_output_velocity(value / (MIT_Params[self.type]["GEAR_RATIO"]))

    def set_motor_voltage(self, value: float) -> float:
        # Not implemented
        pass

    @property
    def motor_position(self) -> float:
        """
        Wrapper for get_output_angle that accounts for gear ratio to get motor-side angle

        Returns:
            The most recently updated motor-side angle in rad.
        """
        return float(self._motor_state.position * MIT_Params[self.type]["GEAR_RATIO"])

    @property
    def motor_velocity(self) -> float:
        """
        Wrapper for get_output_velocity that accounts for gear ratio to get motor-side velocity

        Returns:
            The most recently updated motor-side velocity in rad/s.
        """
        return float(self._motor_state.velocity * MIT_Params[self.type]["GEAR_RATIO"])

    @property
    def motor_acceleration(self) -> float:
        """
        Wrapper for get_output_acceleration that accounts for gear ratio to get motor-side acceleration

        Returns:
            The most recently updated motor-side acceleration in rad/s/s.
        """
        return float(self._motor_state.acceleration * MIT_Params[self.type]["GEAR_RATIO"])

    @property
    def motor_torque(self) -> float:
        """
        Wrapper for get_output_torque that accounts for gear ratio to get motor-side torque

        Returns:
            The most recently updated motor-side torque in Nm.
        """
        return float(self.output_torque * MIT_Params[self.type]["GEAR_RATIO"])

    # Pretty stuff
    def __str__(self) -> str:
        """Prints the motor's device info and current"""
        return (
            self.device_info_string()
            + " | Position: "
            + f"{round(self.output_angle, 3): 1f}"
            + " rad | Velocity: "
            + f"{round(self.output_velocity, 3): 1f}"
            + " rad/s | current: "
            + f"{round(self.motor_current, 3): 1f}"
            + " A | torque: "
            + f"{round(self.output_torque, 3): 1f}"
            + " Nm"
        )

    # Checks the motor connection by sending a 10 commands and making sure the motor responds.
    def check_can_connection(self) -> bool:
        """
        Checks the motor's connection by attempting to send 10 startup messages.
        If it gets 10 replies, then the connection is confirmed.

        Returns:
            True if a connection is established and False otherwise.

        Raises:
            RuntimeError: If the motor control has not been entered.
        """
        if not self._entered:
            raise RuntimeError(
                "Tried to check_can_connection before entering motor control! \
                Enter control using the __enter__ method, or instantiating the TMotorManager in a with block."
            )
        Listener = can.BufferedReader()
        self._canman.notifier.add_listener(Listener)
        for _i in range(10):
            self.power_on()
            time.sleep(0.001)
        success = True
        self._is_open = True
        time.sleep(0.1)
        for _i in range(10):
            if Listener.get_message(timeout=0.1) is None:
                success = False
                self._is_open = False
        self._canman.notifier.remove_listener(Listener)
        return success


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
