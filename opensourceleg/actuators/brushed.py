import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

import numpy as np
from gpiozero import OutputDevice, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

from opensourceleg.actuators.base import (
    CONTROL_MODE_CONFIGS,
    MOTOR_CONSTANTS,
    ControlModeConfig,
    PositionControlActuatorBase,
)
from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import EncoderCounterBase

# Maxon x VNH7070AY specifications
MAXON_MODELS: dict[str, dict[str, Any]] = {
    "B7E883374A15": {
        "Curr_min": None,  # A
        "Curr_max": None,  # A
        "GEAR_RATIO": 6.6,
    },
}


@dataclass
class BrushedMotorState:
    """Motor state data structure"""

    position: float = 0.0  # degrees
    velocity: float = 0.0  # RPM
    current: float = 0.0  # milliamps
    temperature: float = 0.0  # celsius
    error: int = 0


# Simplified unit conversion functions
def degrees_to_radians(degrees: float) -> float:
    """
    Convert degrees to radians

    Args:
        degrees (float): Angle in degrees.

    Returns:
        float: Angle in radians.
    """
    return degrees * np.pi / 180.0


def radians_to_degrees(radians: float) -> float:
    """
    Convert radians to degrees

    Args:
        radians (float): Angle in radians.

    Returns:
        float: Angle in degrees.
    """
    return radians * 180.0 / np.pi


def _maxon_position_mode_entry(maxon_actuator: "MaxonActuator") -> None:
    LOGGER.debug(msg=f"[{maxon_actuator.tag}]  Entering Position control mode.")


def _maxon_position_mode_exit(maxon_actuator: "MaxonActuator") -> None:
    LOGGER.debug(msg=f"[{maxon_actuator.tag}]  Exiting Position control mode.")
    maxon_actuator.stop()


MAXON_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    POSITION=ControlModeConfig(
        entry_callback=_maxon_position_mode_entry,
        exit_callback=_maxon_position_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
    CURRENT=None,  # CURRENT mode not supported
    VELOCITY=None,  # VELOCITY mode not supported
    IDLE=None,  # IDLE mode not supported
    IMPEDANCE=None,  # IMPEDANCE mode not supported
    VOLTAGE=None,  # VOLTAGE mode not supported
)


class MaxonActuator(PositionControlActuatorBase):
    """
    Class for controlling a Maxon brushed motor with the VNH7070AY driver.

    Supports position control via PID and direct PWM commands.
    Current, voltage, impedance, and velocity control modes are not supported.
    """

    def __init__(
        self,
        enable_pin: int = 12,
        ina_pin: int = 24,
        inb_pin: int = 25,
        gear_ratio: float = 6.6,
        frequency: int = 6000,
        offline: bool = False,
        pwm_maximum_command: float = 0.3,
        pwm_minimum_command: float = 0.07,
        pwm_lower_limit: float = 0.02,
        tag: str = "maxon_actuator",
        motor_constants: Optional[MOTOR_CONSTANTS] = None,
    ) -> None:
        """
        Initialize Maxon motor.

        Args:
            enable_pin (int): GPIO pin number for the PWM enable signal. Defaults is 12.
            ina_pin (int): GPIO pin number for motor direction input A. Defaults is 24.
            inb_pin (int): GPIO pin number for motor direction input B. Defaults is 25.
            gear_ratio (float): Gearbox reduction ratio. Defaults is 6.6.
            frequency (int): PWM frequency in Hz. Defaults is 6000.
            offline (bool): If True, skips GPIO initialization. Defaults is False.
            pwm_maximum_command (float): Maximum allowable PWM duty cycle. Defaults is 0.3.
            pwm_minimum_command (float): Minimum PWM duty cycle that produces motion. Defaults is 0.07.
            pwm_lower_limit (float): PWM values below this threshold are set to zero. Defaults is 0.02.
            tag (str): Human-readable identifier for this actuator instance. Defaults is "maxon_actuator".
            motor_constants (optional): Motor constant parameters. Defaults is None.
        """
        if motor_constants is None:
            motor_constants = MOTOR_CONSTANTS(
                MOTOR_COUNT_PER_REV=1024,
                NM_PER_AMP=0.00652,
                MAX_CASE_TEMPERATURE=85.0,
                MAX_WINDING_TEMPERATURE=125.0,
            )

        super().__init__(
            gear_ratio=gear_ratio,
            offline=offline,
            tag=tag,
            motor_constants=motor_constants,
            frequency=frequency,
        )

        self.enable_pin = enable_pin
        self.ina_pin = ina_pin
        self.inb_pin = inb_pin

        self.pwm_maximum_command = pwm_maximum_command
        self.pwm_minimum_command = pwm_minimum_command
        self.pwm_lower_limit = pwm_lower_limit

        if not self._is_offline:
            self._factory = LGPIOFactory()

            self.speed_control = PWMOutputDevice(self.enable_pin, frequency=8000, initial_value=0)
            self.inb = OutputDevice(self.inb_pin, initial_value=False)
            self.ina = OutputDevice(self.ina_pin, initial_value=False)
            LOGGER.info("Initialized Maxon x VNH7070AY.")
        else:
            LOGGER.info("Called brushed motor initialization in offline mode.")

    @property
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        return MAXON_CONTROL_MODE_CONFIGS

    def start(self) -> None:
        """
        Not supported.
        """
        pass

    def stop(self) -> None:
        """Stops the motor by setting PWM to zero and disabling direction outputs."""
        self.speed_control.value = 0
        self.ina.off()
        self.inb.off()

    def update(self) -> None:
        """Updates the actuator's data with encoder counter reading."""
        if self.encoder_counter:
            self.motor_position_cts = self.encoder_counter.count
        else:
            self.motor_position_cts = 0.0
        self.motor_position_mm = self.cts_to_mm(self.motor_position_cts)
        self.motor_position_perc = self.cts_to_perc(self.motor_position_cts)

    def set_motor_position(self, value: float = 0.0) -> None:
        """Set the motor position. Not yet supported by this library."""
        raise NotImplementedError("Set motor position not implemented. Control the motor by setting PWM.")

    def set_output_impedance(self, value: float = 0.0) -> None:
        """Set the output impedance. Not yet supported by this library."""
        raise NotImplementedError("Set output impedance not implemented. Control the motor by setting PWM.")

    def set_impedance_gains(self, k: float, b: float) -> None:
        """Set impedance control gains. Not yet supported by this library."""
        raise NotImplementedError("Set impedance gains not implemented. Motor should be controlled by position or pwm.")

    def set_position_gains(self, k_p: float = 0.015, k_i: float = 2, k_d: float = 0.0001, ff: float = 0.0) -> None:
        """
        Set PID gains for position control.

        Default values are tuned for the VSO configuration. Other actuator
        or mechanism configurations will likely need retuning.

        Args:
            k_p: Proportional gain.
            k_i: Integral gain.
            k_d: Derivative gain.
            ff: Feedforward gain.
        """
        self.k_p = k_p  # Proportional gain
        self.k_i = k_i  # Integral gain
        self.k_d = k_d  # Derivative gain

    def _set_impedance_gains(self, k: float = 0.0, b: float = 0.0) -> None:
        """Set impedance control gains. Not yet supported by this library."""
        raise NotImplementedError("Set impedance gains not implemented. Motor should be controlled by position or pwm.")

    def home(  # type: ignore[override]
        self,
        homing_pwm: float = 0.25,
        sample_rate: float = 0.05,
        position_threshold: int = 200,
        home_zero: bool = True,
        timeout_s: float = 8.0,
        callback: Optional[Callable[[], None]] = None,
    ) -> None:
        """
        Home the actuator by driving to a mechanical hard stop.

        Moves the motor at a fixed PWM until the encoder position stops
        changing within the given threshold, then stops and optionally
        executes a callback. The zero position corresponds to 0% stiffness
        on the VSO; the hard stop corresponds to 100% stiffness.

        Args:
            homing_pwm (float): PWM duty cycle applied during homing. Defaults to 0.25.
            sample_rate (float): Time in seconds between encoder samples. Defaults to 0.05.
            position_threshold (int): Maximum encoder count change between samples that is considered stationary.
                Defaults to 200.
            home_zero (bool): If True, home to the zero-stiffness position. If False, home
                to the full-stiffness hard stop. Defaults to True.
            timeout_s (float): Maximum homing duration in seconds. Defaults to 8.0.
            callback (Optional[Callable[[], None]]): Optional callback function to be called when homing completes.
        """
        time.sleep(1)
        keep_going = True

        if home_zero:
            LOGGER.info("Homing to zero position (0% stiffness).")
            self.set_motor_direction_backward()
            time.sleep(0.5)  # Ensure direction is set before applying PWM
        else:
            LOGGER.info("Homing to hard stop (100% stiffness).")
            self.set_motor_direction_forward()
            time.sleep(0.5)  # Ensure direction is set before applying PWM

        self.set_motor_pwm(homing_pwm)

        while keep_going:
            self.update()
            last_position = self.motor_position_cts
            time.sleep(sample_rate)

            self.update()
            error = self.motor_position_cts - last_position
            if -position_threshold <= error <= position_threshold:
                self.stop()
                keep_going = False

        if callback is not None:
            callback()

    @property
    def motor_encoder_position_perc(self) -> float:
        """
        Motor encoder position as a percentage of the full range of motion for the motor in one direction.

        Returns:
            float: Position in percentage.
        """
        self.update()
        return self.motor_position_perc

    @property
    def motor_position(self) -> float:
        """
        Motor position in radians.

        Returns:
            float : Motor position in radians.
        """
        self.update()
        return self.motor_position_cts * 2 * np.pi / 1024.0

    @property
    def motor_velocity(self) -> float:
        """
        Motor velocity (radians / second). Not supported by this driver.

        Returns:
            float: Always returns 0.0.
        """
        LOGGER.warning("Motor velocity reading is not available.")
        return 0.0

    @property
    def case_temperature(self) -> float:
        """
        Motor case temperature in degrees Celsius. Not supported by this driver.

        The VNH7070AY has thermal shutdown protection, but it does not provide a real-time temperature
        without an external sensor.

        Returns:
            float: Always returns 0.0.
        """
        LOGGER.warning("No temperature reading available for the motor casing.")
        return 0.0

    @property
    def winding_temperature(self) -> float:
        """
        Motor winding temperature in degrees Celsius. Not supported by this driver.

        The VNH7070AY has thermal shutdown protection but does not provide
        real-time temperature without an external sensor.

        Returns:
            float: Always returns 0.0.
        """
        LOGGER.warning("No temperature reading available for the motor windings.")
        return 0.0

    def perc_to_cts(self, percentage: float) -> float:
        """
        Convert a percentage of full range of motion to encoder counts.

        Args:
            percentage (float): Position as a percentage.

        Returns:
            float: Corresponding encoder count.
        """
        return percentage * self.scale_perc

    def cts_to_perc(self, counts: float) -> float:
        """
        Convert a percentage of full range of motion for the motor in one direction
        to encoder counts.

         Args:
            counts (float): Encoder count value.

        Returns:
            float: Position as a percentage.
        """
        return counts / self.scale_perc

    def mm_to_cts(self, mm: float) -> float:
        """
        Convert linear displacement in millimeters to encoder counts.

        Args:
            mm (float): Linear displacement in millimeters.

        Returns:
            float: Corresponding encoder count.
        """
        return mm * self.scale

    def cts_to_mm(self, counts: float) -> float:
        """
        Convert a number of encoder counts to a number of mm moved assuming a
        rotary to linear transmission like a lead screw.

        Args:
            counts (float): Encoder count value.

        Returns:
            float: Linear displacement in millimeters.
        """
        return counts / self.scale

    def position_control_init(self, k_p: float = 0.015, k_i: float = 2, k_d: float = 0.0001) -> None:
        """
        Initialize the PID position controller and reset all internal state.

        Args:
            k_p (float): Proportional gain. Defaults to 0.015.
            k_i (float): Integral gain. Defaults to 2.
            k_d (float): Derivative gain. Defaults to 0.0001.
        """
        self.set_position_gains(k_p, k_i, k_d)
        self.error_encoder_last = 0.0
        self.d_term_last = 0.0
        self.d_term_filtered_last = 0.0
        self.i_term = 0.0
        self.last_pwm = 0.0

    def position_control_config(
        self,
        scale_perc: float = 19972.65,
        scale: float = 21281.976,
        min_pos_error: float = 0.3,
        slider_max_perc: float = 99.5,
        slider_min_perc: float = 0.5,
        allowable_coupler_drift: float = -0.2,
        time_limit: float = 15.0,
    ) -> None:
        """
        Designed for the Variable Stiffness Orthosis

        After this time, the PWM will be set to zero, and the code assumes that the slider is jammed.

        Args:
            scale_perc: encoder counts to 1% of full range of motion for the motor in one direction
            scale: encoder conversion scale (counts to mm)
            min_pos_error: Minimum desired change in slider position that will result in a motor command
            slider_max_perc: [%] This is set slightly below 100% so that the spring support does not hit the hard stop.
            slider_min_perc: [%] This is set slightly above 0% so that the spring support does not hit the coupler.
            time_limit: [sec] maximum time for position control loop to execute (safety).
        """
        LOGGER.info("Configuring position control.")
        self.scale_perc = scale_perc
        self.scale = scale
        self.min_error = min_pos_error
        self.slider_max_perc = slider_max_perc
        self.slider_min_perc = slider_min_perc
        self.allowable_coupler_drift = allowable_coupler_drift
        self.time_limit = time_limit

        self.slider_min_counts = self.slider_min_perc * self.scale_perc
        self.slider_max_counts = self.slider_max_perc * self.scale_perc

        self.slider_min_mm = self.slider_min_counts / self.scale
        self.slider_max_mm = self.slider_max_counts / self.scale

    def lpfilter1(self, x: list[float], y_past: list[float]) -> float:
        """
        Apply a first-order low-pass IIR filter to the derivative term.

        Used to low-pass filter the derivative term in the PID control of the VSO spring support.

        Args:
             x (list[float]): Last two unfiltered derivative values [x_k, x_{k-1}].
             y_past (list[float]): Last one filtered derivative value [y_{k-1}].

         Returns:
             float: Filtered derivative value y.
        """
        a1 = [1, -0.509525449494429]
        b1 = [0.245237275252786, 0.245237275252786]
        # send it last 1 filtered points and last 2 unfiltered points
        y = -(a1[1] * y_past[0]) + b1[0] * x[0] + b1[1] * x[1]
        return y

    def pid_ctrl_position(self, error_encoder: float, dt: float) -> float:
        """
        Compute a PID PWM output given the current position error.

        Args:
            error_encoder (float): Current position error in encoder counts.
            dt (float): Time step in seconds since the last controller update.

        Returns:
            float: PWM duty cycle command.
        """
        p_term = self.k_p * error_encoder

        error_derivative = (error_encoder - self.error_encoder_last) / dt
        self.error_encoder_last = error_encoder
        d_term = error_derivative * self.k_d
        d_term_filtered = self.lpfilter1([self.d_term_last, d_term], [self.d_term_filtered_last])
        self.d_term_last = d_term
        self.d_term_filtered_last = d_term_filtered

        # only integrate when not saturated (prevent windup)
        if -(self.pwm_maximum_command - 5) < self.last_pwm < (self.pwm_maximum_command - 5):
            self.i_term = self.i_term + (self.k_i * error_encoder * dt)

        pwm_feedback = int(p_term + self.i_term + d_term_filtered) / 100
        self.last_pwm = pwm_feedback

        return pwm_feedback

    def check_coupler_drift(self) -> None:
        """Check whether the lead screw coupler has drifted out of position."""
        if self.allowable_coupler_drift < self.motor_position_mm < 0:
            LOGGER.warning("Coupler has drifted a little, but probably not an issue yet")
            LOGGER.info(f"motor_position_mm: {self.motor_position_mm:.4f}")

        if self.motor_position_mm <= self.allowable_coupler_drift:
            LOGGER.warning(
                "Coupler has been pushed back towards the motor and should be reassembled. "
                "Coupler should be flush with the lead screw, as far away from motor as "
                "possible for accurate stiffness reports."
            )
            LOGGER.info(f"motor_position_mm: {self.motor_position_mm:.4f}")

    def set_motor_direction_forward(self) -> None:
        """Set the motor direction to be forwards."""
        self.ina.on()
        self.inb.off()

    def set_motor_direction_backward(self) -> None:
        """Set the motor direction to be backwards."""
        self.ina.off()
        self.inb.on()

    def set_motor_pwm(self, pwm: float) -> None:
        """Set the motor pwm rate."""
        if pwm > self.pwm_maximum_command:
            LOGGER.info("PWM command above maximum. Setting to maximum.")
            pwm = self.pwm_maximum_command
        elif pwm < self.pwm_minimum_command and pwm >= self.pwm_lower_limit:
            LOGGER.info("PWM command below minimum. Setting to minimum.")
            pwm = self.pwm_minimum_command
        elif pwm < self.pwm_lower_limit:
            LOGGER.info("PWM command below lower limit. Setting to zero.")
            pwm = 0.0

        self.speed_control.value = pwm

    def set_motor_encoder(self, encoder_counter: EncoderCounterBase) -> None:
        """
        Set the motor encoder counter.

        Args:
            encoder_counter: Encoder counter instance providing its attribute.
        """
        self.encoder_counter = encoder_counter


if __name__ == "__main__":
    pass
