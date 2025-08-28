from collections import deque
from typing import Optional, Union

import numpy as np

from opensourceleg.actuators.base import MOTOR_CONSTANTS

# Sensor validation constants
MIN_SENSIBLE_CURRENT = -80000.0  # -80A in mA
MAX_SENSIBLE_CURRENT = 80000.0  # +80A in mA
MAX_SENSIBLE_TEMPERATURE = 200.0  # °C
MIN_SENSIBLE_TEMPERATURE = 0.0  # °C
SENSOR_HISTORY_SIZE = 2

__all__ = [
    "Counter",
    "DataPacketCorruptionException",
    "EdgeDetector",
    "I2tLimitException",
    "SaturatingRamp",
    "ThermalLimitException",
    "ThermalModel",
    "clamp_within_vector_range",
    "from_twos_complement",
    "to_twos_complement",
]


class ThermalLimitException(Exception):
    def __init__(self, message: str = "Software thermal limit exceeded. Exiting.") -> None:
        self.message = message
        super().__init__(self.message)


class I2tLimitException(Exception):
    def __init__(self, message: str = "Dephy Actpack I2t limit exceeded. Exiting.") -> None:
        self.message = message
        super().__init__(self.message)


class DataPacketCorruptionException(Exception):
    def __init__(self, message: str = "Data packets have been corrupted. Please check your connection.") -> None:
        self.message = message
        super().__init__(self.message)


class ThermalModel:
    """
    Enhanced thermal model with outlier filtering and soft-limiting safety controller.

    Based on `A Modular Framework for Task-Agnostic, Energy Shaping Control of Lower Limb Exoskeletons`
    by Jianping Lin, Gray C. Thomas, Nikhil V. Divekar, Vamsi Peddinti, & Robert D. Gregg

    Thermal Circuit Model:
        Two-node lumped system with winding and case temperatures.

        Equations:
            1: Cw * dTw/dt = I²R(T) + (Th-Tw)/Rwh
            2: Ch * dTh/dt = (Tw-Th)/Rwh + (Ta-Th)/Rha

    Features:
        - Integrated outlier detection for current/temperature sensors
        - Soft-limiting thermal safety controller with formal guarantees
        - Physically sensible value validation with slope-based projection
        - Backward compatible API with enhanced methods
        - Motor-specific parameter configuration via MOTOR_CONSTANTS

    Authors:
        - Gray Thomas, Senthur Ayyappan

    Args:
        motor_constants: MOTOR_CONSTANTS instance with thermal parameters
        actuator_tag: Actuator identifier for error messages. Defaults to "actuator"
        ambient_temperature: Ambient temperature in °C. Defaults to 21.0
    """

    def __init__(
        self,
        motor_constants: MOTOR_CONSTANTS,
        actuator_tag: str = "actuator",
        ambient_temperature: float = 21.0,
    ) -> None:
        self.winding_thermal_capacitance = motor_constants.WINDING_THERMAL_CAPACITANCE
        self.case_thermal_capacitance = motor_constants.CASE_THERMAL_CAPACITANCE
        self.winding_to_case_resistance = motor_constants.WINDING_TO_CASE_RESISTANCE
        self.case_to_ambient_resistance = motor_constants.CASE_TO_AMBIENT_RESISTANCE
        self.copper_temperature_coefficient = motor_constants.COPPER_TEMPERATURE_COEFFICIENT
        self.reference_temperature = motor_constants.REFERENCE_TEMPERATURE
        self.reference_resistance = motor_constants.REFERENCE_RESISTANCE

        self.winding_hard_limit = motor_constants.MAX_WINDING_TEMPERATURE
        self.winding_soft_limit = motor_constants.WINDING_SOFT_LIMIT
        self.case_hard_limit = motor_constants.MAX_CASE_TEMPERATURE
        self.case_soft_limit = motor_constants.CASE_SOFT_LIMIT

        self.winding_temperature: float = ambient_temperature
        self.case_temperature: float = ambient_temperature
        self.ambient_temperature: float = ambient_temperature

        self.current_history: deque[float] = deque(maxlen=SENSOR_HISTORY_SIZE)
        self.temperature_history: deque[float] = deque(maxlen=SENSOR_HISTORY_SIZE)

        self.actuator_tag: str = actuator_tag

        self._current_sensor_faults: int = 0
        self._case_temperature_faults: int = 0

    def __repr__(self) -> str:
        return f"ThermalModel(Tw={self.winding_temperature:.1f}°C, Tc={self.case_temperature:.1f}°C)"

    def _is_within_bounds(self, value: float, min_bound: float, max_bound: float) -> bool:
        """Check if value is within physically sensible bounds."""
        return min_bound <= value <= max_bound

    def _get_fallback_value(self, history: deque, default: float) -> float:
        """Get fallback value when sensor reading is invalid."""
        return history[-1] if len(history) > 0 else default

    def _diagnose_sensor_value(
        self,
        raw_value: float,
        history: deque,
        min_bound: float,
        max_bound: float,
        fallback_default: float,
        dt: float,
        fault_counter: int,
    ) -> tuple[float, int]:
        """Generic sensor diagnosis with bounds validation"""
        # TODO: Move diagnostics to a separate mixin just like the offline mode
        # that both actuators and sensors can inherit
        raw_value = float(raw_value)

        if not self._is_within_bounds(raw_value, min_bound, max_bound):
            final_value = self._get_fallback_value(history, fallback_default)
            fault_counter += 1

            if fault_counter >= int(1.0 / (2 * dt)):
                raise DataPacketCorruptionException(
                    f"[{self.actuator_tag.upper()}] data packets have been corrupted for a prolonged period. "
                    f"Please check your connection."
                )
        else:
            final_value = raw_value
            fault_counter = 0  # Reset fault counter on successful read

        history.append(final_value)
        return final_value, fault_counter

    def update(
        self,
        dt: float,
        motor_current: float,
        case_temperature: Optional[float] = None,
        factor_of_safety: float = 1.0,
    ) -> float:
        """
        Updates winding and case temperatures with sensor diagnosis and returns scale factor.

        Args:
            dt: Time step in seconds. Defaults to 1/200 (200Hz)
            motor_current: Motor current in mA. Defaults to 0
            case_temperature: External case temperature in °C. If None, uses thermal model.
            factor_of_safety: Factor of safety for thermal predictions. Defaults to 1.0

        Returns:
            float: Thermal scale factor [0,1] for torque/current scaling

        Thermal Dynamics:
            Cw * dTw/dt = I²R(T) + (Tc-Tw)/Rwc
            Cc * dTc/dt = (Tw-Tc)/Rwc + (Ta-Tc)/Rca
        """

        diagnosed_current, self._current_sensor_faults = self._diagnose_sensor_value(
            motor_current,
            self.current_history,
            MIN_SENSIBLE_CURRENT,
            MAX_SENSIBLE_CURRENT,
            0.0,
            dt,
            self._current_sensor_faults,
        )
        if case_temperature is not None:
            diagnosed_case_temp, self._case_temperature_faults = self._diagnose_sensor_value(
                case_temperature,
                self.temperature_history,
                MIN_SENSIBLE_TEMPERATURE,
                MAX_SENSIBLE_TEMPERATURE,
                self.ambient_temperature,
                dt,
                self._case_temperature_faults,
            )
        else:
            diagnosed_case_temp = None

        current_amps = diagnosed_current * 1e-3

        # R(T) = R₀(1 + alpha(T - T₀))
        resistance_at_temp = self.reference_resistance * (
            1 + self.copper_temperature_coefficient * (self.winding_temperature - self.reference_temperature)
        )

        # I²R
        joule_heating = factor_of_safety * current_amps**2 * resistance_at_temp

        # Winding temperature dynamics
        dTw_dt = (
            joule_heating + (self.case_temperature - self.winding_temperature) / self.winding_to_case_resistance
        ) / self.winding_thermal_capacitance
        self.winding_temperature += dt * dTw_dt

        # Case temperature dynamics
        if diagnosed_case_temp is not None:
            self.case_temperature = diagnosed_case_temp
        else:
            # Compute case temperature from thermal model
            dTc_dt = (
                (self.winding_temperature - self.case_temperature) / self.winding_to_case_resistance
                + (self.ambient_temperature - self.case_temperature) / self.case_to_ambient_resistance
            ) / self.case_thermal_capacitance
            self.case_temperature += dt * dTc_dt

        self.check_thermal_limits_and_raise()

        # Return thermal scale factor
        return self.get_thermal_scale_factor()

    def _soft_limiting_function(self, temperature: float, soft_limit: float, hard_limit: float) -> float:
        """
        Soft-limiting function
        """
        if temperature <= soft_limit:
            return 1.0
        elif temperature >= hard_limit:
            return 0.0
        else:
            return (hard_limit - temperature) / (hard_limit - soft_limit)

    def get_thermal_scale_factor(self) -> float:
        """
        Calculate thermal scaling factor using Jianping's soft-limiting approach.

        Returns:
            float: Scale factor [0,1] for torque/current scaling
        """
        winding_scale = self._soft_limiting_function(
            self.winding_temperature, self.winding_soft_limit, self.winding_hard_limit
        )
        case_scale = self._soft_limiting_function(self.case_temperature, self.case_soft_limit, self.case_hard_limit)

        # Combined scale factor (√(S_case * S_winding))
        combined_scale = winding_scale * case_scale
        return float(np.sqrt(combined_scale)) if combined_scale > 0 else 0.0

    def check_thermal_limits_and_raise(self) -> None:
        """
        Check thermal limits and raise exceptions if exceeded.
        This replaces the thermal limit checking logic from actuator update method.

        Raises:
            ThermalLimitException: If hard thermal limits are exceeded
        """
        if self.case_temperature >= self.case_hard_limit:
            raise ThermalLimitException(
                f"[{self.actuator_tag.upper()}] Case thermal limit {self.case_hard_limit}°C reached. "
                f"Current Case Temperature: {self.case_temperature:.1f}°C. Exiting."
            )

        if self.winding_temperature >= self.winding_hard_limit:
            raise ThermalLimitException(
                f"[{self.actuator_tag.upper()}] Winding thermal limit {self.winding_hard_limit}°C reached. "
                f"Current Winding Temperature: {self.winding_temperature:.1f}°C. Exiting."
            )


class EdgeDetector:
    """
    Used to calculate rising and falling edges of a digital signal in real time.
    Call edgeDetector.update(digitalSignal) to update the detector.
    Then read edgeDetector.rising_edge or falling edge to know if the event occurred.

    Author: Kevin Best
    https://github.com/tkevinbest
    """

    def __init__(self, bool_in: bool) -> None:
        self.cur_state: bool = bool_in
        self.rising_edge: bool = False
        self.falling_edge: bool = False

    def __repr__(self) -> str:
        return "EdgeDetector"

    def update(self, bool_in: bool) -> None:
        self.rising_edge = bool_in and not self.cur_state
        self.falling_edge = not bool_in and self.cur_state
        self.cur_state = bool_in


class SaturatingRamp:
    """
    Creates a signal that ramps between 0 and 1 at the specified rate.
    Looks like a trapezoid in the time domain
    Used to slowly enable joint torque for smooth switching at startup.
    Call saturatingRamp.update() to update the value of the ramp and return the value.
    Can also access saturatingRamp.value without updating.

    Example usage:
        ramp = saturatingRamp(100, 1.0)

        # In loop
            torque = torque * ramp.update(enable_ramp)

    Author: Kevin Best
    https://github.com/tkevinbest
    """

    def __init__(self, loop_frequency: float = 100, ramp_time: float = 1.0) -> None:
        """
        Args:
            loop_frequency (int, optional): Rate in Hz (default 100 Hz). Defaults to 100.
            ramp_time (float, optional): Time to complete the ramp. Defaults to 1.0.
        """
        self.delta_per_update = 1.0 / (loop_frequency * ramp_time)
        self.value = 0.0

    def __repr__(self) -> str:
        return "SaturatingRamp"

    def update(self, enable_ramp: bool = False) -> float:
        """
        Updates the ramp value and returns it as a float.
        If enable_ramp is true, ramp value increases
        Otherwise decreases.

        Example usage:
            torque = torque * ramp.update(enable_ramp)

        Args:
            enable_ramp (bool, optional): If enable_ramp is true, ramp value increases. Defaults to False.

        Returns:
            value (float): Scalar between 0 and 1.
        """
        delta = self.delta_per_update if enable_ramp else -1 * self.delta_per_update
        self.value += delta

        self.value = min(max(self.value, 0), 1)
        return self.value


def clamp_within_vector_range(
    input_value: Union[float, int], input_vector: list[Union[float, int]]
) -> Union[float, int]:
    """
    This function ensures that input_value remains within the range spanned by the input_vector.
    If the input_value falls outside the vector's bounds, it'll return the appropriate max or min value from the vector.

    Example:
        clamp_within_vector_range(10, [0,1,2,3]) = 3
        clamp_within_vector_range(-10, [0,1,2,3]) = 0

    Author:
        Kevin Best, 8/7/2023
        https://github.com/tkevinbest
    """
    min_allowed = min(input_vector)
    max_allowed = max(input_vector)
    return max(min(input_value, max_allowed), min_allowed)


def to_twos_complement(value: int, bit_length: int) -> int:
    """Converts a signed integer to 2's complement for a defined number of bits
    as an unsigned integer

    Args:
        value (int): Signed integer to convert
        bit_length (int): Number of bits of 2's complement representation

    Returns:
        int: Unsigned integer 2's complement

    Author: Axel Sjögren Holtz (axel.sjogren.holtz@vgregion.se)

    Raises:
        ValueError: If value is too small or too large for the given bit length
    """
    min_value = -(2 ** (bit_length - 1))
    max_value = 2 ** (bit_length - 1) - 1

    if value < min_value:
        raise ValueError(f"Value {value} is too small for {bit_length} bits")
    if value > max_value:
        raise ValueError(f"Value {value} is too large for {bit_length} bits")

    if value >= 0:
        return value

    return int(value + 2**bit_length)


def from_twos_complement(value: int, bit_length: int) -> int:
    """Converts a 2's complement integer to a signed integer

    Args:
        value (int): 2's complement integer
        bit_length (int): Number of bits of 2's complement representation

    Returns:
        int: Signed integer

    Author: Axel Sjögren Holtz (axel.sjogren.holtz@vgregion.se)

    Raises:
        TypeError: If value or bit_length is not an integer
        ValueError: If value is negative, bit_length is negative, or value exceeds bit_length
    """
    if not isinstance(value, int):
        raise TypeError("value must be an integer")
    if value < 0:
        raise ValueError("value must be non-negative")
    if not isinstance(bit_length, int):
        raise TypeError("bit_length must be an integer")
    if bit_length < 0:
        raise ValueError("bit_length must be non-negative")
    if value.bit_length() > bit_length:
        raise ValueError(f"value ({value}) exceeds the specified bit_length ({bit_length})")

    if value >= 2 ** (bit_length - 1):
        return int(value - (2**bit_length))
    else:
        return int(value)


class Counter:
    """
    A simple counter class that increments a counter each time the increment_counter argument is set true.
    To reset the counter, call update with increment_counter set to false.

    Author: Kevin Best, 9/25/2024
    https://github.com/tkevinbest
    """

    def __init__(self) -> None:
        self._count: int = 0

    def update(self, increment_counter: bool) -> None:
        if increment_counter:
            self._count += 1
        else:
            self._count = 0

    @property
    def current_count(self) -> int:
        """Returns the current count"""
        return self._count
