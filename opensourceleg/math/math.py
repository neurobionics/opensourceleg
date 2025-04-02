from typing import Any, Optional, Union

import numpy as np

__all__ = [
    "EdgeDetector",
    "SaturatingRamp",
    "ThermalModel",
    "clamp_within_vector_range",
    "from_twos_complement",
    "to_twos_complement",
]


class ThermalModel:
    """
    Thermal model of a motor developed by Jianping Lin and Gray C. Thomas
    @U-M Locomotion Lab, directed by Dr. Robert Gregg

    Assumptions:
        1: The motor is a lumped system with two thermal nodes: the winding and the case.
        2: The winding and the case are assumed to be in thermal equilibrium with the ambient.
        3: The winding and the case are assumed to be in thermal equilibrium with each other.

    Equations:
        1: C_w * dT_w/dt = (I^2)R + (T_c-T_w)/R_WC
        2: C_c * dT_c/dt = (T_w-T_c)/R_WC + (T_w-T_a)/R_CA

        where:
            C_w: Thermal capacitance of the winding
            C_c: Thermal capacitance of the case
            R_WC: Thermal resistance between the winding and the case
            R_CA: Thermal resistance between the case and the ambient
            T_w: Temperature of the winding
            T_c: Temperature of the case
            T_a: Temperature of the ambient
            I: Current
            R: Resistance

    Implementation:
        1: The model is updated at every time step with the current and the ambient temperature.
        2: The model can be used to predict the temperature of the winding and the case at any time step.
        3: The model can also be used to scale the torque based on the temperature of the winding and the case.

    Args:
        ambient (float): Ambient temperature in Celsius. Defaults to 21.
        params (dict): Dictionary of parameters. Defaults to dict().
        temp_limit_windings (float): Maximum temperature of the windings in Celsius. Defaults to 115.
        soft_border_C_windings (float): Soft border of the windings in Celsius. Defaults to 15.
        temp_limit_case (float): Maximum temperature of the case in Celsius. Defaults to 80.
        soft_border_C_case (float): Soft border of the case in Celsius. Defaults to 5.


    """

    def __init__(
        self,
        ambient: float = 21,
        params: Optional[dict[Any, Any]] = None,
        temp_limit_windings: float = 115,
        soft_border_C_windings: float = 15,
        temp_limit_case: float = 80,
        soft_border_C_case: float = 5,
    ) -> None:
        # The following parameters result from Jack Schuchmann's test with no fans
        if params is None:
            params = {}
        self.C_w: float = 0.20 * 81.46202695970649
        self.R_WC = 1.0702867186480716
        self.C_c = 512.249065845453
        self.R_CA = 1.9406620046327363
        self.α: float = 0.393 * 1 / 100  # Pure copper. Taken from thermalmodel3.py
        self.R_T_0 = 65  # temp at which resistance was measured
        self.R_ϕ_0 = 0.376  # emirical, from the computed resistance (q-axis voltage/ q-axis current). Ohms

        self.__dict__.update(params)
        self.T_w: float = ambient
        self.T_c: float = ambient
        self.T_a: float = ambient
        self.soft_max_temp_windings: float = temp_limit_windings - soft_border_C_windings
        self.abs_max_temp_windings: float = temp_limit_windings
        self.soft_border_windings: float = soft_border_C_windings

        self.soft_max_temp_case: float = temp_limit_case - soft_border_C_case
        self.abs_max_temp_case: float = temp_limit_case
        self.soft_border_case: float = soft_border_C_case

    def __repr__(self) -> str:
        return "ThermalModel"

    def update(self, dt: float = 1 / 200, motor_current: float = 0) -> None:
        """
        Updates the temperature of the winding and the case based on the current and the ambient temperature.

        Args:
            dt (float): Time step in seconds. Defaults to 1/200.
            motor_current (float): Motor current in mA. Defaults to 0.

        Dynamics:
            1: self.C_w * d self.T_w /dt = (I^2)R + (self.T_c-self.T_w)/self.R_WC
            2: self.C_c * d self.T_c /dt = (self.T_w-self.T_c)/self.R_WC + (self.T_w-self.T_a)/self.R_CA
        """

        I_q_des: float = motor_current * 1e-3

        I2R = (
            I_q_des**2 * self.R_ϕ_0 * (1 + self.α * (self.T_w - self.R_T_0))
        )  # accounts for resistance change due to temp.

        dTw_dt = (I2R + (self.T_c - self.T_w) / self.R_WC) / self.C_w
        dTc_dt: float = ((self.T_w - self.T_c) / self.R_WC + (self.T_a - self.T_c) / self.R_CA) / self.C_c
        self.T_w += dt * dTw_dt
        self.T_c += dt * dTc_dt

    def update_and_get_scale(self, dt: float, motor_current: float = 0, FOS: float = 1.0) -> float:
        """
        Updates the temperature of the winding and the case based on the current and
        the ambient temperature and returns the scale factor for the torque.

        Args:
            dt (float): Time step in seconds.
            motor_current (float): Motor current in mA. Defaults to 0.
            FOS (float): Factor of safety. Defaults to 3.0.

        Returns:
            float: Scale factor for the torque.

        Dynamics:
            1: self.C_w * d self.T_w /dt = (I^2)R + (self.T_c-self.T_w)/self.R_WC
            2: self.C_c * d self.T_c /dt = (self.T_w-self.T_c)/self.R_WC + (self.T_w-self.T_a)/self.R_CA
        """

        I_q_des: float = motor_current * 1e-3

        I2R_des = (
            FOS * I_q_des**2 * self.R_ϕ_0 * (1 + self.α * (self.T_w - self.R_T_0))
        )  # accounts for resistance change due to temp.
        scale = 1.0
        if self.T_w > self.abs_max_temp_windings:
            scale = 0.0
        elif self.T_w > self.soft_max_temp_windings:
            scale *= (self.abs_max_temp_windings - self.T_w) / (
                self.abs_max_temp_windings - self.soft_max_temp_windings
            )

        if self.T_c > self.abs_max_temp_case:
            scale = 0.0
        elif self.T_c > self.soft_max_temp_case:
            scale *= (self.abs_max_temp_case - self.T_w) / (self.abs_max_temp_case - self.soft_max_temp_case)

        I2R = I2R_des * scale

        dTw_dt = (I2R + (self.T_c - self.T_w) / self.R_WC) / self.C_w
        dTc_dt: float = ((self.T_w - self.T_c) / self.R_WC + (self.T_a - self.T_c) / self.R_CA) / self.C_c
        self.T_w += dt * dTw_dt
        self.T_c += dt * dTc_dt

        if scale <= 0.0:
            return 0.0
        if scale >= 1.0:
            return 1.0

        return float(np.sqrt(scale))  # this is how much the torque should be scaled


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
