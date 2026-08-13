"""Module for using the DRV5056 family of Hall effect sensors."""

from typing import ClassVar

from opensourceleg.logging import LOGGER
from opensourceleg.sensors.base import HallBase


class DRV5056(HallBase):
    """
    Class for communication with the DRV5056Ax family of Hall effect sensors.

    This class allows reading Hall effect sensors in mT, mV, or V.
    """

    # Class attributes

    # Power supply voltage options
    _DRV_VCC_3_3 = 3.3
    _DRV_VCC_5 = 5

    _QUIESCENT_OFFSET = 0.6  # V

    # Unit conversions
    _V_TO_MV = 1000

    # Helpful dictionaries
    _SENSOR_TO_SENS: ClassVar[dict[str, float]] = {
        "A1": 200,  # mV / mT
        "A2": 100,
        "A3": 50,
        "A4": 25,
        "A6": 100,
        "A8": 66.6,
        "Z1": 200,
        "Z2": 100,
        "Z3": 50,
        "Z4": 25,
    }

    _SENS_3_3 = 0.6

    _SENSOR_TO_RANGE: ClassVar[dict[str, int]] = {
        "A1": 20,
        "A2": 39,
        "A3": 79,
        "A4": 158,
        "A6": 39,
        "A8": 64,
        "Z1": 20,
        "Z2": 39,
        "Z3": 79,
        "Z4": 158,
    }

    _SENS_3_3_RANGE: ClassVar[dict[str, int]] = {
        "A1": 19,
        "A2": 39,
        "A3": 78,
        "A4": 155,
        "A6": 39,
        "A8": 65,
        "Z1": 19,
        "Z2": 39,
        "Z3": 78,
        "Z4": 155,
    }

    _SENSOR_TO_COMP: ClassVar[dict[str, float]] = {
        "A1": 0.0012,
        "A2": 0.0012,
        "A3": 0.0012,
        "A4": 0.0012,
        "A6": 0.0012,
        "A8": 0.0012,
        "Z1": 0.0,
        "Z2": 0.0,
        "Z3": 0.0,
        "Z4": 0.0,
    }

    def __init__(
        self,
        tag: str = "DRV5056A1",
        offline: bool = False,
        sensor_num: str = "A1",
        t_a: int = 23,
        supply_voltage: float = 5,
    ):
        """
        Initialize the DRV5056 instance.

        Args:
            tag (str): Identifier for the Hall effect instance. Default is "DRV5056A1".
            offline (bool): If True, the ADC operates in offline mode. Default is False.
            sensor_num (str): Hall effect sensor part number. Default is A1.
            t_a (int): Ambient temperature. Default is 23 degrees Celsius.
            supply_voltage (float): Power supply voltage. Default is 5 V.

        """
        if offline:
            exit(1)

        super().__init__(tag=tag, offline=offline)

        self._sensor_num = sensor_num
        self._t_a = t_a
        self._supply_voltage = supply_voltage

    def __repr__(self) -> str:
        return "DRV5056"

    def configure(
        self,
    ) -> None:
        """
        Configure Hall effect settings based on part number and supply voltage.

        Raises:
            ValueError: If sensor_num is not a supported part number.
            ValueError: If supply voltage is outside the supported range.
        """
        # --- SENSITIVITY ---
        if self._sensor_num not in self._SENSOR_TO_SENS:
            raise ValueError(
                f"Unsupported sensor={self._sensor_num}. Choose from {sorted(self._SENSOR_TO_SENS.keys())}"
            )
        self.base_sensitivity = self._SENSOR_TO_SENS[self._sensor_num]
        self.lower_volt_sensitivity = self._SENS_3_3 * self.base_sensitivity

        # --- RANGE ---
        self.base_range = self._SENSOR_TO_RANGE[self._sensor_num]
        self.lower_volt_range = self._SENS_3_3_RANGE[self._sensor_num]

        # --- TEMPERATURE COMPENSATION ---
        self._s_tc = self._SENSOR_TO_COMP[self._sensor_num]

        # --- VCC ---
        if self._supply_voltage != self._DRV_VCC_3_3 and self._supply_voltage != self._DRV_VCC_5:
            if 4.5 <= self._supply_voltage <= 5.5:
                self.range = self.base_range
                self._sensitivity = self.base_sensitivity * self._supply_voltage / self._DRV_VCC_5
            elif 3 <= self._supply_voltage <= 3.6:
                self.range = self.lower_volt_range
                self._sensitivity = self.lower_volt_sensitivity * self._supply_voltage / self._DRV_VCC_3_3
            else:
                raise ValueError("Supply voltage out of range.")
        elif self._supply_voltage == self._DRV_VCC_3_3:
            self.range = self.lower_volt_range
            self._sensitivity = self.lower_volt_sensitivity
        elif self._supply_voltage == self._DRV_VCC_5:
            self.range = self.base_range
            self._sensitivity = self.base_sensitivity

    def start(self) -> None:
        """Start the Hall effect sensor by enabling the data stream."""
        self._streaming = True

    def stop(self) -> None:
        """Stop the Hall effect sensor by disabling the data stream."""
        self._streaming = False

    def update(self) -> None:
        """Calculate the estimated magnetic response."""
        self.field_strength = (self.voltage * self._V_TO_MV - self._QUIESCENT_OFFSET) / (
            self._sensitivity * (1 + (self._s_tc * (self._t_a - 25)))
        )
        if self.range == self.field_strength:
            LOGGER.error("Careful. The sensor may be out of range and your magnetic field may be higher.")

    @property
    def is_streaming(self) -> bool:
        """
        Check if the Hall is currently streaming data.

        Returns:
            bool: True if streaming, False otherwise.
        """
        return self._streaming

    @property
    def voltage(self) -> float:
        """
        Get the latest Hall effect data.

        Returns:
            float: Voltage reading from the sensor.
        """
        return 0.0

    @property
    def field_mT(self) -> float:
        """
        Get the latest Hall effect data in millitesla.

        Returns:
            float: Magnetic field strength in millitesla (mT).
        """
        self.update()
        return self.field_strength

    @property
    def data(self) -> float:
        """Not yet supported by this library."""
        raise NotImplementedError("Data not implemented.")
