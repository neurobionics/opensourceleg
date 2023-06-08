# Global Units Dictionary
ALL_UNITS = {
    "force": {
        "N": 1.0,
        "lbf": 4.4482216152605,
        "kgf": 9.80665,
    },
    "torque": {
        "N-m": 1.0,
        "lbf-in": 0.1129848290276167,
        "lbf-ft": 1.3558179483314004,
        "kgf-cm": 0.0980665,
        "kgf-m": 0.980665,
    },
    "stiffness": {
        "N/rad": 1.0,
        "N/deg": 0.017453292519943295,
        "lbf/rad": 0.224809,
        "lbf/deg": 0.003490659,
        "kgf/rad": 1.8518518518518519,
        "kgf/deg": 0.031746031746031744,
    },
    "damping": {
        "N/(rad/s)": 1.0,
        "N/(deg/s)": 0.017453292519943295,
        "lbf/(rad/s)": 0.224809,
        "lbf/(deg/s)": 0.003490659,
        "kgf/(rad/s)": 1.8518518518518519,
        "kgf/(deg/s)": 0.031746031746031744,
    },
    "length": {
        "m": 1.0,
        "cm": 0.01,
        "in": 0.0254,
        "ft": 0.3048,
    },
    "position": {
        "rad": 1.0,
        "deg": 0.017453292519943295,
    },
    "mass": {
        "kg": 1.0,
        "g": 0.001,
        "lb": 0.45359237,
    },
    "velocity": {
        "rad/s": 1.0,
        "deg/s": 0.017453292519943295,
        "rpm": 0.10471975511965977,
    },
    "acceleration": {
        "rad/s^2": 1.0,
        "deg/s^2": 0.017453292519943295,
    },
    "time": {
        "s": 1.0,
        "ms": 0.001,
        "us": 0.000001,
    },
    "current": {
        "mA": 1,
        "A": 1000,
    },
    "voltage": {
        "mV": 1,
        "V": 1000,
    },
    "gravity": {
        "m/s^2": 1.0,
        "g": 9.80665,
    },
    "temperature": {
        "C": 1.0,
        "F": 0.55556,
        "K": 1.0,
    },
}

TEMPERATURE_CONVERSIONS = {
    "C": 0,
    "F": 32,
    "K": 273.15,
}


class UnitsDefinition(dict):
    """
    UnitsDefinition class is a dictionary with set and get methods that checks if the keys are valid

    Methods:
        __setitem__(key: str, value: dict) -> None
        __getitem__(key: str) -> dict
        convert(value: float, attribute: str) -> None
    """

    def __setitem__(self, key: str, value: dict) -> None:
        if key not in self:
            raise KeyError(f"Invalid key: {key}")

        if value not in ALL_UNITS[key].keys():
            raise ValueError(f"Invalid unit: {value}")

        super().__setitem__(key, value)

    def __getitem__(self, key: str) -> dict:
        if key not in self:
            raise KeyError(f"Invalid key: {key}")
        return super().__getitem__(key)

    def convert_to_default_units(self, value: float, attribute: str) -> float:
        """
        convert a value from one unit to the default unit

        Args:
            value (float): Value to be converted
            attribute (str): Attribute to be converted

        Returns:
            float: Converted value in the default unit
        """
        val: float = value * ALL_UNITS[attribute][self[attribute]]

        if attribute == "temperature":
            val = (value - TEMPERATURE_CONVERSIONS[self[attribute]]) * ALL_UNITS[attribute][self[attribute]]  # type: ignore

        return val

    def convert_from_default_units(self, value: float, attribute: str) -> float:
        """
        convert a value from the default unit to another unit

        Args:
            value (float): Value to be converted
            attribute (str): Attribute to be converted

        Returns:
            float: Converted value in the default unit
        """

        val: float = value / ALL_UNITS[attribute][self[attribute]]

        if attribute == "temperature":
            val = val + TEMPERATURE_CONVERSIONS[self[attribute]]  # type: ignore

        return val


DEFAULT_UNITS = UnitsDefinition(
    {
        "force": "N",
        "torque": "N-m",
        "stiffness": "N/rad",
        "damping": "N/(rad/s)",
        "length": "m",
        "position": "rad",
        "mass": "kg",
        "velocity": "rad/s",
        "acceleration": "rad/s^2",
        "time": "s",
        "current": "mA",
        "voltage": "mV",
        "gravity": "m/s^2",
        "temperature": "C",
    }
)

# if __name__ == "__main__":
#     units: UnitsDefinition = DEFAULT_UNITS
#     units["temperature"] = "F"  # type: ignore

#     print(units.convert_to_default_units(value=80, attribute="temperature"))
#     print(units.convert_from_default_units(value=0, attribute="temperature"))
