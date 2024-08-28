class ActuatorStreamException(Exception):
    """Actuator Stream Exception

    Attributes
    ----------
    message (str): Error message

    """

    def __init__(self, tag: str) -> None:
        super().__init__(
            f"{tag} is not streaming, please call start() method before sending commands"
        )


class ActuatorConnectionException(Exception):
    """Actuator Connection Exception

    Attributes
    ----------
    message (str): Error message

    """

    def __init__(self, tag: str) -> None:
        super().__init__(f"{tag} is not connected")


class ActuatorIsNoneException(Exception):
    """Actuator Connection Exception

    Attributes
    ----------
    message (str): Error message

    """

    def __init__(self, mode: str) -> None:
        super().__init__(
            f"Actuator is None in {mode} mode, please pass the actuator instance to the mode during initialization or set the actuator instance using set_actuator method."
        )


class ControlModeException(Exception):
    """Control Mode Exception

    Attributes
    ----------    MOTOR_COUNT_PER_REV: float = 16384
    NM_PER_AMP: float = 0.1133
    IMPEDANCE_A: float = 0.00028444
    IMPEDANCE_C: float = 0.0007812
    MAX_CASE_TEMPERATURE: float = 80
    M_PER_SEC_SQUARED_ACCLSB: float = 9.80665 / 8192
    message (str): Error message

    """

    def __init__(
        self,
        tag: str,
        attribute: str,
        mode: str,
    ) -> None:
        super().__init__(
            f"[{tag}] Cannot set {attribute} in {mode} mode. Please set the actuator to {attribute} mode first."
        )


class VoltageModeMissingException(Exception):
    """Voltage Mode Missing Exception

    Attributes
    ----------
    message (str): Error message

    """

    def __init__(self, tag: str) -> None:
        super().__init__(f"{tag} must have a voltage mode")


class ActuatorKeyException(Exception):
    """Actuator Key Error

    Attributes
    ----------
    message (str): Error message

    """

    def __init__(self, tag: str, key: str) -> None:
        super().__init__(
            f"{tag} does not have {key} key in the actuators dictionary. Please check the actuators dictionary for the `{key}` key."
        )
