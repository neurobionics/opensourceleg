class ActuatorStreamException(Exception):
    """Exception raised when an actuator is not streaming.

    This exception indicates that a command was attempted on an actuator that is not
    currently streaming data.

    Attributes:
        message (str): Explanation of the error.
    """

    def __init__(self, tag: str) -> None:
        """Initialize the ActuatorStreamException.

        Args:
            tag (str): The identifier or tag of the actuator.
        """
        super().__init__(f"{tag} is not streaming, please call start() method before sending commands")


class ActuatorConnectionException(Exception):
    """Exception raised when an actuator is not connected.

    This exception is used to indicate that a connection to the actuator could not be
    established.

    Attributes:
        message (str): Explanation of the error.
    """

    def __init__(self, tag: str) -> None:
        """Initialize the ActuatorConnectionException.

        Args:
            tag (str): The identifier or tag of the actuator.
        """
        super().__init__(f"{tag} is not connected")


class ActuatorIsNoneException(Exception):
    """Exception raised when an actuator instance is None in a given mode.

    This exception is raised when an actuator instance is expected but found to be None,
    indicating that the actuator instance should be passed during initialization or set via
    a designated method.

    Attributes:
        message (str): Explanation of the error.
    """

    def __init__(self, mode: str) -> None:
        """Initialize the ActuatorIsNoneException.

        Args:
            mode (str): The mode during which the actuator instance was found to be None.
        """
        super().__init__(
            f"Actuator is None in {mode} mode, please pass the actuator instance to the mode during "
            "initialization or set the actuator instance using set_actuator method."
        )


class ControlModeException(Exception):
    """Exception raised when an attribute cannot be set due to an incorrect control mode.

    This exception indicates that a certain attribute cannot be configured because the actuator
    is not in the appropriate control mode.

    Attributes:
        message (str): Explanation of the error.
    """

    def __init__(self, tag: str, attribute: str, mode: str) -> None:
        """Initialize the ControlModeException.

        Args:
            tag (str): The identifier or tag of the actuator.
            attribute (str): The attribute that could not be set.
            mode (str): The control mode in which the attribute cannot be set.
        """
        super().__init__(
            f"[{tag}] Cannot use {attribute}() in {mode} mode.\n"
            f"Please verify that the actuator is in the correct control mode. Exiting..."
        )


class VoltageModeMissingException(Exception):
    """Exception raised when an actuator is missing a voltage mode.

    This exception is used to indicate that an actuator does not have a voltage mode
    configured when one is required.

    Attributes:
        message (str): Explanation of the error.
    """

    def __init__(self, tag: str) -> None:
        """Initialize the VoltageModeMissingException.

        Args:
            tag (str): The identifier or tag of the actuator.
        """
        super().__init__(f"{tag} must have a voltage mode")


class ActuatorKeyException(Exception):
    """Exception raised when a required key is missing from the actuator's dictionary.

    This exception indicates that a specific key was not found in the actuator dictionary,
    which is necessary for proper operation.

    Attributes:
        message (str): Explanation of the error.
    """

    def __init__(self, tag: str, key: str) -> None:
        """Initialize the ActuatorKeyException.

        Args:
            tag (str): The identifier or tag of the actuator.
            key (str): The missing key in the actuator dictionary.
        """
        super().__init__(
            f"{tag} does not have {key} key in the actuators dictionary. "
            f"Please check the actuators dictionary for the `{key}` key."
        )
