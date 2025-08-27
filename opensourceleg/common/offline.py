"""
Offline mode mixin for hardware devices.

This module provides a general pattern for creating offline versions of hardware devices
that can be used for testing, simulation, and development without requiring actual hardware.
"""

from functools import partial
from typing import Any, Callable, ClassVar

from opensourceleg.logging.logger import LOGGER


class OfflineMixin:
    """
    Mixin class to provide offline mode functionality for hardware devices.

    This mixin allows devices to operate in an offline mode where hardware-dependent
    methods and properties are replaced with stubs that provide sensible default behavior.
    """

    # Subclasses should define these class variables
    _OFFLINE_METHODS: ClassVar[list[str]] = []
    _OFFLINE_PROPERTIES: ClassVar[list[str]] = []
    _OFFLINE_PROPERTY_DEFAULTS: ClassVar[dict[str, Any]] = {}

    def __init__(self, *args: Any, offline: bool = False, **kwargs: Any) -> None:
        """
        Initialize the offline mixin.

        Args:
            offline (bool): Whether to enable offline mode. Defaults to False.
            *args: Positional arguments passed to parent classes.
            **kwargs: Keyword arguments passed to parent classes.
        """
        self._is_offline: bool = offline
        self._original_offline_methods: dict[str, Callable[..., Any]] = {}

        super().__init__(*args, **kwargs)

        if self._is_offline:
            self._enable_offline_mode()

    def _offline_method_stub(self, method_name: str, *args: Any, **kwargs: Any) -> None:
        """
        Stub method for hardware-required methods in offline mode.

        Args:
            method_name (str): Name of the offline method.
            *args (Any): Positional arguments passed to the method.
            **kwargs (Any): Keyword arguments passed to the method.
        """
        LOGGER.debug(msg=f"[{self._get_device_identifier()}] {method_name}() called in offline mode - no action taken")

    def _offline_property_stub(self, property_name: str) -> Any:
        """
        Stub method for hardware-required properties in offline mode.

        Args:
            property_name (str): Name of the offline property.

        Returns:
            Any: Default value for the property.
        """
        default_value = self._OFFLINE_PROPERTY_DEFAULTS.get(property_name, 0.0)
        LOGGER.debug(
            msg=f"[{self._get_device_identifier()}] {property_name} accessed in offline mode - "
            f"returning {default_value}"
        )
        return default_value

    def _get_device_identifier(self) -> str:
        """
        Get an identifier for the device for logging purposes.

        Returns:
            str: Device identifier, defaults to class name if no tag attribute exists.
        """
        if hasattr(self, "_tag"):
            return str(self._tag)
        elif hasattr(self, "tag"):
            return str(self.tag)
        else:
            return self.__class__.__name__

    def _enable_offline_mode(self) -> None:
        """
        Enable offline mode by patching hardware-dependent methods and properties.

        This method:
        1. Replaces hardware methods with stubs
        2. Dynamically creates a subclass with patched properties
        3. Updates connection states if applicable
        """
        # Patch hardware methods
        for method_name in self._OFFLINE_METHODS:
            try:
                method = getattr(self, method_name)
                if callable(method):
                    self._original_offline_methods[method_name] = method
                    setattr(self, method_name, partial(self._offline_method_stub, method_name))
                    LOGGER.debug(msg=f"[{self._get_device_identifier()}] Patched {method_name}() for offline mode")
            except AttributeError:
                LOGGER.debug(
                    msg=f"[{self._get_device_identifier()}] {method_name}() not found - skipping offline patch"
                )

        # Create dynamic subclass with patched properties for zero runtime overhead
        if self._OFFLINE_PROPERTIES:
            self._patch_properties()

        # Update connection states if the device has them
        if hasattr(self, "_is_open"):
            self._is_open = True
        if hasattr(self, "_is_streaming"):
            self._is_streaming = True

        LOGGER.info(
            msg=f"[{self._get_device_identifier()}] Offline mode enabled - hardware methods and properties patched"
        )

    def _patch_properties(self) -> None:
        """
        Create a dynamic subclass with patched properties for offline mode.

        This approach has zero runtime overhead for online mode since properties
        are replaced at the class level rather than using __getattribute__.
        """
        original_class = type(self)
        offline_class_name = f"Offline{original_class.__name__}"

        # Create property stubs for hardware properties
        offline_properties = {}
        for property_name in self._OFFLINE_PROPERTIES:
            if hasattr(original_class, property_name):
                original_prop = getattr(original_class, property_name)
                if isinstance(original_prop, property):
                    # Create a new property that returns the default value directly
                    def make_offline_property(prop_name: str) -> property:
                        default_value = self._OFFLINE_PROPERTY_DEFAULTS.get(prop_name, 0.0)

                        def getter(instance: "OfflineMixin") -> Any:
                            return default_value

                        return property(getter)

                    offline_properties[property_name] = make_offline_property(property_name)
                    LOGGER.debug(msg=f"[{self._get_device_identifier()}] Prepared offline property for {property_name}")

        # Create the new offline class with patched properties
        offline_class = type(offline_class_name, (original_class,), offline_properties)

        # Change this instance's class to the offline version
        self.__class__ = offline_class

    @property
    def is_offline(self) -> bool:
        """
        Check if the device is in offline mode.

        Returns:
            bool: True if offline; otherwise, False.
        """
        return self._is_offline
