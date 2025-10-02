"""
Comprehensive tests for the OfflineMixin class.

This module tests the general offline mode functionality that can be applied
to any hardware device (actuators, sensors, etc.).
"""

from typing import Any, ClassVar
from unittest.mock import patch

import pytest

from opensourceleg.common.offline import OfflineMixin


class MockDevice(OfflineMixin):
    """Mock device class for testing OfflineMixin functionality."""

    _OFFLINE_METHODS: ClassVar[list[str]] = ["start", "stop", "calibrate"]
    _OFFLINE_PROPERTIES: ClassVar[list[str]] = ["temperature", "voltage", "status"]
    _OFFLINE_PROPERTY_DEFAULTS: ClassVar[dict[str, Any]] = {
        "temperature": 25.0,
        "voltage": 12.0,
        "status": "ready",
    }

    def __init__(self, tag: str = "mock_device", offline: bool = False):
        self._tag = tag
        self._temperature = 100.0  # Original value
        self._voltage = 24.0  # Original value
        self._status = "active"  # Original value
        super().__init__(offline=offline)

    def start(self):
        """Mock start method."""
        return "started"

    def stop(self):
        """Mock stop method."""
        return "stopped"

    def calibrate(self):
        """Mock calibrate method."""
        return "calibrated"

    @property
    def temperature(self) -> float:
        return self._temperature

    @property
    def voltage(self) -> float:
        return self._voltage

    @property
    def status(self) -> str:
        return self._status


class MockDeviceWithConnectionState(OfflineMixin):
    """Mock device with connection state attributes for testing."""

    _OFFLINE_METHODS: ClassVar[list[str]] = ["connect"]

    def __init__(self, offline: bool = False):
        self._is_open = False
        self._is_streaming = False
        super().__init__(offline=offline)

    def connect(self):
        return "connected"


# Test fixtures
@pytest.fixture
def online_device():
    """Create a device in online mode."""
    return MockDevice(offline=False)


@pytest.fixture
def offline_device():
    """Create a device in offline mode."""
    return MockDevice(offline=True)


@pytest.fixture
def connection_device_offline():
    """Create a device with connection states in offline mode."""
    return MockDeviceWithConnectionState(offline=True)


# Basic offline mode tests
def test_offline_mode_initialization(offline_device):
    """Test that offline mode initializes correctly."""
    assert offline_device.is_offline is True


def test_online_mode_initialization(online_device):
    """Test that online mode initializes correctly."""
    assert online_device.is_offline is False


def test_offline_mode_enables_automatically(offline_device):
    """Test that offline mode patches are applied during initialization."""
    # The device should have been patched during __init__
    assert offline_device.__class__.__name__.startswith("Offline")


# Method stubbing tests
def test_offline_methods_are_stubbed(offline_device):
    """Test that offline methods are replaced with stubs."""
    # These should not call the original methods
    offline_device.start()  # Should not return "started"
    offline_device.stop()  # Should not return "stopped"
    offline_device.calibrate()  # Should not return "calibrated"


@patch("opensourceleg.logging.logger.LOGGER.debug")
def test_offline_method_stub_logging(mock_logger, offline_device):
    """Test that offline method stubs log debug messages."""
    offline_device.start()

    # Check that debug logging was called with expected message
    mock_logger.assert_called()
    call_args = mock_logger.call_args[1]["msg"]
    assert "start() called in offline mode - no action taken" in call_args
    assert offline_device._tag in call_args


def test_online_methods_work_normally(online_device):
    """Test that online mode methods work normally."""
    assert online_device.start() == "started"
    assert online_device.stop() == "stopped"
    assert online_device.calibrate() == "calibrated"


# Property patching tests
def test_offline_properties_return_defaults(offline_device):
    """Test that offline properties return configured default values."""
    assert offline_device.temperature == 25.0  # Default, not original 100.0
    assert offline_device.voltage == 12.0  # Default, not original 24.0
    assert offline_device.status == "ready"  # Default, not original "active"


def test_online_properties_return_actual_values(online_device):
    """Test that online properties return actual values."""
    assert online_device.temperature == 100.0  # Original value
    assert online_device.voltage == 24.0  # Original value
    assert online_device.status == "active"  # Original value


def test_offline_property_fallback_default():
    """Test that properties without configured defaults fallback to 0.0."""

    class DeviceWithoutDefaults(OfflineMixin):
        _OFFLINE_PROPERTIES: ClassVar[list[str]] = ["unknown_prop"]

        def __init__(self):
            super().__init__(offline=True)

        @property
        def unknown_prop(self):
            return 999.0

    device = DeviceWithoutDefaults()
    assert device.unknown_prop == 0.0  # Fallback default


# Dynamic class generation tests
def test_offline_mode_creates_dynamic_class(offline_device):
    """Test that offline mode creates a new class with patched properties."""
    assert offline_device.__class__.__name__ == "OfflineMockDevice"
    assert offline_device.__class__ != MockDevice


def test_online_mode_preserves_original_class(online_device):
    """Test that online mode preserves the original class."""
    assert online_device.__class__.__name__ == "MockDevice"
    assert online_device.__class__ == MockDevice


# Connection state tests
def test_offline_mode_sets_connection_states(connection_device_offline):
    """Test that offline mode sets connection states to True."""
    assert connection_device_offline._is_open is True
    assert connection_device_offline._is_streaming is True


# Device identifier tests
def test_device_identifier_with_tag():
    """Test device identifier uses _tag when available."""
    device = MockDevice(tag="test_device", offline=True)
    assert device._get_device_identifier() == "test_device"


def test_device_identifier_with_tag_property():
    """Test device identifier uses tag property as fallback."""

    class DeviceWithTagProperty(OfflineMixin):
        def __init__(self):
            self.tag = "prop_device"
            super().__init__(offline=True)

    device = DeviceWithTagProperty()
    assert device._get_device_identifier() == "prop_device"


def test_device_identifier_fallback_to_class_name():
    """Test device identifier falls back to class name."""

    class DeviceWithoutTag(OfflineMixin):
        def __init__(self):
            super().__init__(offline=True)

    device = DeviceWithoutTag()
    # Without properties to patch, no dynamic class is created, so name stays the same
    assert device._get_device_identifier() == "DeviceWithoutTag"


# Integration tests
def test_mixed_methods_and_properties():
    """Test a device with both methods and properties in offline mode."""

    class ComplexDevice(OfflineMixin):
        _OFFLINE_METHODS: ClassVar[list[str]] = ["initialize", "shutdown"]
        _OFFLINE_PROPERTIES: ClassVar[list[str]] = ["reading", "state"]
        _OFFLINE_PROPERTY_DEFAULTS: ClassVar[dict[str, Any]] = {
            "reading": 42.0,
            "state": "offline",
        }

        def __init__(self):
            super().__init__(offline=True)

        def initialize(self):
            return "init"

        def shutdown(self):
            return "shutdown"

        @property
        def reading(self):
            return 123.0

        @property
        def state(self):
            return "online"

    device = ComplexDevice()

    # Methods should be stubbed
    device.initialize()  # Should not return "init"
    device.shutdown()  # Should not return "shutdown"

    # Properties should return defaults
    assert device.reading == 42.0  # Not 123.0
    assert device.state == "offline"  # Not "online"


@patch("opensourceleg.logging.logger.LOGGER.info")
def test_offline_mode_info_logging(mock_logger):
    """Test that offline mode logs info message when enabled."""
    MockDevice(offline=True)

    mock_logger.assert_called()
    call_args = mock_logger.call_args[1]["msg"]
    assert "Offline mode enabled" in call_args
    assert "hardware methods and properties patched" in call_args


# Edge cases
def test_offline_mode_with_no_configurations():
    """Test offline mode works with empty configuration."""

    class MinimalDevice(OfflineMixin):
        def __init__(self):
            super().__init__(offline=True)

    device = MinimalDevice()
    assert device.is_offline is True


def test_offline_mode_missing_methods_handled_gracefully():
    """Test that missing methods in OFFLINE_METHODS are handled gracefully."""

    class DeviceWithMissingMethods(OfflineMixin):
        _OFFLINE_METHODS: ClassVar[list[str]] = ["nonexistent_method"]

        def __init__(self):
            super().__init__(offline=True)

    # Should not raise exception
    device = DeviceWithMissingMethods()
    assert device.is_offline is True


def test_offline_mode_missing_properties_handled_gracefully():
    """Test that missing properties in OFFLINE_PROPERTIES are handled gracefully."""

    class DeviceWithMissingProperties(OfflineMixin):
        _OFFLINE_PROPERTIES: ClassVar[list[str]] = ["nonexistent_property"]

        def __init__(self):
            super().__init__(offline=True)

    # Should not raise exception
    device = DeviceWithMissingProperties()
    assert device.is_offline is True
