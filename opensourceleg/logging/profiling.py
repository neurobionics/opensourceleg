"""
Performance profiling decorators for Sentry integration.

This module provides decorators for tracing performance of hardware I/O operations
in the opensourceleg library, optimized for Raspberry Pi real-time control.

Strategy:
Profile ONLY hardware I/O operations (the operations that affect real-time performance):
- Component updates: actuator.update(), sensor.update()
- Motor commands: set_motor_position(), set_motor_current(), set_motor_voltage(), etc.
- Other hardware communication methods

Profiling is:
- Simple: Either on or off (OPENSOURCELEG_ENABLE_PROFILING=1)
- Focused: Only hardware I/O operations (10% sample rate)
- Safe: Non-blocking, happens asynchronously on background thread
- Low overhead: ~1% when enabled, 0% when disabled (default)

Decorators:
- @trace_performance: For hardware I/O transactions
- @trace_span: For operations within transactions

Examples:
    >>> from opensourceleg.logging.profiling import trace_performance
    >>>
    >>> # Profile actuator updates (hardware I/O)
    >>> @trace_performance(op="actuator.update", description="Update Dephy actuator")
    >>> def update(self):
    >>>     # Read from hardware, update thermal model, etc.
    >>>     ...
    >>>
    >>> # Profile motor commands (hardware I/O)
    >>> @trace_performance(op="actuator.set_motor_position", description="Set motor position")
    >>> def set_motor_position(self, value):
    >>>     # Send command to hardware
    >>>     ...
"""

import functools
from typing import Any, Callable

import sentry_sdk

__all__ = ["trace_performance", "trace_span"]


def trace_performance(op: str, description: str = "") -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """Decorator to create a Sentry transaction for hardware I/O profiling.

    Use this decorator on hardware I/O operations to monitor real-time performance:
    - Component updates: actuator.update(), sensor.update()
    - Motor commands: set_motor_position(), set_motor_current(), etc.
    - Hardware communication: Any method that talks to physical hardware

    Profiling is controlled by OPENSOURCELEG_ENABLE_PROFILING environment variable:
    - Off (default): No profiling, only error tracking
    - On: Profile 10% of hardware I/O operations (~1% total overhead)

    Profiling is SAFE for real-time control:
    - Non-blocking (happens on background thread)
    - Asynchronous (network I/O doesn't block your code)
    - Low overhead (~1% when enabled)

    Args:
        op: Operation type (e.g., "actuator.update", "actuator.set_motor_position")
        description: Human-readable description of the operation

    Returns:
        Decorated function that creates a transaction

    Examples:
        >>> # Profile component updates
        >>> @trace_performance(op="actuator.update", description="Update Dephy actuator")
        >>> def update(self):
        >>>     # Read hardware, update state
        >>>     pass
        >>>
        >>> # Profile motor commands
        >>> @trace_performance(op="actuator.set_motor_position", description="Set motor position")
        >>> def set_motor_position(self, position):
        >>>     # Send command to hardware
        >>>     pass
    """

    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        @functools.wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            with sentry_sdk.start_transaction(
                op=op,
                name=f"{func.__module__}.{func.__name__}",
                description=description or func.__doc__,
            ):
                return func(*args, **kwargs)

        return wrapper

    return decorator


def trace_span(op: str, description: str = "") -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """Decorator to create a span within an existing transaction.

    Use for individual methods within actuators, sensors, etc. A span is only
    created if there's an active transaction (created by @trace_performance).

    This is useful for breaking down component updates into sub-operations
    to identify bottlenecks within the update cycle.

    Args:
        op: Operation type (e.g., "actuator.read_data", "sensor.process")
        description: Human-readable description of the operation

    Returns:
        Decorated function that creates a span

    Examples:
        >>> # Within an update() transaction, track sub-operations
        >>> @trace_span(op="actuator.read_data", description="Read actuator data")
        >>> def _read_data(self):
        >>>     # Read data from hardware
        >>>     pass
        >>>
        >>> @trace_span(op="actuator.process_thermal", description="Process thermal model")
        >>> def _process_thermal(self):
        >>>     # Thermal calculations
        >>>     pass
    """

    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        @functools.wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            with sentry_sdk.start_span(op=op, description=description or f"{func.__module__}.{func.__name__}"):
                return func(*args, **kwargs)

        return wrapper

    return decorator
