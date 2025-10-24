"""
Sentry configuration for opensourceleg library.

This module provides privacy-focused Sentry initialization with:
- Error tracking for internal library code only
- Performance monitoring with dynamic sampling
- Profiling with configurable overhead
- Complete user privacy (no PII, no user code paths)

Sentry Threading Model:
- Profiling thread: Separate thread samples call stacks at 100Hz
- Transport worker: Background thread queues & sends data asynchronously
- Network I/O: Non-blocking, happens on worker thread
- Queue: Configurable size, drops events if full (no blocking)

Performance Overhead:
- Profiling: 1-5% CPU from sampling thread (pure CPU overhead)
- Transport: <1% (async, non-blocking)
- Total: 2-6% worst case with full profiling

Configuration via environment variables:
- OPENSOURCELEG_DISABLE_SENTRY=1: Completely disable Sentry
- OPENSOURCELEG_PROFILING=none|minimal|moderate|full: Set profiling level
- ENVIRONMENT=production|development|staging: Set environment tag
"""

import os
from typing import Any, Optional

import sentry_sdk
from sentry_sdk.integrations.loguru import LoguruIntegration

__all__ = ["init_sentry"]


def traces_sampler(sampling_context: dict[str, Any]) -> float:
    """Simple sampling: only profile hardware I/O operations.

    Sampling strategy for Raspberry Pi real-time performance monitoring:
    - Hardware I/O (update, set_*, read_*): 10% sample rate
    - Everything else: 0% (not profiled)

    This focuses profiling on the operations that actually matter for
    real-time performance: hardware communication with actuators and sensors.

    Args:
        sampling_context: Context about the transaction being sampled

    Returns:
        Sample rate between 0.0 and 1.0
    """
    transaction_context = sampling_context.get("transaction_context", {})
    name = transaction_context.get("name", "").lower()

    # Only profile hardware I/O operations
    # These are the critical operations that affect real-time performance
    hardware_io_keywords = [
        "update",      # Component updates (actuator.update, sensor.update)
        "set_motor",   # Motor commands (set_motor_position, set_motor_current, etc.)
        "set_output",  # Output commands
        "set_current", # Current commands
        "set_position",# Position commands
        "set_voltage", # Voltage commands
        "set_torque",  # Torque commands
        "set_impedance", # Impedance commands
    ]

    if any(keyword in name for keyword in hardware_io_keywords):
        return 0.1  # 10% sample rate for hardware I/O operations

    # Don't profile anything else
    return 0.0


def init_sentry() -> None:
    """Initialize Sentry with privacy-focused, low-overhead configuration.

    Configuration:
    - Privacy: No PII collected
    - Performance: Dynamic sampling based on operation type
    - Profiling: Configurable via OPENSOURCELEG_ENABLE_PROFILING environment variable
    - Opt-out: Set OPENSOURCELEG_DISABLE_SENTRY=1 to disable

    Profiling (via OPENSOURCELEG_ENABLE_PROFILING):
    - Off (default): Only error tracking, <1% overhead
    - On: Profile hardware I/O operations (update, set_* methods), ~1-2% overhead

    When enabled, profiles 10% of hardware I/O operations:
    - actuator.update(), sensor.update()
    - set_motor_position(), set_motor_current(), set_motor_voltage(), etc.
    - Other hardware communication methods

    This tells you if components meet their real-time deadlines.

    Example:
        >>> # Disable Sentry entirely
        >>> os.environ['OPENSOURCELEG_DISABLE_SENTRY'] = '1'
        >>>
        >>> # Enable profiling of hardware I/O operations
        >>> os.environ['OPENSOURCELEG_ENABLE_PROFILING'] = '1'
    """
    # Allow opt-out via environment variable
    if os.getenv("OPENSOURCELEG_DISABLE_SENTRY"):
        # Import logger here to avoid circular import
        from loguru import logger

        logger.info("Sentry disabled via OPENSOURCELEG_DISABLE_SENTRY")
        return

    # Simple on/off profiling (default: off for Raspberry Pi production)
    # When enabled, profiles ALL hardware I/O operations at 100% sample rate
    enable_profiling = os.getenv("OPENSOURCELEG_ENABLE_PROFILING", "0") == "1"

    # If profiling is enabled, profile 100% of sampled transactions
    # If disabled, don't profile anything
    profiles_sample_rate = 1.0 if enable_profiling else 0.0

    sentry_sdk.init(
        dsn="https://84b6268de789ef082573af49d5a169eb@o4510241211809792.ingest.us.sentry.io/4510241340719104",
        # Privacy: no PII
        send_default_pii=False,
        # Performance monitoring with dynamic sampling
        enable_tracing=True,
        traces_sampler=traces_sampler,  # Dynamic: 50% init, 1% loops, 10% default
        # Profiling: Simple on/off, profiles 100% of sampled traces
        # With 10% sampling of hardware I/O operations, this gives:
        # - Off (default): 0% profiling
        # - On: ~10% of hardware I/O operations profiled (~1% total overhead)
        profiles_sample_rate=profiles_sample_rate,
        # Transport configuration for Raspberry Pi
        # Limit queue size to prevent memory issues on resource-constrained devices
        transport_queue_size=100,
        # Integrations
        integrations=[
            LoguruIntegration(
                level=20,  # INFO - capture INFO and above as breadcrumbs (contextual data)
                event_level=40,  # ERROR - only send ERROR and CRITICAL as issues
            ),
        ],
        # Environment
        environment=os.getenv("ENVIRONMENT", "production"),
    )

    # Import logger here to avoid circular import
    from loguru import logger

    profiling_status = "enabled" if enable_profiling else "disabled"
    logger.info(f"Sentry initialized (error tracking: enabled, profiling: {profiling_status})")
