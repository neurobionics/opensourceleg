"""
Sentry integration examples for opensourceleg library.

Demonstrates:
1. Basic logging with loguru
2. Error tracking with Sentry
3. Hardware I/O profiling (simple on/off)
4. Configuration via environment variables
"""

import os
import time

import opensourceleg
from opensourceleg.logging import logger, trace_performance

# Configuration examples:
# os.environ['OPENSOURCELEG_DISABLE_SENTRY'] = '1'  # Disable Sentry completely
# os.environ['OPENSOURCELEG_ENABLE_PROFILING'] = '1'  # Enable hardware I/O profiling
# os.environ['ENVIRONMENT'] = 'development'  # Set environment tag


def main():
    """Main function demonstrating Sentry and logging integration."""

    # Example 1: Basic logging (automatically initialized)
    logger.info(f"opensourceleg v{opensourceleg.__version__} initialized")
    logger.debug("This is a debug message (only in file logs)")
    logger.warning("This is a warning message")

    # Example 2: Error tracking (automatically sent to Sentry)
    try:
        # Simulate an error in robotics code
        result = 1 / 0
    except ZeroDivisionError:
        logger.exception("Error in robotics code - this exception is sent to Sentry")
        # Exception automatically captured by Sentry LoguruIntegration

    # Example 3: Simulating hardware I/O operations
    # These would be profiled if OPENSOURCELEG_ENABLE_PROFILING=1
    @trace_performance(op="actuator.update", description="Simulated actuator update")
    def simulate_actuator_update():
        """Simulate reading from hardware."""
        time.sleep(0.001)  # Simulated I2C read + processing
        logger.debug("Actuator updated")

    @trace_performance(op="actuator.set_motor_current", description="Simulated motor command")
    def simulate_motor_command(current_ma):
        """Simulate sending command to hardware."""
        time.sleep(0.0005)  # Simulated SPI write
        logger.debug(f"Motor current set to {current_ma} mA")

    # These are only profiled if OPENSOURCELEG_ENABLE_PROFILING=1
    # With 10% sample rate, so ~10 out of 100 calls will be profiled
    logger.info("Simulating 100 hardware I/O operations...")
    for i in range(100):
        simulate_actuator_update()
        if i % 10 == 0:
            simulate_motor_command(1000)

    logger.info("Sentry demo complete!")

    # Show current configuration
    sentry_status = "Disabled" if os.getenv("OPENSOURCELEG_DISABLE_SENTRY") else "Enabled"
    profiling_status = "Enabled" if os.getenv("OPENSOURCELEG_ENABLE_PROFILING") == "1" else "Disabled"

    logger.info(f"Sentry status: {sentry_status}")
    logger.info(f"Profiling status: {profiling_status}")

    if sentry_status == "Enabled" and profiling_status == "Disabled":
        logger.info("Mode: Error tracking only (<1% overhead)")
    elif sentry_status == "Enabled" and profiling_status == "Enabled":
        logger.info("Mode: Error tracking + hardware I/O profiling (~1-2% overhead)")
    else:
        logger.info("Mode: Sentry completely disabled")


if __name__ == "__main__":
    main()
