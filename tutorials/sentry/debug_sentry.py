"""Debug script to test Sentry log integration."""

import time

import sentry_sdk
from loguru import logger
from sentry_sdk.integrations.loguru import LoguruIntegration

# Initialize Sentry with LoguruIntegration
sentry_sdk.init(
    dsn="https://84b6268de789ef082573af49d5a169eb@o4510241211809792.ingest.us.sentry.io/4510241340719104",
    traces_sample_rate=1.0,
    send_default_pii=False,
    environment="development",
    integrations=[
        LoguruIntegration(
            level=20,  # INFO - capture info and above in breadcrumbs
            event_level=20,  # INFO - send info and above as events to Sentry
        ),
    ],
)

print("=" * 60)
print("Sentry Debug Test")
print("=" * 60)

# Test 1: Direct Sentry message
print("\n[Test 1] Sending direct message via sentry_sdk.capture_message()...")
event_id = sentry_sdk.capture_message("Direct test message", level="info")
print(f"Event ID: {event_id}")

# Test 2: Logger messages (should be captured by LoguruIntegration)
print("\n[Test 2] Sending messages via loguru logger...")
logger.info("INFO: This is an info message via loguru")
logger.warning("WARNING: This is a warning message via loguru")
logger.error("ERROR: This is an error message via loguru")

# Test 3: Exception (should be captured automatically)
print("\n[Test 3] Raising an exception...")
try:
    result = 1 / 0
except ZeroDivisionError:
    logger.exception("EXCEPTION: Division by zero caught and logged")

# Flush events to Sentry
print("\n[Flushing] Sending all events to Sentry...")
success = sentry_sdk.flush(timeout=10)
print(f"Flush successful: {success}")

print("\n" + "=" * 60)
print("Test complete! Check your Sentry dashboard.")
print("You should see:")
print("1. Direct test message")
print("2. Info, warning, and error messages from loguru")
print("3. Division by zero exception")
print("=" * 60)
