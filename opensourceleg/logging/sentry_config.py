"""
Sentry configuration for opensourceleg library.

Provides automatic error tracking and profiling.

To profile your code:
    import sentry_sdk

    sentry_sdk.profiler.start_profiler()
    # Your code here - everything gets profiled automatically
    sentry_sdk.profiler.stop_profiler()

Environment variables:
- OPENSOURCELEG_DISABLE_SENTRY=1: Disable Sentry completely
- ENVIRONMENT=production|development|staging: Set environment tag
"""

import logging
import os
from typing import Any

import sentry_sdk
from sentry_sdk.integrations.loguru import LoguruIntegration

__all__ = ["init_sentry"]

OPENSOURCELEG_DSN = "https://84b6268de789ef082573af49d5a169eb@o4510241211809792.ingest.us.sentry.io/4510241340719104"


def _get_bool_env(var_name: str, default: bool = False) -> bool:
    """Parse environment variable as boolean.

    Accepts: '1', 'true', 'yes' (case-insensitive) as True
             '0', 'false', 'no', '' as False

    Args:
        var_name: Environment variable name
        default: Default value if variable is not set or invalid

    Returns:
        Boolean value
    """
    value = os.getenv(var_name, "").lower()
    if value in ("1", "true", "yes"):
        return True
    if value in ("0", "false", "no", ""):
        return False

    # Import logger here to avoid circular import
    from loguru import logger
    logger.warning(f"Invalid value for {var_name}: '{value}', using default: {default}")
    return default


def process_event_before_send(event: dict[str, Any], hint: dict[str, Any]) -> dict[str, Any]:
    """Process log events to add metadata and clean up message formatting.

    This hook extracts file/function/line metadata from the log record and adds it
    to the event's contexts for better display in Sentry UI.
    """
    if "logentry" in event and "log_record" in hint:
        record = hint["log_record"]

        event.setdefault("contexts", {})["source"] = {
            "filename": getattr(record, "filename", None),
            "function": getattr(record, "funcName", None),
            "lineno": getattr(record, "lineno", None),
            "module": getattr(record, "module", None),
            "pathname": getattr(record, "pathname", None),
        }

        if hasattr(record, "created"):
            from datetime import datetime, timezone
            event["contexts"]["log"] = {
                "timestamp": datetime.fromtimestamp(record.created, timezone.utc).isoformat(),
            }

    return event


def init_sentry() -> None:
    """Initialize Sentry for error tracking and profiling.

    Uses manual profiling mode - call sentry_sdk.profiler.start_profiler() to start
    profiling and stop_profiler() to stop. Everything in between gets profiled.

    Environment variables:
        OPENSOURCELEG_DISABLE_SENTRY=1: Disable Sentry completely
    """
    # Import logger here to avoid circular import
    from loguru import logger

    if _get_bool_env("OPENSOURCELEG_DISABLE_SENTRY"):
        logger.info("Sentry disabled via OPENSOURCELEG_DISABLE_SENTRY")
        return

    try:
        sentry_sdk.init(
            dsn=OPENSOURCELEG_DSN,
            send_default_pii=False,
            # Enable tracing (not required for profiling but recommended)
            enable_tracing=True,
            traces_sample_rate=1.0,
            # Manual profiling mode - you control when profiling runs
            profile_lifecycle="manual",
            profile_session_sample_rate=1.0,
            before_send=process_event_before_send,
            integrations=[
                LoguruIntegration(
                    level=logging.INFO,
                    event_level=logging.ERROR,
                    breadcrumb_format="{message}",
                    event_format="{message}",
                ),
            ],
            environment=os.getenv("ENVIRONMENT", "production"),
        )

        logger.info("Sentry initialized (error tracking + manual profiling)")
    except Exception as e:
        logger.warning(f"Failed to initialize Sentry: {e}")
