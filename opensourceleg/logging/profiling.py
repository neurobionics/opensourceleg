"""
Performance tracing decorators for Sentry integration.

Use @trace_performance to instrument operations - Sentry will automatically
profile them (capturing call stacks) when profiling is enabled.
"""

import functools
from typing import Any, Callable

import sentry_sdk

__all__ = ["trace_performance", "trace_span"]


def trace_performance(
    op: str | None = None,
    description: str = "",
) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """Decorator to create a Sentry transaction with automatic profiling.

    When profiling is enabled, Sentry automatically captures call stacks
    during the transaction.

    Args:
        op: Operation type (e.g., "actuator.update"). Auto-generated if not provided.
        description: Human-readable description (uses docstring if not provided)

    Example:
        @trace_performance()
        def update(self):
            # Automatically profiled when called
            pass

        # Or with custom op:
        @trace_performance(op="actuator.update")
        def update(self):
            pass
    """

    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        # Auto-generate op from module and function name if not provided
        operation = op or f"{func.__module__.split('.')[-1]}.{func.__name__}"

        @functools.wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            with sentry_sdk.start_transaction(
                op=operation,
                name=f"{func.__module__}.{func.__name__}",
                description=description or func.__doc__,
            ):
                return func(*args, **kwargs)

        return wrapper

    return decorator


def trace_span(
    op: str | None = None,
    description: str = "",
) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """Decorator to create a span within an existing transaction.

    Use for sub-operations to add timing detail to transactions.

    Args:
        op: Operation type. Auto-generated if not provided.
        description: Human-readable description

    Example:
        @trace_span()
        def _read_sensor(self):
            pass
    """

    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        # Auto-generate op from function name if not provided
        operation = op or f"{func.__module__.split('.')[-1]}.{func.__name__}"

        @functools.wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            with sentry_sdk.start_span(
                op=operation,
                description=description or f"{func.__module__}.{func.__name__}",
            ):
                return func(*args, **kwargs)

        return wrapper

    return decorator
