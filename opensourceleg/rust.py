"""
High-performance Rust backend for opensourceleg

This module provides fast implementations of common robotics algorithms
using Rust and PyO3 bindings.
"""

from contextlib import suppress

with suppress(ImportError):
    from opensourceleg.rust import *  # noqa: F403
