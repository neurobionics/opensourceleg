"""
Real-time profiling tools for OpenSourceLeg.

This module provides low-overhead profiling using VizTracer,
optimized for real-time robotics applications.
"""

from opensourceleg.profile.tracer import IterationStats, RealtimeTracer

__all__ = ["RealtimeTracer", "IterationStats"]
