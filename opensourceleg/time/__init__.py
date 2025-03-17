from opensourceleg.time.time import SoftRealtimeLoop
from opensourceleg.time.profile import Profiler

"""
Time module for opensourceleg library.

Module Overview:

This module provides classes for managing soft real-time in the opensourceleg library.

Key Classes:

-   `SoftRealtimeLoop` class is used to create a soft real-time loop that runs at a specified frequency.
	It also handles signal interruptions gracefully.
"""

__all__ = ["SoftRealtimeLoop", "Profiler"]
