"""
Minimal Sentry profiling example - Manual profiling mode.

Just run this script and check your Sentry dashboard:
- Profiling > Profiles (for flame graphs)
- Issues (for error tracking)
"""

import time

import sentry_sdk

from opensourceleg.utilities import SoftRealtimeLoop
from opensourceleg.logging import logger
import numpy as np

FREQ = 200  # Hz
DURATION = 10 # seconds

DIMENSION = 100

def test_function(t) -> np.ndarray:
    a = np.random.rand(DIMENSION, DIMENSION)
    b = np.random.rand(DIMENSION, DIMENSION)
    result = np.matmul(a, b)
    return result


def main():
    clock = SoftRealtimeLoop(dt=1.0 / FREQ)
    sentry_sdk.profiler.start_profiler()

    for t in clock:
        if t > DURATION:
            break

        c = test_function(t)
        print(t, c.shape)

    sentry_sdk.profiler.stop_profiler()

if __name__ == "__main__":
    main()
