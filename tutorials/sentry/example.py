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
DURATION = 2 # seconds

DIMENSION = 100

def some_math(t) -> np.ndarray:
    print(f"Performing some math at time {t}")
    a = np.random.rand(DIMENSION, DIMENSION)
    b = np.random.rand(DIMENSION, DIMENSION)
    result = np.matmul(a, b)
    return result


def main():
    clock = SoftRealtimeLoop(dt=1.0 / FREQ)

    for t in clock:
        if t > DURATION:
            logger.error("Take this to sentry!")
            break

        c = some_math(t)

if __name__ == "__main__":
    main()
