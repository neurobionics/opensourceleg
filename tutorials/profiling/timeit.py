import numpy as np
import timeit

from opensourceleg.profiling import RealtimeTracer
from opensourceleg.utilities import SoftRealtimeLoop

FREQ = 200
DURATION = 2
MATRIX_DIMENSION = 200


def some_math(t: float) -> np.ndarray:
    print(f"Performing some math at time {t:.2f}s")
    a = np.random.rand(MATRIX_DIMENSION, MATRIX_DIMENSION)
    b = np.random.rand(MATRIX_DIMENSION, MATRIX_DIMENSION)
    return a * b

def main():
    clock = SoftRealtimeLoop(dt=1 / FREQ, report=True)
    for t in clock:
        if t > DURATION:
            break
        some_math(t)


if __name__ == "__main__":
    total_time = timeit.timeit("main()", globals=globals(), number=1)
    print(f"Total execution time: {total_time:.2f} seconds")