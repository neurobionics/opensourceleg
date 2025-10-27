import numpy as np

from opensourceleg.utilities import SoftRealtimeLoop
import cProfile
import pstats

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
    profiler = cProfile.Profile()
    profiler.enable()
    main()
    profiler.disable()

    stats = pstats.Stats(profiler)
    stats.sort_stats(pstats.SortKey.CUMULATIVE)
    stats.print_stats(10)