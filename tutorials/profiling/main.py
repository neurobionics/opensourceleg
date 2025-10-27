"""
Example demonstrating real-time profiling with opensourceleg.profile

Usage:
    # Using CLI with auto-detection (simplest)
    python -m opensourceleg.profile main_annotated.py --frequency 200 --analyze

    # Using CLI with explicit pattern
    python -m opensourceleg.profile main_annotated.py --frequency 200 --pattern crazy_math --analyze

    # Using CLI with custom config
    python -m opensourceleg.profile main_annotated.py --config .viztracerrc --frequency 200 --analyze

    # Run directly with inline analysis
    python main_annotated.py

Note: The package ships with a default config at opensourceleg/profile/.viztracerrc
      Pattern auto-detection finds the most frequently called function automatically
"""

import numpy as np

from opensourceleg.profile import RealtimeTracer
from opensourceleg.utilities import SoftRealtimeLoop

FREQ = 200
DURATION = 2
MATRIX_DIMENSION = 200


def crazy_math(t: float) -> np.ndarray:
    a = np.random.rand(MATRIX_DIMENSION, MATRIX_DIMENSION)
    b = np.random.rand(MATRIX_DIMENSION, MATRIX_DIMENSION)
    return a * b


if __name__ == "__main__":
    tracer = RealtimeTracer(output_file="result.json")

    with tracer:
        clock = SoftRealtimeLoop(dt=1 / FREQ, report=True)

        for t in clock:
            if t > DURATION:
                break
            tracer.mark("start")
            crazy_math(t)

    tracer.analyze(frequency=FREQ, start_marker="start")
    tracer.summary()
    tracer.plot_histogram(save_path="histogram.png")
    tracer.plot_timeline(save_path="timeline.png")

