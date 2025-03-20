import time
from opensourceleg.time import Profiler

def example_tic_toc():
    """
    Example demonstrating the use of tic and toc for profiling.
    """
    profiler = Profiler("tic_toc_example")
    profiler.tic()
    time.sleep(0.1)  # Simulate some work
    duration = profiler.toc()
    print(f"Duration of operation: {duration:.3f} seconds")


def example_decorator():
    """
    Example demonstrating the use of the Profiler as a decorator.
    """
    profiler = Profiler("decorator_example")

    @profiler.decorate
    def simulated_work():
        time.sleep(0.05)  # Simulate some work

    for _ in range(5):
        simulated_work()

    print(f"Total runs: {profiler.N}, Total time: {profiler.agg:.3f} seconds")


def example_lambda():
    """
    Example demonstrating the use of the Profiler with a lambda function.
    """
    profiler = Profiler("lambda_example")
    for _ in range(5):
        profiler.profile(lambda: time.sleep(0.05))  # Simulate some work

    print(f"Total runs: {profiler.N}, Total time: {profiler.agg:.3f} seconds")


if __name__ == "__main__":
    print("Running tic-toc example:")
    example_tic_toc()

    print("\nRunning decorator example:")
    example_decorator()

    print("\nRunning lambda example:")
    example_lambda()