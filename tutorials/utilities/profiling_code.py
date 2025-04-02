import time

from opensourceleg.utilities import Profiler


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


def example_context():
    """
    Example demonstrating the use of the Profiler with a context manager.
    """
    profiler = Profiler("context_example")
    with profiler:
        time.sleep(0.1)  # Simulate some work
    with profiler:
        time.sleep(0.2)  # Simulate some more work

    print(f"Total runs: {profiler.N}, Total time: {profiler.agg:.3f} seconds")
    print(f"Average time per run: {profiler.agg / profiler.N:.3f} seconds")
    print(f"Variance: {profiler.aggvar:.3f}")


if __name__ == "__main__":
    print("Running tic-toc example:")
    example_tic_toc()

    print("\nRunning decorator example:")
    example_decorator()

    print("\nRunning lambda example:")
    example_lambda()

    print("\nRunning context manager example:")
    example_context()
