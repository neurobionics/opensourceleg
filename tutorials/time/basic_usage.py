import time

from opensourceleg.time.time import SoftRealtimeLoop


def main():
    # Create loop with 1ms timestep
    rt_loop = SoftRealtimeLoop(dt=0.001)

    # Example 1: Simple function that runs once
    def basic_function():
        print("Basic loop iteration")
        return 0  # Stop after one iteration

    print("Running basic function...")
    rt_loop.run(basic_function)

    # Example 2: Function that runs for a specific duration
    print("\nRunning timed function for 3 seconds...")
    start_time = time.monotonic()

    def timed_function():
        elapsed = time.monotonic() - start_time
        print(f"Time elapsed: {elapsed:.2f} seconds")
        if elapsed > 3:
            return 0  # Stop after 3 seconds
        return None

    rt_loop.run(timed_function)


if __name__ == "__main__":
    main()
