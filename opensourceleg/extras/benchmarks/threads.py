import threading
import time

from opensourceleg.extras.benchmarks.decorators import profile_time

counter = 0
PROFILING_ITERATIONS = 1000
FREQ = 1000


def core_function() -> None:
    global counter
    counter += 1
    # print("Counter: ", counter)
    time.sleep(1 / FREQ)


@profile_time(iterations=PROFILING_ITERATIONS)
def basic_counter() -> None:
    for _ in range(10):
        core_function()


@profile_time(iterations=PROFILING_ITERATIONS)
def threaded_counter() -> None:
    threads = []
    for _ in range(10):
        thread = threading.Thread(target=core_function)
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()


def main() -> None:
    basic_counter()
    threaded_counter()


if __name__ == "__main__":
    main()
