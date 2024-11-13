import threading
import time

from opensourceleg.benchmarks.decorators import profile_time

counter = 0
PROFILING_ITERATIONS = 1000
FREQ = 1000


def core_function():
    global counter
    counter += 1
    # print("Counter: ", counter)
    time.sleep(1 / FREQ)


@profile_time(iterations=PROFILING_ITERATIONS)
def basic_counter():
    for _ in range(10):
        core_function()


@profile_time(iterations=PROFILING_ITERATIONS)
def threaded_counter():
    threads = []
    for _ in range(10):
        thread = threading.Thread(target=core_function)
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()


def main():
    basic_counter()
    threaded_counter()


if __name__ == "__main__":
    main()
