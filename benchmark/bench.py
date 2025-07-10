from python_based_logger import LOGGER
from util import time_taken, KB, MB

def python_log_all_msgs():
    LOGGER.debug("benchmark debug message")
    LOGGER.info("benchmark info message")
    LOGGER.warning("benchmark warn message")
    LOGGER.error("benchmark debug message")

def python_log_x_msgs(x: int):
    for i in range(0, x):
        LOGGER.debug(f"benchmark info message {i+1}")

# Size in bytes
def python_log_sized_msgs(size):
    LOGGER.debug(size * 'x')

def benchmark_throughput_and_size():
    # Compare msg throughput
    time_taken(python_log_x_msgs, 1000)
    time_taken(python_log_x_msgs, 10000)
    #time_taken(python_log_x_msgs, 100000)
    #time_taken(python_log_x_msgs, 1000000)
    #time_taken(python_log_x_msgs, 10000000)
    time_taken(python_log_sized_msgs, 10 * KB)
    time_taken(python_log_sized_msgs, 10 * MB)

benchmark_throughput_and_size()