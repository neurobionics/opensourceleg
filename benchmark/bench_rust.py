from observable import Logger
from observable import LogLevel
from util import time_taken, KB, MB

def rust_log_all_msgs():
    Logger.debug("benchmark debug message")
    Logger.info("benchmark info message")
    Logger.warn("benchmark warn message")
    Logger.trace("benchmark trace message")
    Logger.error("benchmark debug message")

def rust_log_x_msgs(x: int):
    for i in range(0, x):
        Logger.debug(f"benchmark info message {i+1}")

# Size in bytes
def rust_log_sized_msgs(size):
    Logger.debug(size * 'x')

Logger.init(time_format = None, log_directory = None, log_name = None, print_stdout = False, file_max_bytes = 0, backup_count = 5, 
            stdout_level = None, logfile_level = LogLevel.TRACE)

time_taken(rust_log_x_msgs, 1000)
time_taken(rust_log_x_msgs, 10000)
time_taken(rust_log_x_msgs, 100000)
time_taken(rust_log_x_msgs, 1000000)
time_taken(rust_log_x_msgs, 10000000)

time_taken(rust_log_sized_msgs, 10 * KB)
time_taken(rust_log_sized_msgs, 10 * MB)