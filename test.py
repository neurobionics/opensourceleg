from profiler import Logger
from profiler import LogLevel
import time
import cProfile

class simple:
    z = 0
    def add(self, x):
        self.z+=x
    def __str__(self):
        return f"Simple({self.z})"


def func():
    # None values default
    Logger.init(time_format = None, log_directory = None, log_name = None, print_stdout = False, file_max_bytes = 1028, backup_count = 5, 
                stdout_level = LogLevel.WARN, logfile_level = LogLevel.TRACE)

    t = time.time()
    make_simple = simple()
    for i in range(1, 10000):
        Logger.debug("debug msg")
        make_simple.add(3)
        Logger.trace_variables(dict(variable=make_simple.z))
        Logger.record()
        Logger.info("info msg")
    print(time.time() - t)
    # Logger.trace_variables(dict(one=1, two=2))
    # Logger.flush_record()

cProfile.run('func()')