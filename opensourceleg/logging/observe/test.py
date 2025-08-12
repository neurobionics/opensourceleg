from observable import Logger
from observable import LogLevel
import time
import cProfile

class simple:
    z = 0
    def add(self, x):
        self.z+=x
    def get_z(self):
        return self.z
    def __str__(self):
        return f"Simple({self.z})"


def func():
    # None values default
    Logger.init(log_directory = None, log_name = None, file_max_bytes = 1028, backup_count = 5, 
                stdout_level = LogLevel.TRACE, logfile_level = LogLevel.OFF)

    t = time.time()
    make_simple = simple()
    Logger.track_functions(dict(track_z=make_simple.get_z))
    Logger.track_functions(dict(track_z=make_simple.get_z))
    for i in range(1, 50):
        Logger.debug("debug msg")
        make_simple.add(3)
        Logger.record()
        Logger.info("info msg")
    print(time.time() - t)
    Logger.untrack_functions(["track_z", "Lul"])
    # Logger.trace_variables(dict(one=1, two=2))
    # Logger.flush_record()

    Logger.update_log_file_configuration(log_directory="./bogs", log_name="bogs.log", file_max_bytes=5000, backup_count=10)

    for i in range(1, 1000):
        Logger.debug("lalalalaa it worked!")

func()