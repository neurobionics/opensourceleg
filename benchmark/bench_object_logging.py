from profiler import Logger, LogLevel
from util import time_taken
from python_based_logger import LOGGER

Logger.init(time_format = None, log_directory = None, log_name = None, print_stdout = False, file_max_bytes = 0, backup_count = 5, 
            stdout_level = None, logfile_level = LogLevel.TRACE)

class simple:
    z = 0
    def add(self, x):
        self.z+=x
    def get_z(self):
        return self.z
    def __str__(self):
        return f"Simple({self.z})"
    
def rust_log_obj(obj: simple, num_msgs):
    Logger.track_functions(dict(z_val=obj.get_z))
    for i in range(num_msgs):
        Logger.record()
    Logger.untrack_functions(["z_val"])

def python_log_obj(obj: simple, num_msgs):
    LOGGER.track_variable(obj.get_z, "str rep")
    for i in range(num_msgs):
        LOGGER.update()

obj = simple()
time_taken(rust_log_obj, obj, 1000000)
time_taken(python_log_obj, obj, 1000000)

def rust_log_primitives(num_msgs):
    for i in range(num_msgs):
        Logger.trace_variables(dict=dict(primitive=1))
        Logger.record()

def python_log_primitives(obj, num_msgs):
    LOGGER.track_variable(obj.get_z, "primitive")
    for i in range(num_msgs):
        LOGGER.update()

time_taken(rust_log_primitives, 1000000)
time_taken(python_log_primitives, obj, 1000000)