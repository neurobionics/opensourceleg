from profiler import Logger
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
    Logger.init(time_format = None, log_directory = None, log_name = None, print_stdout = False, file_max_bytes = 1028, backup_count = 5)

    t = time.time()
    make_simple = simple()
    for i in range(1, 1000):
        Logger.debug("debug msg")
        Logger.warn("this is a warning")
        make_simple.add(3)
        Logger.trace_variables(dict(one=1, two=make_simple))
    
    Logger.flush_record()
    print(time.time() - t)
cProfile.run('func()')
#func()