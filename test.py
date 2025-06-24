from profiler import Logger
import time
import cProfile

class simple:
    z = 5
    def add(self, x):
        self.z+=x
    def __str__(self):
        return f"Simple({self.z})"
    

def func():
    # None values default
    Logger.init(time_format = None, log_directory = None, log_name = None, print_stdout = False)

    t = time.time()
    make_simple = simple()
    for i in range(1, 1000000):
        Logger.trace_variables(dict(one=1, two=2))
        Logger.flush_record()
        
    print(time.time() - t)
cProfile.run('func()')
#func()