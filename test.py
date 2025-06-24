from profiler import Logger
import time
import cProfile

class simple:
    z = 5
    def add(x, y):
        return x + y
    def __str__(self):
        return f"Simple({self.z})"

def func():
    # None values default
    Logger.init(time_format = None, log_directory = None, log_name = None, print_stdout = False)

    t = time.time()
    for i in range(1, 1000000):
        Logger.warn("yolo")
        
    print(time.time() - t)
cProfile.run('func()')
#func()