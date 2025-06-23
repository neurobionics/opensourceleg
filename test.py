from profiler import Logger
import time
import cProfile

def func():
    # None values default
    Logger.init(time_format = None, log_directory = None, log_name = None, print_stdout = False)

    t = time.time()
    for i in range(1, 1000000):
        Logger.debug("yolo")

    print(time.time() - t)

#cProfile.run('func()')
func()