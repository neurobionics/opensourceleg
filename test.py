from profiler import Logger
import time
import cProfile

def func():
    Logger.init()

    t = time.time()
    for i in range(1, 10000):
        Logger.debug("blab hbhb 0gbhsotn")
        Logger.trace_variables(dict(one=1, two=2))
        Logger.flush_record()
    print(time.time() - t)

cProfile.run('func()')