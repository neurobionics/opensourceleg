from profiler import Logger
import time

# None values default
Logger.init(time_format = None, log_directory = None, log_name = None, print_stdout = False)

t = time.time()
for i in range(1, 1000000):
    Logger.debug("blab hbhb 0gbhsotn")
    Logger.trace_variables(dict(one=1, two=2))
    Logger.flush_record()
print(time.time() - t)