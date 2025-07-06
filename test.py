from profiler import Logger
from profiler import LogLevel
import time
import cProfile
import zenoh

class simple:
    z = 0
    def add(self, x):
        self.z+=x
    def __str__(self):
        return f"Simple({self.z})"
    

def func():
    # None values default
    Logger.init(time_format = None, log_directory = None, log_name = None, print_stdout = False, file_max_bytes = 0, backup_count = 5, 
                stdout_level = LogLevel.DEBUG, logfile_level = LogLevel.TRACE)
    t = time.time()
    make_simple = simple()
    Logger.start_ros_subscriber("key/value")
    #time.sleep(2)
    with zenoh.open(zenoh.Config()) as session:
        key = 'key/value'
        pub = session.declare_publisher(key)
        for i in range(0, 200):
            pub.put(str(i))
            print(i)

    # for i in range(1, 1000000):
    #     Logger.debug("debug msg")
    print(time.time() - t)
    # Logger.trace_variables(dict(one=1, two=2))
    # Logger.flush_record()

# cProfile.run('func()')
func()