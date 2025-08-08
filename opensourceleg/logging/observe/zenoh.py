from observable import Logger
from observable import LogLevel
import time
import sys

Logger.init(print_stdout=True, stdout_level=LogLevel.DEBUG)
Logger.start_ros_subscriber("./zenoh.json5", "topic")
while True:
    print("here")
    sys.stdout.flush()
    time.sleep(1)