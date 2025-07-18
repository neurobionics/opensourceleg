from observable import Logger
import time
import sys

Logger.init()
Logger.start_ros_subscriber("**")
while True:
    print("here")
    sys.stdout.flush()
    time.sleep(1)