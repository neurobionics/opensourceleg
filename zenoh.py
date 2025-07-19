from observable import Logger
import time
import sys

Logger.init()
Logger.start_ros_subscriber("./zenoh.json5", "**")
while True:
    print("here")
    sys.stdout.flush()
    time.sleep(1)