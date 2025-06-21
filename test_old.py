from logger import LOGGER
import time

class myClass:
    def __init__(self):
        self.value = 42

obj = myClass()
t = time.time()
for i in range(1, 1000000):
    LOGGER.debug("blab hbhb 0gbhsotn")
    LOGGER.track_variable(lambda: obj.value, "answer")
    LOGGER.flush_buffer()
print(time.time() - t)