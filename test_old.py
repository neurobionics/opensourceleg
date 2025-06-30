from logger import LOGGER
import time
import cProfile

class simple:
    z = 0
    def add(self, x):
        self.z+=x
    def __str__(self):
        return f"Simple({self.z})"

def func():
    t = time.time()
    make_simple = simple()
    for i in range(1, 1000000):
        LOGGER.debug("debug msg")
        make_simple.add(3)
        LOGGER.track_variable(lambda: make_simple.z, "answer")
        LOGGER.flush_buffer()
        LOGGER.info("info msg")
    print(time.time() - t)
cProfile.run('func()')