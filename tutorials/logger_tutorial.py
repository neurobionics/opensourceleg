from opensourceleg.logging.logger import Logger

local_logger = Logger(file_path="./test_log")

local_logger.set_file_level(level="DEBUG")
local_logger.set_stream_level(level="INFO")


class SimpleClass:
    def __init__(self):
        self.a = 1
        self.b = 2
        self.c = 3


simple_class = SimpleClass()

local_logger.add_attributes(container=simple_class, attributes=["a", "b", "c"])
local_logger.update()
local_logger.debug("message")
