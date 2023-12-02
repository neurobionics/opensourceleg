Logger Tutorial
====================

This tutorial demonstrates the usage of the `Logger` class from the `opensourceleg.tools.logger` module to log attributes from class instances to a CSV file.

Import the Logger Class
-----------------------

To use the `Logger` class, import it from the `opensourceleg.tools.logger` module:

.. code-block:: python

    from opensourceleg.tools.logger import Logger

Initialization
--------------

To begin logging, create an instance of the `Logger` class:

.. code-block:: python

    local_logger = Logger(file_path="./test_log")

- `file_path` (optional): Specify the path and filename for the log file. The default is "./test_log".

Setting Logging Levels
----------------------

Set different logging levels for the file and stream handlers. Available levels are "DEBUG," "INFO," "WARNING," "ERROR," and "CRITICAL."

.. code-block:: python

    local_logger.set_file_level(level="DEBUG")
    local_logger.set_stream_level(level="INFO")

- `set_file_level`: Set the logging level for the file handler.
- `set_stream_level`: Set the logging level for the stream handler.

Creating a Sample Class
-----------------------

Define a sample class with attributes to be logged:

.. code-block:: python

    class SimpleClass:
        def __init__(self):
            self.a = 1
            self.b = 2
            self.c = 3

    simple_class = SimpleClass()

Adding Attributes for Logging
------------------------------

Use the `add_attributes` method to specify the class instance and attributes to log:

.. code-block:: python

    local_logger.add_attributes(simple_class, ["a", "b", "c"])

- `container`: Pass the object (instance of a class) or a dictionary containing the attributes to be logged.
- `attributes`: Provide a list of attributes to log.

Logging Data
------------

Once attributes are added, log data using the `data` method:

.. code-block:: python

    local_logger.data()

- `data`: Logs the attributes of the class instance to the CSV file.

Logging a Debug Message
-----------------------

Log a debug message using the `debug` method:

.. code-block:: python

    local_logger.debug("message")

- `debug`: Logs a debug message.

Closing the Logger
------------------

After logging is complete, close the CSV file using the `close` method:

.. code-block:: python

    local_logger.close()

- `close`: Closes the CSV file.
