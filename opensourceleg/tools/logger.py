from typing import Any, Callable, List, Union, Tuple

import csv
import logging
from logging.handlers import RotatingFileHandler

import pandas as pd
import numpy as np 
import multiprocessing as mp
from multiprocessing import shared_memory
import psutil
import time
import os
import signal

"""
Module Overview:

This module defines a custom logger class, `Logger`, designed to log attributes
from class instances to a CSV file. It extends the `logging.Logger` class.

Key Class:

- `Logger`: Logs attributes of class instances to a CSV file. It supports
setting different logging levels for file and stream handlers.

Usage Guide:

1. Create an instance of the `Logger` class.
2. Optionally, set the logging levels for file and stream handlers using
   `set_file_level` and `set_stream_level` methods.
3. Add class instances and attributes to log using the `add_attributes` method.
4. Start logging data using the `update` method.
5. Optionally, close the CSV file using the `close` method.

Note:

This file is referenced by the OSL class and is instantiated manually when an OSL
is instantiated.

"""
# Function to save log file
def save_log(log_df:pd.DataFrame, log_name:str, df_idx:int, print_debug:bool):
    if print_debug: print("OSL Logging: Saving the log file to disk.")
    df_to_save = log_df.head(int(df_idx[0]))
    if print_debug: print("OSL Logging: Log file size: ", df_to_save.shape)
    df_to_save.to_csv(log_name, index=False)
    if print_debug: print("OSL Logging: Log file saved.")

# Function for the watcher process
def watcher_process(shm_name:str, np_array_shape:Tuple[int], 
                    np_array_dtype:np.dtype, df_columns:List[str], 
                    df_curr_idx_name:str, parent_pid:int, log_name:str, 
                    save_data_flag: mp.Event, exit_logging_flag: mp.Event,
                    print_debug: bool):
    
    # Create the shared memory block for the pandas dataframe
    existing_shm = shared_memory.SharedMemory(name=shm_name)
    np_array = np.ndarray(np_array_shape, dtype=np_array_dtype, 
                          buffer=existing_shm.buf)
    log_df = pd.DataFrame(np_array, columns=df_columns)

    # Create the shared memory block for the current index
    df_curr_idx_shm = shared_memory.SharedMemory(
        name=df_curr_idx_name, create=False)
    df_curr_idx = np.ndarray((1,), dtype=np.int64, buffer=df_curr_idx_shm.buf)

    # Get the parent process
    parent = psutil.Process(parent_pid)
    if print_debug: print("OSL Logger: Watcher process started.")

    # If the parent process is killed, don't exit until we save the data
    def handle_signals(signum, frame):
        if print_debug: print(f"OSL Logger: Signal {signum} received.")
        save_data_flag.set()
        exit_logging_flag.set()
    signal.signal(signal.SIGCHLD,  handle_signals)
    signal.signal(signal.SIGTERM, handle_signals)
    signal.signal(signal.SIGINT, handle_signals)

    # Watch the parent process and save the data if it is killed or we are 
    # told to save the data
    try:
        while True:
            if print_debug: print(f"Process status {parent.status()}, "
                                  f"save data flag {save_data_flag.is_set()}, "
                                   f"break loop {exit_logging_flag.is_set()}, "
                                   f"log df size {existing_shm.size}")
            # Check if the parent process is still running or we want to save
            if not parent.is_running() or save_data_flag.is_set():
                save_log(log_df, log_name, df_curr_idx, print_debug)
                save_data_flag.clear()

            if exit_logging_flag.is_set():
                if print_debug: print("OSL Logger: Exiting the watcher proc.")
                break
            time.sleep(1)

    except Exception as e:
        if print_debug: print(f"OSL Logger: Watcher process error:"
                              f" {e}, saving log file.")
        save_log(log_df, log_name, df_curr_idx, print_debug)
        # Close shared memory
        existing_shm.close()
        existing_shm.unlink()
        df_curr_idx_shm.close()
        df_curr_idx_shm.unlink()
        exit(1)


class Logger(logging.Logger):
    """
    Logger class is a class that logs attributes from a class to a csv file

    Methods:
        __init__(self, container: object, file_path: str, logger: logging.Logger = None) -> None
        log(self) -> None
    """

    def __init__(
        self,
        file_path: str = "./osl",
        log_format: str = "[%(asctime)s] %(levelname)s: %(message)s",
        pandas_logging: bool = False,
        pandas_buffer_num_rows: int = 300_000,
        print_debug: bool = False,
    ) -> None:
        
        """
        Initializes the logger object

        Inputs
        ------
        file_path: str - The path to the file where the log will be saved
        log_format: str - The format of the log message
        pandas_logging: bool - decides if we use pandas to log the data or 
            the standard csv writer. Pandas has less real time performance 
            impacts but only saves at the end. Default is False (i.e. standard
            csv writer)
        pandas_buffer_num_rows: int -  Number of rows that the pandas dataframe
            will pre-allocate. If you go over this number then pandas gets 
            really slow. The default is 300,000 rows. That is enough for 
            10 minutes at 500hz. 
        print_debug: bool - Whether to print debug messages or not. Default is
            False.
        """
        self._print_debug = print_debug

        self._file_path: str = file_path + ".log"

        self._containers: list[Union[object, dict[Any, Any]]] = []
        self._container_names: list[str] = []
        self._attributes: list[list[str]] = []

        self._pandas_logging = pandas_logging

        if pandas_logging is True:
            self._pandas_buffer_num_rows = pandas_buffer_num_rows
            self.save_data_flag = mp.Event()
            self.exit_logging_flag = mp.Event()

        else:
            self._file = open(file_path + ".csv", "w", newline="")
            self._writer = csv.writer(self._file)

            self._log_levels = {
                "DEBUG": logging.DEBUG,
                "INFO": logging.INFO,
                "WARNING": logging.WARNING,
                "ERROR": logging.ERROR,
                "CRITICAL": logging.CRITICAL,
            }

            self._file_handler = RotatingFileHandler(
                filename=self._file_path,
                mode="w",
                maxBytes=0,
                backupCount=10,
            )
            self._file_handler.setLevel(level=logging.DEBUG)
            self._file_handler.setFormatter(fmt=self._std_formatter)

            self._stream_handler = logging.StreamHandler()
            self._stream_handler.setLevel(level=logging.INFO)
            self._stream_handler.setFormatter(fmt=self._std_formatter)

            self.addHandler(hdlr=self._stream_handler)
            self.addHandler(hdlr=self._file_handler)

        super().__init__(__name__)
        self.setLevel(logging.DEBUG)

        self._std_formatter = logging.Formatter(log_format)

        self._is_logging = False

        self._header_data: list[str] = []
        self._data: list[Any] = []

    def __repr__(self) -> str:
        return f"Logger"

    def set_file_level(self, level: str = "DEBUG") -> None:
        """
        Sets the level of the logger

        Args:
            level (str): Level of the logger
        """
        if level not in self._log_levels.keys():
            self.warning(msg=f"Invalid logging level: {level}")

        self._file_handler.setLevel(level=self._log_levels[level])

    def set_stream_level(self, level: str = "INFO") -> None:
        """
        Sets the level of the logger

        Args:
            level (str): Level of the logger
        """
        if level not in self._log_levels.keys():
            self.warning(msg=f"Invalid logging level: {level}")

        self._stream_handler.setLevel(level=self._log_levels[level])

    def add_attributes(
        self,
        container: Union[object, dict[Any, Any]],
        attributes: list[str],
        container_name: str = None,
    ) -> None:
        """
        Adds class instance and attributes to log

        Args:
            container (object, dict): Container can either be an object (instance of a class)
                or a Dict containing the attributes to be logged.
            attributes (list[str]): List of attributes to log
            container_name (str): An alternative name for the container that you want to be used in the log.
                                  Otherwise, the class's __repr__() method is used.
        """
        if container_name is None:
            if type(container) is dict:
                if "__main__" in container.values():
                    self._container_names.append("")
                else:
                    self._container_names.append(f"{container}:")
            else:
                if type(container).__repr__ is not object.__repr__:
                    self._container_names.append(f"{container}:")
                else:
                    self._container_names.append(f"")
        else:
            self._container_names.append(container_name + ":")

        self._containers.append(container)
        self._attributes.append(attributes)

    def update(self) -> None:
        """
        Logs the attributes of the class instance to the csv file
        """
        if not self._containers:
            return

        if not self._is_logging:
            for container, attributes, container_name in zip(
                self._containers, self._attributes, self._container_names
            ):
                for attribute in attributes:
                    self._header_data.append(container_name + f"{attribute}")

            if self._pandas_logging is True:
                
                # Create shared memory block for pandas dataframe
                self._pandas_buffer = pd.DataFrame(columns=self._header_data,
                                                    index=range(self._pandas_buffer_num_rows))
                np_array = self._pandas_buffer.to_numpy()
                try:
                    self.shm_df = shared_memory.SharedMemory(
                        name='log_df',create=True, size=np_array.nbytes
                    )
                except FileExistsError:
                    shm_df = shared_memory.SharedMemory(
                        name='log_df',create=False
                    )
                    shm_df.close()
                    shm_df.unlink()
                    self.shm_df = shared_memory.SharedMemory(
                        name='log_df',create=True, size=np_array.nbytes
                    )
                # Create a NumPy array backed by shared memory
                self.shm_array = np.ndarray(
                    np_array.shape, dtype=np_array.dtype, buffer=self.shm_df.buf
                )
                # Copy the original data into shared memory
                np.copyto(self.shm_array, np_array)

                # Create shared memory block for pandas row index
                try:
                    self._pandas_row_index_mem = shared_memory.SharedMemory(
                        name='row_index', create=True, size=np.dtype(np.int64).itemsize
                    )
                except FileExistsError:
                    pandas_row_index_mem = shared_memory.SharedMemory(
                        name='row_index', create=False
                    )
                    pandas_row_index_mem.close()
                    pandas_row_index_mem.unlink()
                    self._pandas_row_index_mem = shared_memory.SharedMemory(
                        name='row_index', create=True, size=np.dtype(np.int64).itemsize
                    )
                self._pandas_row_index = np.ndarray(
                    (1,), dtype=np.int64, buffer=self._pandas_row_index_mem.buf
                )
                self._pandas_row_index[0] = 0

                # Create the watcher process
                self._watcher_process = mp.Process(
                    target=watcher_process,
                    args=(self.shm_df.name, np_array.shape, np_array.dtype,
                          self._header_data,self._pandas_row_index_mem.name,
                          os.getpid(), self._file_path, 
                            self.save_data_flag, self.exit_logging_flag,
                            self._print_debug),
                )
                self._watcher_process.daemon = True
                self._watcher_process.start()

                self._is_logging = True
            else:
                self._writer.writerow(self._header_data)
                self._is_logging = True

        for container, attributes in zip(self._containers, self._attributes):
            if isinstance(container, dict):
                for attribute in attributes:
                    self._data.append(container.get(attribute))
            else:
                for attribute in attributes:
                    self._data.append(getattr(container, attribute))

        if self._pandas_logging:
            self.shm_array[self._pandas_row_index[0],:] = self._data
            self._pandas_row_index += 1
        else:
            self._writer.writerow(self._data)
            self._header_data.clear()
            self._file.flush()
        self._data.clear()

    def force_save_data(self):
        """
        Forces the data to be saved to the log file
        """
        if self._pandas_logging:
            self.save_data_flag.set()
        else:
            self._file.flush()

    def __del__(self) -> None:
        """
        Closes the file when the object is deleted
        """
        if self._print_debug: print("OSL Logger: Running __del__.")
        if self._pandas_logging is True:
            self.save_data_flag.set()
            self._watcher_process.join()
            # Close shared memory
            self.shm_df.close()
            self.shm_df.unlink()
            self._pandas_row_index_mem.close()
            self._pandas_row_index_mem.unlink()

        if not self._pandas_logging:
            self._file.close()


if __name__ == "__main__":
    local_logger = Logger(file_path="./test_log", pandas_logging=True, 
                          print_debug=True, pandas_buffer_num_rows=200)
    print("Logger created")
    local_variable_1 = 100
    local_variable_2 = 101200

    class SimpleClass:
        def __init__(self):
            self.a = 1
            self.b = 2
            self.c = 3

        def __repr__(self) -> str:
            return f"SimpleClass"

    simple_class = SimpleClass()

    local_logger.add_attributes(locals(), ["local_variable_1"])
    local_logger.add_attributes(locals(), ["local_variable_2"], "custom_container")
    local_logger.add_attributes(simple_class, ["a", "b", "c"])
    time.sleep(3)
    local_logger.update()
    local_logger.update()
    local_logger.update()
    local_logger.update()


    print(f"local_logger dataframe {local_logger._pandas_buffer.size}")
    time.sleep(3)
    # Trigger a segfault
    if False:
        import ctypes
        ctypes.string_at(0)
    print("Test finished")
