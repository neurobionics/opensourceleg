from enum import Enum
from types import TracebackType
from typing import Any, Callable, Dict, List, Optional, ParamSpec, Self, Tuple, TypeVar

class LogLevel(Enum):
    """Enum for available log levels."""
    TRACE: LogLevel
    DEBUG: LogLevel
    INFO: LogLevel
    WARN: LogLevel
    ERROR: LogLevel


class Logger:
    """
    A class for configuring and using a logger.
    This class is a Python interface to the Rust logging implementation.
    """

    @staticmethod
    def init(
        time_format: Optional[str] = None,
        log_directory: Optional[str] = None,
        log_name: Optional[str] = None,
        print_stdout: bool = False,
        file_max_bytes: int = 0,
        backup_count: int = 0,
    ) -> None:
        """
        Initializes the global logger. This can be called multiple times - but behaviour will not change

        Args:
            time_format (Optional[str]): The format for timestamps in the log, using chrono format specifiers. Defaults to "%Y-%m-%d %H:%M:%S%.3f".
            log_directory (Optional[str]): The directory to save log files. Defaults to "./logs".
            log_name (Optional[str]): The base name for the log file. Defaults to "logfile.log".
            print_stdout (bool): Whether to print log messages to standard output. Defaults to False.
            file_max_bytes (int): The maximum size in bytes for a log file before it is rotated. A value of 0 means no rotation based on size. Defaults to 0.
            backup_count (int): The number of backup log files to keep. A value of 0 means infinite backups. Defaults to 0.
        """

    @staticmethod
    def debug(msg: str) -> None:
        """
        Logs a message with the DEBUG level.

        Args:
            msg (str): The message to log.
        """

    @staticmethod
    def info(msg: str) -> None:
        """
        Logs a message with the INFO level.

        Args:
            msg (str): The message to log.
        """

    @staticmethod
    def trace(msg: str) -> None:
        """
        Logs a message with the TRACE level.

        Args:
            msg (str): The message to log.
        """

    @staticmethod
    def error(msg: str) -> None:
        """
        Logs a message with the ERROR level.

        Args:
            msg (str): The message to log.
        """
    @staticmethod
    def warn(msg: str) -> None:
        """
        Logs a message with the WARN level.

        Args:
            msg (str): The message to log.
        """

    @staticmethod
    def trace_variables(variables: Dict[str, Any]) -> None:
        """
        Records the state of multiple variables from a dictionary.
        These variables are held in memory until `flush_record` is called.

        Args:
            variables (Dict[str, Any]): A dictionary of variable names to their values.
        """

    @staticmethod
    def track_functions(functions: Dict[str, Any]) -> None:
        """
        Keeps track of the given functions, will write the result to the
        variable logfile every time record() is called. The key for the
        dictionary should be a string identifier for the function. This
        will be used to untrack a function & label the variable.

        Note this only works for functions with 0 arguments.

        Args:
            functions (Dict[str, Any]): A dictionary of function ids to 0 argument functions
        """

    @staticmethod
    def untrack_functions(untrack_list: List[Any]) -> None:
        """
        Untracks the function associated with the string identifier.
        """

    @staticmethod
    def record() -> None:
        """
        Writes all currently tracked variables to the buffer.
        """

    @staticmethod
    def start_ros_subscriber(key_expr: str) -> None:
        """
        Starts a background thread which subscribes to the channel
        defined by key_expr parameter. 
        """

P = ParamSpec('P')
T = TypeVar('T')
class PyProfiler:
    """
    A high-performance profiler for timing Python code execution.
    
    This profiler provides multiple interfaces for timing code:
    - Manual timing with tic/toc methods
    - Function profiling with the profile method
    - Decorator usage when wrapping functions
    - Context manager usage with 'with' statements
    
    Args:
        wraps: Optional function to wrap for decorator usage
        verbose: Whether to print timing information to stdout (default: True)
    """
    
    def __init__(
        self, 
        wraps: Optional[Callable[..., Any]] = None, 
        verbose: bool = True
    ) -> None:
        """
        Initialize the profiler.
        
        Args:
            wraps: Optional function to wrap. When provided, this profiler
                  can be used as a decorator or called directly.
            verbose: If True, timing information will be printed to stdout.
                    If False, timing runs silently.
        """
    
    def tic(self) -> None:
        """
        Start timing. Records the current timestamp as the start time.
        
        This method should be called before the code you want to time.
        Use toc() to get the elapsed time since this call.
        """
    
    def toc(self) -> float:
        """
        Stop timing and return elapsed seconds since the last tic().
        
        Returns:
            The elapsed time in seconds as a float since the last tic() call.
            
        Note:
            If verbose=True, this will also print the elapsed time to stdout.
        """
    
    def profile(self, func: Callable[[], T]) -> Tuple[T, float]:
        """
        Profile a zero-argument function call.
        
        This method calls the provided function, measures its execution time,
        and returns both the function's result and the elapsed time.
        
        Args:
            func: A callable that takes no arguments. The function to profile.
                 
        Returns:
            A tuple containing:
            - The return value of the function call
            - The elapsed time in seconds as a float
            
        Raises:
            Any exception that the profiled function raises will be propagated.
            
        Example:
            >>> profiler = PyProfiler()
            >>> result, time_taken = profiler.profile(lambda: expensive_computation())
            >>> print(f"Result: {result}, Time: {time_taken:.3f}s")
        """