from opensourceleg_rs import Logger, LogLevel


def basic_configuration():
    """Demonstrates basic logger configuration with common settings."""
    print("\n=== Basic Configuration Example ===")
    # Basic logger setup with common parameters
    Logger.init(
        log_directory="./logs",  # Log file directory
        log_name="experiment_1",  # Base name for log files
        print_stdout=True,  # Output to console
        stdout_level=LogLevel.INFO,  # Level for console output
        logfile_level=LogLevel.TRACE,  # Level for file logging
    )


def file_management():
    """Demonstrates file management features including rotation."""
    print("\n=== File Management Example ===")
    # Configure logger with file rotation settings
    Logger.init(
        file_max_bytes=1024 * 1024,  # 1MB max file size
        backup_count=5,  # Keep 5 backup files
    )


def debug_configuration():
    """Demonstrates a typical debug configuration setup."""
    print("\n=== Debug Configuration Example ===")
    # Setup logger with debug-friendly configuration
    Logger.init(
        logfile_level=LogLevel.DEBUG,
        stdout_level=LogLevel.DEBUG,
    )


def production_configuration():
    """Demonstrates a typical production configuration setup."""
    print("\n=== Production Configuration Example ===")
    # Setup logger with production-ready configuration
    Logger.init(
        logfile_level=LogLevel.INFO,
        stdout_level=LogLevel.WARN,
        file_max_bytes=1024 * 1024 * 10,  # 10MB
        backup_count=5,
    )


if __name__ == "__main__":
    # Only one configuration can be active during a run
    basic_configuration()
    # file_management()
    # debug_configuration()
    # production_configuration()
