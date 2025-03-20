from opensourceleg.logging import Logger, LogLevel


def basic_configuration():
    """Demonstrates basic logger configuration with common settings."""
    print("\n=== Basic Configuration Example ===")
    # Basic logger setup with common parameters
    logger = Logger(
        log_path="./logs",  # Log file directory
        file_name="experiment_1",  # Base name for log files
        log_format="[%(asctime)s] %(levelname)s: %(message)s",
        file_level=LogLevel.DEBUG,  # Level for file logging
        stream_level=LogLevel.INFO,  # Level for console output
        buffer_size=1000,  # Entries before auto-flush
        enable_csv_logging=True,  # Enable/disable CSV logging
    )
    print(f"Logger configured with buffer size: {logger.buffer_size}")


def log_levels():
    """Demonstrates how to set and modify log levels."""
    print("\n=== Log Levels Example ===")
    logger = Logger()
    # Available log levels (in order of severity):
    # - LogLevel.DEBUG    - Detailed debugging information
    # - LogLevel.INFO     - General information
    # - LogLevel.WARNING  - Warning messages
    # - LogLevel.ERROR    - Error messages
    # - LogLevel.CRITICAL - Critical errors
    logger.set_file_level(LogLevel.DEBUG)  # More detailed file logging
    logger.set_stream_level(LogLevel.INFO)  # Less console spam
    print(f"File level set to: {logger.file_level}")
    print(f"Stream level set to: {logger.stream_level}")


def file_management():
    """Demonstrates file management features including rotation."""
    print("\n=== File Management Example ===")
    # Configure logger with file rotation settings
    logger = Logger(
        file_max_bytes=1024 * 1024,  # 1MB max file size
        file_backup_count=5,  # Keep 5 backup files
    )
    logger.set_file_name("new_experiment")
    print(f"File name set to: {logger.file_name}")


def custom_formatting():
    """Demonstrates custom format string configuration."""
    print("\n=== Custom Formatting Example ===")
    logger = Logger()
    # Available format variables:
    # %(asctime)s     - Timestamp
    # %(levelname)s   - Log level
    # %(message)s     - Log message
    # %(filename)s    - Source filename
    # %(lineno)d     - Line number
    # %(funcName)s   - Function name
    logger.set_format("[%(asctime)s][%(levelname)s] %(filename)s:%(lineno)d - %(message)s")
    print(f"Format set to: {logger.log_format}")


def buffer_management():
    """Demonstrates buffer size configuration and manual flushing."""
    print("\n=== Buffer Management Example ===")
    # Configure logger with custom buffer settings
    logger = Logger(
        buffer_size=500,  # Flush every 500 entries
        enable_csv_logging=True,  # Enable CSV output
    )
    logger.set_buffer_size(2000)
    logger.flush_buffer()
    print(f"Buffer size adjusted to: {logger.buffer_size}")


def debug_configuration():
    """Demonstrates a typical debug configuration setup."""
    print("\n=== Debug Configuration Example ===")
    # Setup logger with debug-friendly configuration
    logger = Logger(
        file_level=LogLevel.DEBUG,
        stream_level=LogLevel.DEBUG,
        log_format="[%(asctime)s][%(levelname)s] %(filename)s:%(lineno)d - %(message)s",
    )
    print(f"Debug configuration set with format: {logger.log_format}")


def production_configuration():
    """Demonstrates a typical production configuration setup."""
    print("\n=== Production Configuration Example ===")
    # Setup logger with production-ready configuration
    logger = Logger(
        file_level=LogLevel.INFO,
        stream_level=LogLevel.WARNING,
        file_max_bytes=1024 * 1024 * 10,  # 10MB
        file_backup_count=5,
    )
    print(f"Production configuration set with max file size: {logger.file_max_bytes} bytes")


if __name__ == "__main__":
    # Run all demonstrations
    basic_configuration()
    log_levels()
    file_management()
    custom_formatting()
    buffer_management()
    debug_configuration()
    production_configuration()
