import csv
import logging
from logging.handlers import RotatingFileHandler


class Logger(logging.Logger):
    """
    Logger class is a class that logs attributes from a class to a csv file

    Methods:
        __init__(self, class_instance: object, file_path: str, logger: logging.Logger = None) -> None
        log(self) -> None
    """

    def __init__(
        self,
        file_path: str = "./osl",
        log_format: str = "[%(asctime)s] %(levelname)s: %(message)s",
    ) -> None:

        self._file_path: str = file_path + ".log"

        self._class_instances = []
        self._attributes = []

        self._file = open(file_path + ".csv", "w")
        self._writer = csv.writer(self._file)

        self._log_levels = {
            "DEBUG": logging.DEBUG,
            "INFO": logging.INFO,
            "WARNING": logging.WARNING,
            "ERROR": logging.ERROR,
            "CRITICAL": logging.CRITICAL,
        }

        super().__init__(__name__)
        self.setLevel(logging.DEBUG)

        self._std_formatter = logging.Formatter(log_format)

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

        self._is_logging = False

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

    def add_attributes(self, class_instance: object, attributes_str: list[str]) -> None:
        """
        Configures the logger to log the attributes of a class

        Args:
            class_instance (object): Class instance to log the attributes of
            attributes_str (list[str]): List of attributes to log
        """
        self._class_instances.append(class_instance)
        self._attributes.append(attributes_str)

    def data(self) -> None:
        """
        Logs the attributes of the class instance to the csv file
        """
        header_data = []
        data = []

        if not self._is_logging:
            for class_instance, attributes in zip(
                self._class_instances, self._attributes
            ):
                for attribute in attributes:
                    header_data.append(f"{attribute}")

            self._writer.writerow(header_data)
            self._is_logging = True

        for class_instance, attributes in zip(self._class_instances, self._attributes):
            for attribute in attributes:
                data.append(getattr(class_instance, attribute))

        self._writer.writerow(data)
        self._file.flush()

    def close(self) -> None:
        """
        Closes the csv file
        """
        self._file.close()
