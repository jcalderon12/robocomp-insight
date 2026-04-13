from rich.console import Console
from time import strftime, localtime

class Logger:

    def __init__(self, log_file: str = "log.txt") -> None:
        """Initialize the logger with a log file.
            Parameters:
                - log_file (str): Path to the log file (default: "log.txt").
        """
        self.console = Console(highlight=False)
        self.log_file = log_file
        self.file = open(self.log_file, "w")

    def log(self, message: str, style: str = "black") -> None:
        """Log a message to file and console.
            Parameters:
                - message (str): The message to log.
                - style (str): The Rich console style for the message (default: "black").
        """
        self.file.write(f"{strftime('%Y-%m-%d %H:%M:%S', localtime())} - {message}\n")
        self.console.print(message, style=style)
    
    def close(self) -> None:
        """Close the log file.
        """
        self.file.close()