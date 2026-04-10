from rich.console import Console
from time import strftime, localtime

class Logger:

    def __init__(self, log_file: str = "log.txt"):
        self.console = Console(highlight=False)
        self.log_file = log_file
        self.file = open(self.log_file, "w")

    def log(self, message: str, style: str = "black"):
        self.file.write(f"{strftime('%Y-%m-%d %H:%M:%S', localtime())} - {message}\n")
        self.console.print(message, style=style)
    
    def close(self):
        self.file.close()