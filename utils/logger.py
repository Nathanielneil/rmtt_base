import logging
import os
from datetime import datetime
from colorama import Fore, Style, init

init()


class Logger:
    def __init__(self, name="RMTT", log_level=logging.INFO):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(log_level)
        
        if not self.logger.handlers:
            self._setup_handlers()
    
    def _setup_handlers(self):
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        log_filename = f"{log_dir}/rmtt_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        
        file_handler = logging.FileHandler(log_filename, encoding='utf-8')
        console_handler = logging.StreamHandler()
        
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        
        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)
    
    def info(self, message):
        self.logger.info(f"{Fore.GREEN}{message}{Style.RESET_ALL}")
    
    def warning(self, message):
        self.logger.warning(f"{Fore.YELLOW}{message}{Style.RESET_ALL}")
    
    def error(self, message):
        self.logger.error(f"{Fore.RED}{message}{Style.RESET_ALL}")
    
    def debug(self, message):
        self.logger.debug(f"{Fore.CYAN}{message}{Style.RESET_ALL}")
    
    def critical(self, message):
        self.logger.critical(f"{Fore.MAGENTA}{message}{Style.RESET_ALL}")
    
    def flight_log(self, action, details=""):
        self.logger.info(f"{Fore.BLUE}[飞行日志] {action}: {details}{Style.RESET_ALL}")