import logging
from logging.handlers import RotatingFileHandler
import os

def setup_logging(app):
    # Create logs directory if it doesn't exist
    if not os.path.exists('logs'):
        os.makedirs('logs')

    # Create formatter
    formatter = logging.Formatter(
        '[%(asctime)s] %(levelname)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

    # Setup file handler with rotation
    file_handler = RotatingFileHandler(
        'logs/web_service.log',
        maxBytes=10485760,  # 10MB
        backupCount=5
    )
    file_handler.setFormatter(formatter)
    file_handler.setLevel(logging.INFO)

    # Setup console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    console_handler.setLevel(logging.INFO)

    # Get logger and set level
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)

    # Remove any existing handlers and add our handlers
    logger.handlers = []
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    # Set Flask logger to use our handlers
    app.logger.handlers = logger.handlers

    return logger
