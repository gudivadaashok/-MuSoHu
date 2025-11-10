import logging
import os
from logging.handlers import RotatingFileHandler

def setup_logging(app):
    """Configure logging for the Flask application."""
    
    # Create logs directory if it doesn't exist
    log_dir = '/app/logs'
    os.makedirs(log_dir, exist_ok=True)
    
    # Configure root logger
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s'
    )
    
    if not app.debug:
        # Production logging setup
        file_handler = RotatingFileHandler(
            os.path.join(log_dir, 'musohu.log'),
            maxBytes=10240000,  # 10MB
            backupCount=10
        )
        file_handler.setFormatter(logging.Formatter(
            '%(asctime)s %(levelname)s: %(message)s [in %(pathname)s:%(lineno)d]'
        ))
        file_handler.setLevel(logging.INFO)
        app.logger.addHandler(file_handler)
        
    else:
        # Development logging setup
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        console_handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        ))
        app.logger.addHandler(console_handler)
        
        # Also log to file in development
        file_handler = RotatingFileHandler(
            os.path.join(log_dir, 'musohu_dev.log'),
            maxBytes=1024000,  # 1MB
            backupCount=3
        )
        file_handler.setFormatter(logging.Formatter(
            '%(asctime)s %(levelname)s: %(message)s [in %(pathname)s:%(lineno)d]'
        ))
        file_handler.setLevel(logging.DEBUG)
        app.logger.addHandler(file_handler)
    
    app.logger.setLevel(logging.INFO)
    app.logger.info('MuSoHu logging configured')
    
    return app.logger