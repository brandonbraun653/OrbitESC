from functools import wraps

from PyQt5.QtWidgets import QApplication
from loguru import logger


def has_serial_client(method):
    """
    Decorator to check a serial connection is available before allowing a method
    call to proceed. This prevents duplicate code on a variety of methods.
    """
    @wraps(method)
    def _impl(*method_args, **method_kwargs):
        window = QApplication.activeWindow()
        client = window.serial_client
        if client:
            return method(*method_args, **method_kwargs)
        else:
            logger.error(f"{repr(method)}: Serial client not available")

    return _impl

