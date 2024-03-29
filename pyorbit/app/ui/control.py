from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication
from loguru import logger
from pyorbit.serial.utility import has_serial_client


class EmergencyStopButton(QtWidgets.QPushButton):
    """ Custom handler for immediately halting the ESC power stage"""

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self.clicked.connect(self._on_clicked)

    @QtCore.pyqtSlot()
    @has_serial_client
    def _on_clicked(self) -> None:
        """
        Slot method for clearing the serial console.
        Returns:
            None
        """
        window = QApplication.activeWindow()
        logger.error("Emergency stop command not implemented")


class SystemResetButton(QtWidgets.QPushButton):
    """ Custom handler for immediately halting the ESC power stage"""

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self.clicked.connect(self._on_clicked)

    @QtCore.pyqtSlot()
    @has_serial_client
    def _on_clicked(self) -> None:
        """
        Slot method for clearing the serial console.
        Returns:
            None
        """
        window = QApplication.activeWindow()
        if window.serial_client.system_reset():
            logger.info("System reset OK")
        else:
            logger.error("Failed to send system reset command")
