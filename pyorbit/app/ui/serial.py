from typing import Union
import serial.tools.list_ports
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtGui import QTextCharFormat, QColor
from loguru import logger
from pyorbit.serial.client import SerialClient
from pyorbit.serial.observers import ConsoleObserver
from pyorbit.app.main import pyorbit, Settings, AppSettings


class SerialTargetSelect(QtWidgets.QComboBox):
    """ Behavioral class for the serial target combo box. """

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self._selected_target = ""
        self._com_ports = []
        self.activated.connect(self._serial_target_clicked)
        self._rescan()

        # Load a previously selected target from settings.
        old_target = AppSettings.value(Settings.SERIAL_TARGET)
        if old_target in self._com_ports:
            self._selected_target = old_target
            self.setCurrentText(self._selected_target)

    @property
    def active_target(self) -> str:
        return self._selected_target

    @QtCore.pyqtSlot(int)
    def _serial_target_clicked(self, index: int) -> None:
        """
        Slot for when a serial target is selected from the combo box.
        Args:
            index: The index of the selected target in the combo box.

        Returns:
            None
        """
        # Update our selected target.
        self._selected_target = self._com_ports[index]
        logger.trace(f"Selected target: {self._selected_target}")
        self._rescan()

        # Update our settings cache
        AppSettings.setValue(Settings.SERIAL_TARGET, self._selected_target)

    def _rescan(self) -> None:
        """
        Rescan for serial ports and update the list of available ports.
        Returns:
            None
        """
        # Rebuild the list of available ports.
        self._com_ports = [port.device for port in serial.tools.list_ports.comports()]
        self.clear()
        for port in self._com_ports:
            self.addItem(port)

        # If previously selected target is still available, select it.
        if self._selected_target:
            self.setCurrentText(self._selected_target)


class SerialConnectButton(QtWidgets.QPushButton):
    """ Behavioral class for the serial connect button. """

    requestNewConnectionStateSignal = QtCore.pyqtSignal(bool)

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self._connected = False
        self.setText("Connect")
        self.clicked.connect(self._connect_clicked)

    @QtCore.pyqtSlot()
    def _connect_clicked(self) -> None:
        logger.trace("Serial connection button clicked")
        if self._connected:
            # Notify the serial connection thread that we want to disconnect.
            self.setText("Connect")
            self.requestNewConnectionStateSignal.emit(False)
            self._connected = False
        else:
            # Notify the serial connection thread that we want to connect. We'll wait for the online
            # signal to confirm that the connection was successful and update the text appropriately.
            self.setText("Attaching")
            self.requestNewConnectionStateSignal.emit(True)

    @QtCore.pyqtSlot(bool)
    def notify_connection_state_change(self, state: bool) -> None:
        """
        Slot method for notifying the button of a change in the serial connection state.
        Args:
            state: The new state of the serial connection.

        Returns:
            None
        """
        self._connected = state
        if self._connected:
            self.setText("Detach")
        else:
            self.setText("Connect")


class SerialClearConsoleButton(QtWidgets.QPushButton):
    """ Behavioral class for the serial clear console button. """

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self.clicked.connect(self._clear_clicked)

    @QtCore.pyqtSlot()
    def _clear_clicked(self) -> None:
        """
        Slot method for clearing the serial console.
        Returns:
            None
        """
        console = pyorbit().findChild(QtWidgets.QScrollArea, "consoleWindow")  # type: SerialConsoleVisualizer
        console.clear()


class SerialConsoleVisualizer(QtWidgets.QScrollArea):

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        # Initialize the text widget, which is the actual console.
        self._text_widget = QtWidgets.QPlainTextEdit(parent)
        self._text_widget.setReadOnly(True)

    def init_console_window(self) -> None:
        """
        Adds the text widget to the console widget.
        Returns:
            None
        """
        self.setWidget(self._text_widget)

    def clear(self) -> None:
        """
        Clears the console.
        Returns:
            None
        """
        self._text_widget.clear()

    @QtCore.pyqtSlot(str)
    def add_app_console_message(self, message: str) -> None:
        """
        Slot method for logging a message from the application to the console widget.
        Args:
            message: Message to log.

        Returns:
            None
        """
        new_msg = message.strip("\n").strip("\r") + "\r\n"
        text_format = QTextCharFormat()

        if "INFO" in new_msg:
            text_format.setForeground(QColor('blue'))
        elif "WARNING" in new_msg:
            text_format.setForeground(QColor('orange'))
        elif "ERROR" in new_msg:
            text_format.setForeground(QColor('red'))
        elif "TRACE" in new_msg:
            text_format.setForeground(QColor('gray').darker(150))

        self._text_widget.textCursor().insertText(message, text_format)
        self._text_widget.verticalScrollBar().setValue(self._text_widget.verticalScrollBar().maximum())
