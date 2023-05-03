from typing import Union
import time
import serial.tools.list_ports
from PyQt5 import QtCore, QtWidgets
from loguru import logger
from pyorbit.serial_client import SerialClient
from pyorbit.serial_observers import ConsoleObserver


class SerialTarget(QtWidgets.QComboBox):
    """ Behavioral class for the serial target combo box. """

    targetChosenSignal = QtCore.pyqtSignal(str)

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self._selected_target = ""
        self._com_ports = []
        self.activated.connect(self._serial_target_clicked)
        self._rescan()

    @QtCore.pyqtSlot(int)
    def _serial_target_clicked(self, index: int) -> None:
        self._selected_target = self._com_ports[index]
        self.targetChosenSignal.emit(self._selected_target)
        logger.trace(f"Selected target: {self._selected_target}")
        self._rescan()

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


class SerialConnect(QtWidgets.QPushButton):
    """ Behavioral class for the serial connect button. """

    stateChangeSignal = QtCore.pyqtSignal(bool)

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self._connected = False
        self.setText("Connect")
        self.clicked.connect(self._connect_clicked)

    @QtCore.pyqtSlot()
    def _connect_clicked(self) -> None:
        if self._connected:
            self.setText("Connect")
            self._connected = False
        else:
            self.setText("Disconnect")
            self._connected = True

        self.stateChangeSignal.emit(self._connected)
        logger.trace(f"Serial connection button clicked: {self._connected}")


class SerialConsoleVisualizer(QtWidgets.QScrollArea):

    class MessageProxy(QtCore.QObject):
        messageSignal = QtCore.pyqtSignal(str)

        def __init__(self, parent: QtCore.QObject):
            super().__init__(parent)

        def emit_message(self, message: str) -> None:
            self.messageSignal.emit(message)

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self._msg_proxy = self.MessageProxy(self)
        self._text_widget = QtWidgets.QPlainTextEdit(parent)
        self._text_widget.setReadOnly(True)
        self._serial_client = None  # type: Union[SerialClient, None]
        self._console_observer = ConsoleObserver(on_msg_rx=self._msg_proxy.emit_message)
        self._msg_proxy.messageSignal.connect(self.add_console_message)

    @QtCore.pyqtSlot(SerialClient)
    def attach_serial_client(self, client: SerialClient) -> None:
        self._serial_client = client
        self._serial_client.com_pipe.subscribe_observer(self._console_observer)
        self.setWidget(self._text_widget)

        time.sleep(1.0)
        if self._serial_client.is_online:
            self._text_widget.insertPlainText("Serial connection opened.\r\n")
        else:
            self._text_widget.insertPlainText("Serial connection failed.\r\n")

    @QtCore.pyqtSlot()
    def detach_serial_client(self) -> None:
        self._serial_client = None
        self._text_widget.insertPlainText("Serial connection closed.\r\n")

    @QtCore.pyqtSlot(str)
    def add_console_message(self, message: str) -> None:
        new_msg = message.strip("\n").strip("\r") + "\r\n"
        self._text_widget.insertPlainText(new_msg)
        self._text_widget.verticalScrollBar().setValue(self._text_widget.verticalScrollBar().maximum())
