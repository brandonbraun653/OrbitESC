from typing import Optional
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication
from loguru import logger
from pyorbit.serial.client import SerialClient
from threading import Event


def get_serial_client() -> Optional[SerialClient]:
    """
    Returns:
        Returns the serial client object from the serial connection manager.
    """
    return SerialConnectionManagerSingleton().serial_client


class SerialConnectionManagerSingleton(QtCore.QThread):
    """ Manages the serial connection to the target device, reacting to GUI commands to connect and disconnect. """

    onOpenSignal = QtCore.pyqtSignal()
    onCloseSignal = QtCore.pyqtSignal()
    escOnlineStatusSignal = QtCore.pyqtSignal(bool)

    CONNECTION_BAUD_RATE = 2000000

    __instance = None

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(SerialConnectionManagerSingleton, cls).__new__(cls)
            cls.__initialized = False
        return cls.__instance

    def __init__(self, parent: QtCore.QObject = None):
        if not self.__initialized:
            self.__initialized = True
            super().__init__(parent)
            self._client = None  # type: Optional[SerialClient]
            self._online = False
            self._kill_event = Event()

    def run(self) -> None:
        while not self._kill_event.is_set():
            self.msleep(10)
            QtCore.QCoreApplication.processEvents()

            self._notify_availability_change()

    @property
    def serial_client(self) -> SerialClient:
        return self._client

    @QtCore.pyqtSlot()
    def tear_down(self) -> None:
        """
        Slot method to cleanly tear down the serial connection thread.
        Returns:
            None
        """
        logger.debug("Tearing down SerialConnection thread.")
        self._disconnect()
        self.quit()
        self._kill_event.set()

    @QtCore.pyqtSlot(bool)
    def connect_button_clicked(self, state: bool) -> None:
        """
        Slot method for handling the serial connection button click event.
        Args:
            state: The state of the button (True = clicked, False = unclicked)

        Returns:
            None
        """
        logger.trace(f"Serial connection button clicked: {state}")
        if state:
            self._connect()
        elif not state:
            self._disconnect()

    def _connect(self) -> None:
        """
        Opens a connection to the target device using the serial client.
        Returns:
            None
        """
        from pyorbit.app.ui.serial import SerialTargetSelect

        if not self._client:
            window = QApplication.activeWindow()
            selector = window.findChild(QtWidgets.QComboBox, "serialTargetComboBox")  # type: SerialTargetSelect

            logger.info(f"Opening serial connection to {selector.active_target}")
            self._client = SerialClient(selector.active_target, baudrate=self.CONNECTION_BAUD_RATE)
            self.onOpenSignal.emit()

    def _disconnect(self) -> None:
        """
        Closes the serial connection to the target device.
        Returns:
            None
        """
        if self._client:
            logger.info("Closing serial connection")
            self.onCloseSignal.emit()
            self._client.close()
            self._client = None

    def _notify_availability_change(self) -> None:
        """
        Notifies the GUI of a change in the serial connection availability.
        Returns:
            None
        """
        if not self._client:
            return

        if self._client and not self._online and self._client.is_online:
            self._online = True
            self.escOnlineStatusSignal.emit(self._online)
        elif self._client and self._online and not self._client.is_online:
            self._online = False
            self.escOnlineStatusSignal.emit(self._online)
