
from PyQt5 import QtCore
from loguru import logger
from threading import Event
from pyorbit.serial_client import SerialClient


class SerialConnection(QtCore.QThread):

    onOpenSignal = QtCore.pyqtSignal(SerialClient)
    onCloseSignal = QtCore.pyqtSignal()

    def __init__(self, parent: QtCore.QObject = None):
        super().__init__(parent)
        self._kill_signal = Event()
        self._kill_signal.clear()
        self._selected_target = ""
        self._client = None

    def run(self) -> None:
        while not self._kill_signal.is_set():
            QtCore.QCoreApplication.processEvents()
            self.msleep(10)

    @QtCore.pyqtSlot()
    def tear_down(self) -> None:
        logger.debug("Tearing down SerialConnection thread.")
        self._disconnect()
        self._kill_signal.set()

    @QtCore.pyqtSlot(bool)
    def connect_button_clicked(self, state: bool) -> None:
        logger.trace(f"Serial connection button clicked: {state}")
        if state:
            self._connect()
        elif not state:
            self._disconnect()

    @QtCore.pyqtSlot(str)
    def selected_target(self, target: str) -> None:
        self._selected_target = target

    def _connect(self) -> None:
        if not self._client:
            logger.info(f"Opening serial connection to {self._selected_target}")
            self._client = SerialClient(self._selected_target, baudrate=2000000)
            self.onOpenSignal.emit(self._client)

    def _disconnect(self) -> None:
        if self._client:
            logger.info("Closing serial connection")
            self._client.close()
            self._client = None
            self.onCloseSignal.emit()