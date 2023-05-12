from typing import List, Optional
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication
from loguru import logger
from pyorbit.serial.client import SerialClient
from pyorbit.serial.parameters import ParameterId


class ParameterUpdater(QtCore.QObject):
    """ Class for updating parameters in the background """

    refreshRequest = QtCore.pyqtSignal(list)
    refreshComplete = QtCore.pyqtSignal()
    applyRequest = QtCore.pyqtSignal(list)
    applyComplete = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self._client = None  # type: Optional[SerialClient]
        self.refreshRequest.connect(self.refresh)
        self.applyRequest.connect(self.apply)

    @QtCore.pyqtSlot()
    def on_serial_connect(self) -> None:
        """
        Slot method for handling the serial connection event.
        Returns:
            None
        """
        window = QApplication.activeWindow()
        self._client = window.serial_client

        self.refreshRequest.emit(ParameterId.values())

    @QtCore.pyqtSlot()
    def on_serial_disconnect(self) -> None:
        """
        Slot method for handling the serial disconnection event.
        Returns:
            None
        """
        self._client = None

    @QtCore.pyqtSlot(list)
    def refresh(self, parameter_ids: List[ParameterId]) -> None:
        """
        Refreshes the given parameters from the target node.
        Args:
            parameter_ids: The list of parameters to refresh.

        Returns:
            None
        """
        from pyorbit.app.parameters.util import parameter_id_lookup_cache

        for next_item in parameter_ids:
            try:
                parameter = parameter_id_lookup_cache[next_item]
                parameter.refresh(self._client)
            except KeyError:
                logger.warning(f"Could not find parameter {next_item} in cache.")
                continue

    @QtCore.pyqtSlot(list)
    def apply(self, parameter_ids: List[ParameterId]) -> None:
        """
        Applies the given parameters to the target node.
        Args:
            parameter_ids: The list of parameters to apply.

        Returns:
            None
        """
        from pyorbit.app.parameters.util import parameter_id_lookup_cache

        for next_item in parameter_ids:
            try:
                parameter = parameter_id_lookup_cache[next_item]
                parameter.apply(self._client)
            except KeyError:
                logger.warning(f"Could not find parameter {next_item} in cache.")
                continue
