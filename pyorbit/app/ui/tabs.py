from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication
from pyorbit.serial.parameters import ParameterId
from pyorbit.serial.utility import has_serial_client


class ParameterTabWidget(QtWidgets.QTabWidget):
    """ Custom handler for all the parameters displayed via QTabWidget widgets """

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self.currentChanged.connect(self._on_current_changed)

    def _on_current_changed(self, index: int) -> None:
        """
        Slot method for notifying the parameter of a change in the tab index.
        Args:
            index: The new tab index.

        Returns:
            None
        """
        pass
        # Ensure the ESC behaves as expected for the plotters
        # window = QApplication.activeWindow()
        # if window.autoScaleCheckBox.checkState() == QtCore.Qt.CheckState.Checked:
        #     self._enable_esc_live_stream_for_view()
        # else:
        #     self._disable_esc_live_stream_for_view()

    @QtCore.pyqtSlot()
    @has_serial_client
    def parameter_apply_clicked(self) -> None:
        """
        Saves all dirty parameters to the target node.
        Returns:
            None
        """
        window = QApplication.activeWindow()
        window.parameter_updater.applyRequest.emit(ParameterId.values())

    @QtCore.pyqtSlot()
    @has_serial_client
    def parameter_refresh_clicked(self) -> None:
        """
        Refreshes all parameters from the target node.
        Returns:
            None
        """
        window = QApplication.activeWindow()
        window.parameter_updater.refreshRequest.emit(ParameterId.values())

    @QtCore.pyqtSlot()
    def parameter_save_clicked(self) -> None:
        pass

    @QtCore.pyqtSlot()
    def parameter_load_clicked(self) -> None:
        pass

    def _enable_esc_live_stream_for_view(self) -> None:
        pass

    def _disable_esc_live_stream_for_view(self) -> None:
        pass