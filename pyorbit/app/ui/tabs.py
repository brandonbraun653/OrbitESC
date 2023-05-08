from PyQt5 import QtWidgets, QtCore


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
        self.parent().value = index

    @QtCore.pyqtSlot()
    def parameter_apply_clicked(self) -> None:
        pass

    @QtCore.pyqtSlot()
    def parameter_refresh_clicked(self) -> None:
        pass
