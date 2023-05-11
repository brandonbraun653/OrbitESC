from PyQt5 import QtWidgets, QtCore
from pyorbit.app.parameters.util import update_parameter


class ParameterSpinBox(QtWidgets.QSpinBox):
    """ Custom handler for all the parameters displayed via QSpinBox widgets """

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self.valueChanged.connect(self._on_value_changed)

    @QtCore.pyqtSlot(int)
    def _on_value_changed(self, value: int) -> None:
        """
        Slot method for notifying the parameter of a change in the spin box value.
        Args:
            value: The new value of the spin box.

        Returns:
            None
        """
        update_parameter(self.sender(), value)


class ParameterDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    """ Custom handler for all the parameters displayed via QDoubleSpinBox widgets """

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self.valueChanged.connect(self._on_value_changed)

    @QtCore.pyqtSlot(float)
    def _on_value_changed(self, value: float) -> None:
        """
        Slot method for notifying the parameter of a change in the spin box value.
        Args:
            value: The new value of the spin box.

        Returns:
            None
        """
        update_parameter(self.sender(), value)


class ParameterLineEdit(QtWidgets.QLineEdit):
    """ Custom handler for all the parameters displayed via QLineEdit widgets """

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self.textChanged.connect(self._on_text_changed)

    @QtCore.pyqtSlot(str)
    def _on_text_changed(self, value: str) -> None:
        """
        Slot method for notifying the parameter of a change in the line edit value.
        Args:
            value: The new value of the line edit.

        Returns:
            None
        """
        update_parameter(self.sender(), value)
