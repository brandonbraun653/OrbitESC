from PyQt5 import QtWidgets


class InnerLoopCustomPlotRate(QtWidgets.QDoubleSpinBox):
    """ Common handler for the inner loop custom plot rate selector for a particular field """

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self.setRange(0.1, 100)
        self.setSingleStep(0.1)
        self.setValue(1)
        self.valueChanged.connect(self._on_value_changed)

    def _on_value_changed(self, value: float) -> None:
        print(value)