from PyQt5 import QtWidgets

from pyorbit.app.ui.plotting.live_plotter import LivePlotManager


class InnerLoopCustomPlotButton(QtWidgets.QPushButton):
    """ Common handler class for the inner loop custom plot button """

    def __init__(self, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self.clicked.connect(self._on_clicked)
        self.plot_manager = LivePlotManager(self)

    def _on_clicked(self) -> None:
        print("Button clicked")
        self.plot_manager.request_plot()
