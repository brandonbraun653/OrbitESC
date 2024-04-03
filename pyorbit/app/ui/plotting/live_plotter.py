import collections
import random
import time
import math
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QThread, pyqtSignal
from pyqtgraph import PlotWidget
from pyqtgraph.Qt import QtCore


class LivePlotManager(QtCore.QObject):
    """ Manager class for handling live plotting requests from other widgets """

    def __init__(self, parent: QtCore.QObject):
        super().__init__(parent)
        self.plotters = []

    def request_plot(self) -> None:
        plotter = DynamicPlotter(sample_interval=0.01, time_window=10.)
        plotter.sigClosed.connect(lambda: self.remove_plotter(plotter))
        self.plotters.append(plotter)

    def remove_plotter(self, plotter) -> None:
        print("Removing plotter")
        self.plotters.remove(plotter)


class DataThread(QThread):
    sigUpdate = pyqtSignal()

    def __init__(self, sample_interval=0.1, time_window=10.):
        QThread.__init__(self)

        # Data stuff
        self._interval = int(sample_interval * 1000)
        self._buf_size = int(time_window / sample_interval)
        self.data_buffer = collections.deque([0.0] * self._buf_size, self._buf_size)
        self.x = np.linspace(-time_window, 0.0, self._buf_size)
        self.y = np.zeros(self._buf_size, dtype=float)

        # QTimer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(self._interval)

    @staticmethod
    def getdata():
        frequency = 5
        noise = random.normalvariate(0., 1.)
        new = 10. * math.sin(time.time() * frequency * 2 * math.pi) + noise
        return new

    def update_plot(self):
        self.data_buffer.append(self.getdata())
        self.y[:] = self.data_buffer
        self.sigUpdate.emit()


class DynamicPlotter(PlotWidget):
    sigClosed = pyqtSignal()

    def __init__(self, sample_interval=0.1, time_window=10., size=(800, 350)):
        PlotWidget.__init__(self)

        # PyQtGraph stuff
        self.setWindowTitle("dynamic plotting")
        self.show()
        self.resize(*size)
        self.showGrid(x=True, y=True)
        self.setLabel('left', 'amplitude', 'V')
        self.setLabel('bottom', 'time', 's')
        self.curve = self.plot(pen=(255, 0, 0))

        # DataThread
        self.data_thread = DataThread(sample_interval, time_window)
        self.data_thread.sigUpdate.connect(self.update_plot)
        self.data_thread.start()

    def closeEvent(self, a0):
        super().closeEvent(a0)
        self.data_thread.quit()
        self.sigClosed.emit()

    def update_plot(self):
        self.curve.setData(self.data_thread.x, self.data_thread.y)
