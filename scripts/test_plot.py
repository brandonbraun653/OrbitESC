from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread, pyqtSignal
from pyqtgraph.Qt import QtCore
import pyqtgraph as pg

import collections
import random
import time
import math
import numpy as np


class DynamicPlotter(QThread):
    """
    https://github.com/ap--/python-live-plotting/blob/master/plot_pyqtgraph.py

    This class is an adaptation of the code in the referenced link. I was messing around
    a bit with high speed real time plotting for the project and happened across that code.
    I want to be able to build/observe multiple plots on-demand, so this class is a proof
    of concept to see if that's even possible.
    """

    # Signal for the main thread to draw updates
    update_needed = pyqtSignal()

    def __init__(self, sample_interval=0.1, time_window=10., size=(600, 350)):
        QThread.__init__(self)

        # Data stuff
        self._interval = int(sample_interval * 1000)
        self._buf_size = int(time_window / sample_interval)
        self.data_buffer = collections.deque([0.0] * self._buf_size, self._buf_size)
        self.x = np.linspace(-time_window, 0.0, self._buf_size)
        self.y = np.zeros(self._buf_size, dtype=float)
        # PyQtGraph stuff
        self.plt = pg.plot(title='Dynamic Plotting with PyQtGraph')
        self.plt.resize(*size)
        self.plt.showGrid(x=True, y=True)
        self.plt.setLabel('left', 'amplitude', 'V')
        self.plt.setLabel('bottom', 'time', 's')
        self.curve = self.plt.plot(self.x, self.y, pen=(255, 0, 0))
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
        self.curve.setData(self.x, self.y)
        self.update_needed.emit()


if __name__ == '__main__':
    app = QApplication([])
    plotters = [DynamicPlotter(sample_interval=0.01, time_window=10.) for _ in range(5)]

    for plotter in plotters:
        plotter.start()
        plotter.update_needed.connect(app.processEvents)

    app.exec_()
