from collections import deque
from typing import Union

import numpy as np
from PyQt5 import QtCore
from pyqtgraph import PlotWidget

from pyorbit.serial_client import SerialClient
from pyorbit.serial_messages import SystemDataMessage
from pyorbit.observer import MessageObserver


class LiveDataPlotter(MessageObserver, PlotWidget):

    class MessageProxy(QtCore.QObject):
        messageSignal = QtCore.pyqtSignal(str)

        def __init__(self, parent: QtCore.QObject):
            super().__init__(parent)

        def emit_message(self, message: str) -> None:
            self.messageSignal.emit(message)

    def __init__(self, parent: QtCore.QObject = None):
        MessageObserver.__init__(self, func=self._system_data_observer, msg_type=SystemDataMessage)
        PlotWidget.__init__(self, parent)

        self._serial_client = None  # type: Union[SerialClient, None]

        length = 1000
        self.y_axis = deque(np.zeros(length), maxlen=length)
        self.x_axis = deque(np.zeros(length), maxlen=length)
        self.getPlotItem().setRange(yRange=[-1, 1])
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_plot)

        self._data_line = self.plot(np.vstack((np.array(self.x_axis), np.array(self.y_axis))).transpose(), pen='r')

    @QtCore.pyqtSlot(SerialClient)
    def attach_serial_client(self, client: SerialClient) -> None:
        self._serial_client = client
        self._serial_client.com_pipe.subscribe_observer(self)
        self.timer.start(50)

    @QtCore.pyqtSlot()
    def detach_serial_client(self) -> None:
        self._serial_client = None
        self.timer.stop()

    def _system_data_observer(self, msg: SystemDataMessage):
        adc_data = msg.convert_to_message_type()
        self.x_axis.append(adc_data.timestamp / 1e6)
        self.y_axis.append(adc_data.phase_a)

    def _update_plot(self) -> None:
        y = np.array(self.y_axis)
        x = np.array(self.x_axis)
        new_data = np.vstack((x, y)).transpose()
        self._data_line.setData(new_data)
        self.enableAutoRange('x', True)
