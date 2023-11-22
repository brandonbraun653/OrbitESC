from __future__ import annotations

import abc
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List
import pyqtgraph
from PyQt5.QtGui import QPen
from PyQt5.QtWidgets import QApplication, QCheckBox, QWidget, QTabWidget
from loguru import logger
import numpy as np
from PyQt5 import QtCore
from pyqtgraph import PlotWidget, PlotDataItem

from pyorbit.app.main import AppSettings, Settings
from pyorbit.serial.messages import SystemDataMessage
from pyorbit.serial.parameters import ParameterId
from pyorbit.observers import MessageObserver


@dataclass
class PlotAttributes:
    enabled: bool = field(default=False)
    data_field: str = field(default="")
    plot: PlotDataItem = field(default=None)
    pen: QPen = field(default=None)
    data: deque = field(default_factory=lambda: deque([], maxlen=1000))
    time: deque = field(default_factory=lambda: deque([], maxlen=1000))


class AbstractDataPlot(metaclass=abc.ABCMeta):

    @abc.abstractmethod
    def name(self) -> str:
        pass

    @abc.abstractmethod
    def data_observer(self) -> MessageObserver:
        pass

    @abc.abstractmethod
    def attributes(self) -> List[PlotAttributes]:
        pass

    @abc.abstractmethod
    def is_visible(self) -> bool:
        pass


class MotorCurrentPlot(AbstractDataPlot):
    # class MessageProxy(QtCore.QObject):
    #     messageSignal = QtCore.pyqtSignal(str)
    #
    #     def __init__(self, parent: QtCore.QObject):
    #         super().__init__(parent)
    #
    #     def emit_message(self, message: str) -> None:
    #         self.messageSignal.emit(message)

    def __init__(self):
        self._observer = MessageObserver(self._system_data_observer, SystemDataMessage)
        self._phase_a_attr = PlotAttributes(enabled=True, data_field="PhaseACurrent",
                                            pen=pyqtgraph.mkPen(color=(255, 0, 0), width=2, style=QtCore.Qt.SolidLine))
        self._phase_b_attr = PlotAttributes(enabled=True, data_field="PhaseBCurrent",
                                            pen=pyqtgraph.mkPen(color=(0, 255, 0), width=2, style=QtCore.Qt.SolidLine))
        self._phase_c_attr = PlotAttributes(enabled=True, data_field="PhaseCCurrent",
                                            pen=pyqtgraph.mkPen(color=(0, 0, 255), width=2, style=QtCore.Qt.SolidLine))
        self._attributes = [self._phase_a_attr, self._phase_b_attr, self._phase_c_attr]

    def name(self) -> str:
        return "Motor Currents"

    def data_observer(self) -> MessageObserver:
        return self._observer

    def attributes(self) -> List[PlotAttributes]:
        return self._attributes

    def stream_parameter(self) -> ParameterId:
        return ParameterId.StreamPhaseCurrents

    def is_visible(self) -> bool:
        window = QApplication.activeWindow()
        tab = window.findChild(QTabWidget, "tabDataSelection")
        index = tab.currentIndex()
        return index == 0

    def _system_data_observer(self, msg: SystemDataMessage):
        data = msg.convert_to_message_type()
        if isinstance(data, SystemDataMessage.ADCPhaseCurrents):
            time_in_sec = data.timestamp / 1e6
            self._phase_a_attr.time.append(time_in_sec)
            self._phase_a_attr.data.append(data.phase_a)
            self._phase_b_attr.time.append(time_in_sec)
            self._phase_b_attr.data.append(data.phase_b)
            self._phase_c_attr.time.append(time_in_sec)
            self._phase_c_attr.data.append(data.phase_c)


class MotorSpeedPositionPlot(AbstractDataPlot):

    def __init__(self):
        self._observer = MessageObserver(self._system_data_observer, SystemDataMessage)
        self._speed_attr = PlotAttributes(enabled=True, data_field="MotorSpeed",
                                          pen=pyqtgraph.mkPen(color=(0, 0, 255), width=2, style=QtCore.Qt.SolidLine))
        self._pos_attr = PlotAttributes(enabled=True, data_field="MotorPosition",
                                        pen=pyqtgraph.mkPen(color=(0, 255, 0), width=2, style=QtCore.Qt.SolidLine))
        self._attributes = [self._speed_attr, self._pos_attr]

    def name(self) -> str:
        return "Motor Speed and Position"

    def data_observer(self) -> MessageObserver:
        return self._observer

    def attributes(self) -> List[PlotAttributes]:
        return self._attributes

    def is_visible(self) -> bool:
        window = QApplication.activeWindow()
        tab = window.findChild(QTabWidget, "tabDataSelection")
        index = tab.currentIndex()
        return index == 2

    def _system_data_observer(self, msg: SystemDataMessage):
        data = msg.convert_to_message_type()
        if isinstance(data, SystemDataMessage.StateEstimates):
            time_in_sec = data.timestamp / 1e6
            self._speed_attr.time.append(time_in_sec)
            self._speed_attr.data.append(data.speed * (180 / np.pi))
            # self._pos_attr.time.append(time_in_sec)
            # self._pos_attr.data.append(data.position * (180 / np.pi))


class LiveDataPlotter(PlotWidget):
    PLOT_REFRESH_RATE_MS = 50

    def __init__(self, parent: QtCore.QObject = None):
        PlotWidget.__init__(self, parent)

        self._data_plot = None  # type: Optional[AbstractDataPlot]
        if parent.objectName() == "tabMotorCurrent":
            self._data_plot = MotorCurrentPlot()
        elif parent.objectName() == "tabSpeedPosition":
            self._data_plot = MotorSpeedPositionPlot()
        else:
            raise RuntimeError("Unknown plotter widget parent")

        self._plot_refresh_timer = QtCore.QTimer()
        self._plot_refresh_timer.timeout.connect(self._update_plot)
        self._plot_refresh_timer.start(LiveDataPlotter.PLOT_REFRESH_RATE_MS)
        self.getPlotItem().setRange(yRange=[-1, 1])

    @QtCore.pyqtSlot()
    def serial_connect(self) -> None:
        # Register the plot's data observer with the serial client
        window = QApplication.activeWindow()
        if window:
            window.serial_client.com_pipe.subscribe_observer(self._data_plot.data_observer())

        # Update the state of the auto-scale checkbox
        if AppSettings.contains(Settings.PLOT_AUTO_SCALE):
            window.autoScaleCheckBox.setCheckState(QtCore.Qt.CheckState(AppSettings.value(Settings.PLOT_AUTO_SCALE)))

    @QtCore.pyqtSlot()
    def serial_disconnect(self) -> None:
        # Unregister the plot's data observer with the serial client
        window = QApplication.activeWindow()
        if window:
            window.serial_client.com_pipe.unsubscribe(self._data_plot.data_observer().unique_id)

    @QtCore.pyqtSlot(int)
    def toggle_live_data_stream(self, state: int) -> None:
        """
        Slot for when the live data checkbox is toggled.
        Args:
            state: The state of the checkbox

        Returns:
            None
        """
        window = QApplication.activeWindow()
        serial_client = window.serial_client
        check_box = self.sender()  # type: QCheckBox

        if not self._data_plot.is_visible():
            return

        if state == QtCore.Qt.CheckState.Checked:
            # First check to see if the serial target is online. If not available and we are trying to go live, then
            # we need to disable the live state.
            if not serial_client or not serial_client.is_online:
                logger.error("Serial client is not available")
                check_box.setCheckState(QtCore.Qt.CheckState.Unchecked)
                return

            # Try to request a live stream of the data
            logger.info(f"Enabling live stream of {self._data_plot.name()}")
            if window.serial_client.stream_phase_currents(True):
                self._plot_refresh_timer.start(LiveDataPlotter.PLOT_REFRESH_RATE_MS)
                AppSettings.setValue(Settings.PLOT_LIVE_DATA, True)
            else:
                logger.error(f"Failed to live stream {self._data_plot.name()}")
                check_box.setCheckState(QtCore.Qt.CheckState.Unchecked)

        else:
            if not serial_client or not serial_client.is_online:
                return

            logger.info(f"Disabling live stream of {self._data_plot.name()}")
            if window.serial_client.stream_phase_currents(False):
                self._plot_refresh_timer.stop()
                AppSettings.setValue(Settings.PLOT_LIVE_DATA, False)
            else:
                logger.error(f"Failed to disable live stream {self._data_plot.name()}")
                check_box.setCheckState(QtCore.Qt.CheckState.Checked)

    @QtCore.pyqtSlot(int)
    def toggle_auto_scale(self, state: int) -> None:
        AppSettings.setValue(Settings.PLOT_AUTO_SCALE, state)

    @QtCore.pyqtSlot()
    def clear_plot(self) -> None:
        """
        Clears the plot of all data.
        Returns:
            None
        """
        for attr in self._data_plot.attributes():
            attr.time.clear()
            attr.data.clear()

        self._update_plot()

    def _update_plot(self) -> None:
        """
        Updates the plot with the latest data from each data field.
        Returns:
            None
        """
        for attr in self._data_plot.attributes():
            # Create the new plot if it doesn't exist yet
            if attr.plot is None:
                attr.plot = self.plot(np.vstack((np.array(attr.time), np.array(attr.data))).transpose(), pen=attr.pen)

            # Update the plot with the latest data
            if attr.enabled:
                x = np.array(attr.time)
                y = np.array(attr.data)
                attr.plot.setData(x, y)

        if AppSettings.value(Settings.PLOT_AUTO_SCALE) == QtCore.Qt.CheckState.Checked:
            self.enableAutoRange('xy', True)
        else:
            self.enableAutoRange('xy', False)
