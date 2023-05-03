from __future__ import annotations
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Union, Optional, List

import pyqtgraph
from loguru import logger
import re
import numpy as np
from PyQt5 import QtCore, QtWidgets
from pyqtgraph import PlotWidget, PlotDataItem
from pyorbit.serial_client import SerialClient
from pyorbit.serial_messages import SystemDataMessage, ParameterId
from pyorbit.observer import MessageObserver


class LiveDataPlotter(MessageObserver, PlotWidget):
    PLOT_REFRESH_RATE_MS = 50
    PLOT_SAMPLE_HISTORY = 1000

    class MessageProxy(QtCore.QObject):
        messageSignal = QtCore.pyqtSignal(str)

        def __init__(self, parent: QtCore.QObject):
            super().__init__(parent)

        def emit_message(self, message: str) -> None:
            self.messageSignal.emit(message)

    class DataFields(Enum):
        PhaseACurrent = "PhaseACurrent"
        PhaseBCurrent = "PhaseBCurrent"
        PhaseCCurrent = "PhaseCCurrent"
        PhaseAVoltage = "PhaseAVoltage"
        PhaseBVoltage = "PhaseBVoltage"
        PhaseCVoltage = "PhaseCVoltage"
        PhaseACommand = "PhaseACommand"
        PhaseBCommand = "PhaseBCommand"
        PhaseCCommand = "PhaseCCommand"

        @classmethod
        def as_list(cls) -> List[str]:
            return [f.value for f in cls]

        @classmethod
        def phase_currents(cls) -> List[LiveDataPlotter.DataFields]:
            return [cls.PhaseACurrent, cls.PhaseBCurrent, cls.PhaseCCurrent]

    class PlotColors(Enum):
        LightBlue = 0x93B7BE
        MintCream = 0xF1FFFA
        TimberWolf = 0xD5C7BC
        RoseTaupe = 0x785964
        DarkJungleGreen = 0x1C2321
        Asparagus = 0x90A959
        EarthYellow = 0xE9B872
        Redwood = 0xA63D40
        AirForceBlue = 0x6494AA
        MyrtleGreen = 0x508484
        Mint = 0x79C99E
        CoolGray = 0x7D84B2
        VistaBlue = 0x8FA6CB
        Straw = 0xC7D66D
        MountbattenPink = 0xAD7A99
        GoldenBrown = 0x9C6615
        Eggplant = 0x6D435A
        BrightPink = 0xFF6978

        @classmethod
        def as_int_list(cls) -> List[int]:
            return [f.value for f in cls]

        @classmethod
        def as_rgb_list(cls) -> List[(int, int, int)]:
            return [cls._int_to_rgb(f.value) for f in cls]

        @classmethod
        def _int_to_rgb(cls, value: int) -> (int, int, int):
            return (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF

    @dataclass
    class PlotAttributes:
        enabled: bool = field(default=False)
        data_field: str = field(default="")
        plot: PlotDataItem = field(default=None)
        color: tuple = field(default=(0, 0, 0))
        data: deque = field(default_factory=lambda: LiveDataPlotter._default_data_queue())
        time: deque = field(default_factory=lambda: LiveDataPlotter._default_data_queue())

    def __init__(self, parent: QtCore.QObject = None):
        MessageObserver.__init__(self, func=self._system_data_observer, msg_type=SystemDataMessage)
        PlotWidget.__init__(self, parent)

        self._serial_client = None  # type: Union[SerialClient, None]
        self._live_plotting = False
        self._plot_attributes = [LiveDataPlotter.PlotAttributes() for _ in range(len(LiveDataPlotter.DataFields))]
        self._plot_refresh_timer = QtCore.QTimer()
        self._plot_refresh_timer.timeout.connect(self._update_plot)
        self._need_phase_currents = False

        for idx in range(len(self._plot_attributes)):
            self._plot_attributes[idx].color = LiveDataPlotter.PlotColors.as_rgb_list()[idx]

    def init_selection_layout(self, layout: QtWidgets.QGridLayout) -> None:
        """
        Initializes the plot selection objects contained within the layout. These are a collection of
        checkboxes and combo boxes that allow the user to select which plots to display.
        Args:
            layout: Layout that contains the plot selection objects

        Returns:
            None
        """
        # Set the default y-axis range
        self.getPlotItem().setRange(yRange=[-1, 1])
        self.enableAutoRange('xy', True)

        # Initialize the plot control objects
        for idx in range(layout.count()):
            item = layout.itemAt(idx)
            widget = item.widget()

            if widget is None:
                continue

            if isinstance(widget, QtWidgets.QCheckBox):
                widget.checked = False
                widget.stateChanged.connect(self._checkbox_state_changed)
            elif isinstance(widget, QtWidgets.QComboBox):
                widget.addItems(LiveDataPlotter.DataFields.as_list())
                widget.setCurrentIndex(-1)
                widget.currentIndexChanged.connect(self._combo_box_selected)
            else:
                raise ValueError(f"Unhandled widget type: {type(widget)}")

    @staticmethod
    def _object_index(object_name: str) -> int:
        """
        Extracts the index from the object name. The object name is expected to be in the format
        of "<object_name>_<index>", where index is an integer. The naming convention was set in
        the Qt Designer manually.

        Args:
            object_name: String containing the object name

        Returns:
            Integer index
        """
        match = re.match(r".*_(\d+)", object_name)
        if match is None:
            raise ValueError(f"Object name does not contain an index: {object_name}")
        return int(match.group(1))

    @QtCore.pyqtSlot(SerialClient)
    def attach_serial_client(self, client: SerialClient) -> None:
        # Cache the client and subscribe to the data packet this class is interested in
        self._serial_client = client
        self._serial_client.com_pipe.subscribe_observer(self)

        # Enable the plotter by default
        self._live_plotting = True
        self._plot_refresh_timer.start(LiveDataPlotter.PLOT_REFRESH_RATE_MS)

    @QtCore.pyqtSlot()
    def detach_serial_client(self) -> None:
        self._serial_client = None
        self._plot_refresh_timer.stop()

    @QtCore.pyqtSlot()
    def pause_resume_clicked(self) -> None:
        if self._live_plotting:
            self._plot_refresh_timer.stop()
            self._live_plotting = False
            self.sender().setText("Enable")
        else:
            self._plot_refresh_timer.start(LiveDataPlotter.PLOT_REFRESH_RATE_MS)
            self._live_plotting = True
            self.sender().setText("Disable")

    @QtCore.pyqtSlot(int)
    def _checkbox_state_changed(self, state: int) -> None:
        # Update the enabled/disabled field
        obj_idx = self._object_index(self.sender().objectName())
        attr = self._plot_attributes[obj_idx]

        if state == QtCore.Qt.Checked:
            attr.data = self._default_data_queue()
            attr.time = self._default_data_queue()
            if attr.plot is None:
                attr.plot = self.plot(np.vstack((np.array(attr.time), np.array(attr.data))).transpose(),
                                      pen=pyqtgraph.mkPen(color=attr.color, width=2, style=QtCore.Qt.SolidLine))
            else:
                attr.plot.clear()
            attr.enabled = True
        else:
            attr.enabled = False
            attr.plot.clear()

        # Perform debug pipe bandwidth optimization measures
        self._process_phase_current_stream_status()

    @QtCore.pyqtSlot(int)
    def _combo_box_selected(self, index: int) -> None:
        obj_idx = self._object_index(self.sender().objectName())
        self._plot_attributes[obj_idx].data_field = self.DataFields.as_list()[index]

    def _system_data_observer(self, msg: SystemDataMessage):
        data = msg.convert_to_message_type()
        if data is None:
            return
        elif isinstance(data, SystemDataMessage.ADCPhaseCurrents):
            time_in_sec = data.timestamp / 1e6
            for f in self.DataFields.phase_currents():
                # Make sure we have a valid field
                attributes = self._get_field_attributes(f)
                if attributes is None:
                    continue

                # Update the data fields
                attributes.time.append(time_in_sec)
                if f == self.DataFields.PhaseACurrent:
                    attributes.data.append(data.phase_a)
                elif f == self.DataFields.PhaseBCurrent:
                    attributes.data.append(data.phase_b)
                elif f == self.DataFields.PhaseCCurrent:
                    attributes.data.append(data.phase_c)

    def _update_plot(self) -> None:
        for attr in self._plot_attributes:
            if attr.enabled:
                x = np.array(attr.time)
                y = np.array(attr.data)
                attr.plot.setData(x, y)

    def _process_phase_current_stream_status(self) -> None:
        """
        Checks the status of all phase current plots and enables/disables the embedded software
        data stream on-demand. This helps to save bandwidth if the data isn't actually being used.

        Returns:
            None
        """
        # Check if any phase current plots are enabled
        phase_current_fields = [self.DataFields.PhaseACurrent, self.DataFields.PhaseBCurrent,
                                self.DataFields.PhaseCCurrent]
        any_phase_currents_enabled = any([self._plot_field_is_enabled(f.value) for f in phase_current_fields])

        # Enable/disable the phase current stream based on the current status
        if self._need_phase_currents and not any_phase_currents_enabled:
            logger.trace("Disabling phase current stream")
            if self._serial_client.parameter.set(ParameterId.StreamPhaseCurrents, False):
                self._need_phase_currents = False
            else:
                logger.error("Failed to disable phase current stream")
        elif not self._need_phase_currents and any_phase_currents_enabled:
            logger.trace("Enabling phase current stream")
            if self._serial_client.parameter.set(ParameterId.StreamPhaseCurrents, True):
                self._need_phase_currents = True
            else:
                logger.error("Failed to enable phase current stream")

    def _plot_field_is_enabled(self, data_field: str):
        """
        Checks if the checkbox is enabled for the given data field.
        Args:
            data_field: One of the supported data fields that can be plotted

        Returns:
            True if the plot option is enabled, False otherwise
        """
        for attr in self._plot_attributes:
            if attr.data_field == data_field:
                return attr.enabled
        return False

    def _get_field_attributes(self, data_field: LiveDataPlotter.DataFields) -> Optional[LiveDataPlotter.PlotAttributes]:
        """
        Returns the PlotAttribute object for the given data field, if it exists.

        Args:
            data_field: One of the supported data fields that can be plotted

        Returns:
            PlotAttribute object if it exists, None otherwise
        """
        for attr in self._plot_attributes:
            if attr.data_field == data_field.value:
                return attr
        return None

    @classmethod
    def _default_data_queue(cls) -> deque:
        return deque(np.zeros(LiveDataPlotter.PLOT_SAMPLE_HISTORY), maxlen=LiveDataPlotter.PLOT_SAMPLE_HISTORY)
