from PyQt5 import QtWidgets

from pyorbit.app.util import AppSettings, DuplicatedWidgetId
from system_data_pb2 import SystemDataId


class InnerLoopCustomPlotField(DuplicatedWidgetId, QtWidgets.QComboBox):
    """ Common handler class for the inner loop custom plot field selectors """

    instance_counter = 0

    def __init__(self, parent: QtWidgets.QWidget):
        DuplicatedWidgetId.__init__(self)
        QtWidgets.QComboBox.__init__(self, parent)

        # Grab all system data items with a value greater than __INDIVIDUAL_DATA_STREAMS_START
        self.addItem("Select Field", -1)
        for item in SystemDataId.DESCRIPTOR.values:
            if item.number > SystemDataId.INDIVIDUAL_DATA_STREAMS_START:
                self.addItem(item.name, item.number)

        if AppSettings.contains(self.widget_id):
            self.setCurrentIndex(int(AppSettings.value(self.widget_id)))
        else:
            self.setCurrentIndex(0)

        self.currentIndexChanged.connect(self._on_current_index_changed)

    def _on_current_index_changed(self, index: int) -> None:
        AppSettings.setValue(self.widget_id, index)
        print(index)
