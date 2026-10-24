from __future__ import annotations

from PyQt5 import QtCore

AppSettings = QtCore.QSettings("OrbitESC", "PyOrbit")


class DuplicatedWidgetId:
    """
    Helper class for tracking instance identifiers of widgets. Very useful for generating
    unique IDs when saving widget settings/configurations.
    """

    instance_counter = 0

    def __init__(self):
        self._id = f"{self.__class__.__name__}_{DuplicatedWidgetId.instance_counter}"
        DuplicatedWidgetId.instance_counter += 1

    @property
    def widget_id(self) -> str:
        return self._id
