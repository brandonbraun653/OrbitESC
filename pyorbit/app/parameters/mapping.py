from PyQt5 import QtWidgets

from pyorbit.app.parameters.abstract import AbstractParameter
from pyorbit.serial.parameters import ParameterId


class RampFirstOrderTerm(AbstractParameter):
    """ Class for representing the first order term parameter """

    def __init__(self):
        super().__init__()

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.RampControlFirstOrderTerm

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_StartupFirstOrderTermBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox
