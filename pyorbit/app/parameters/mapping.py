from PyQt5 import QtWidgets
from pyorbit.app.parameters.abstract import AbstractParameter
from pyorbit.serial.parameters import ParameterId, ParameterEncoding


class QAxisKp(AbstractParameter):
    """ Class for representing the Q-axis Kp parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.CurrentControlQKp

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_QAxisKpBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class QAxisKi(AbstractParameter):
    """ Class for representing the Q-axis Ki parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.CurrentControlQKi

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_QAxisKiBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class DAxisKp(AbstractParameter):
    """ Class for representing the D-axis Kp parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.CurrentControlDKp

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_DAxisKpBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class DAxisKi(AbstractParameter):
    """ Class for representing the D-axis Ki parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.CurrentControlDKi

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_DAxisKiBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class SpeedControlKp(AbstractParameter):
    """ Class for representing the speed control Kp parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.SpeedControlKp

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_SpeedCtrlKpBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class SpeedControlKi(AbstractParameter):
    """ Class for representing the speed control Ki parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.SpeedControlKi

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_SpeedCtrlKiBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class SpeedControlKd(AbstractParameter):
    """ Class for representing the speed control Kd parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.SpeedControlKd

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_SpeedCtrlKdBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class RampFirstOrderTerm(AbstractParameter):
    """ Class for representing the first order term parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.RampControlFirstOrderTerm

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_StartupFirstOrderTermBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class RampSecondOrderTerm(AbstractParameter):
    """ Class for representing the second order term parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.RampControlSecondOrderTerm

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_StartupSecondOrderTermBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class RampTimeConstant(AbstractParameter):
    """ Class for representing the time constant parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.RampControlRampTime

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_StartupRampTimeBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class TargetIdleRPM(AbstractParameter):
    """ Class for representing the target idle RPM parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.TargetIdleRPM

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_TargetIdleRPMBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class CurrentControlFrequency(AbstractParameter):
    """ Class for representing the current control frequency parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.StatorPWMFrequency

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_CurrentCtrlFreqBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class SpeedControlFrequency(AbstractParameter):
    """ Class for representing the speed control frequency parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.SpeedControlFrequency

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_SpeedCtrlFreqBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class PeakCurrentLimit(AbstractParameter):
    """ Class for representing the peak current limit parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.PeakCurrentThreshold

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_PeakCurrentLimitBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class PeakVoltageLimit(AbstractParameter):
    """ Class for representing the peak voltage limit parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.PeakVoltageThreshold

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_SupplyVoltageLimitBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class BoardName(AbstractParameter):
    """ Class for representing the board name parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.STRING

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.BoardName

    @property
    def object_name(self) -> str:
        return "infoTab_BoardNameBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QLineEdit

    @property
    def read_only(self) -> bool:
        return True


class Description(AbstractParameter):
    """ Class for representing the description parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.STRING

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.Description

    @property
    def object_name(self) -> str:
        return "infoTab_DescriptionBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QLineEdit

    @property
    def read_only(self) -> bool:
        return True


class HWVersion(AbstractParameter):
    """ Class for representing the hardware version parameter """

    def __init__(self):
        super().__init__()
        
    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.UINT8

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.HwVersion

    @property
    def object_name(self) -> str:
        return "infoTab_HWVersionBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QLineEdit

    @property
    def read_only(self) -> bool:
        return True


class SWVersion(AbstractParameter):
    """ Class for representing the software version parameter """

    def __init__(self):
        super().__init__()
        
    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.STRING

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.SwVersion

    @property
    def object_name(self) -> str:
        return "infoTab_SWVersionBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QLineEdit

    @property
    def read_only(self) -> bool:
        return True


class DeviceID(AbstractParameter):
    """ Class for representing the device ID parameter """

    def __init__(self):
        super().__init__()
        
    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.UINT32

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.DeviceId

    @property
    def object_name(self) -> str:
        return "infoTab_DeviceIDBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QLineEdit

    @property
    def read_only(self) -> bool:
        return True


class SerialNumber(AbstractParameter):
    """ Class for representing the serial number parameter """

    def __init__(self):
        super().__init__()
        
    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.STRING

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.SerialNumber

    @property
    def object_name(self) -> str:
        return "infoTab_SerialNumberBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QLineEdit


class HeartbeatLEDRate(AbstractParameter):
    """ Class for representing the heartbeat LED rate parameter """

    def __init__(self):
        super().__init__()
        
    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.ActivityLedScaler

    @property
    def object_name(self) -> str:
        return "systemTab_HeartbeatLEDRateBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class DiskSyncRate(AbstractParameter):
    """ Class for representing the disk sync rate parameter """

    def __init__(self):
        super().__init__()
        
    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.DiskUpdateRateMS

    @property
    def object_name(self) -> str:
        return "systemTab_DiskSyncRateBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


# class PhaseCurrentStreaming(AbstractParameter):
#     """ Class for representing the phase current streaming parameter """
#
#     def __init__(self):
#         super().__init__()
#
#     @property
#     def value_encoding(self) -> ParameterEncoding:
#         return ParameterEncoding.BOOL
#
#     @property
#     def parameter_id(self) -> ParameterId:
#         return ParameterId.StreamPhaseCurrents
#
#     @property
#     def object_name(self) -> str:
#         return "systemTab_PhaseCurrentStreamingCheckBox"
#
#     @property
#     def widget_type(self) -> type:
#         return QtWidgets.QCheckBox


class RotorPoles(AbstractParameter):
    """ Class for representing the rotor poles parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.UINT8

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.RotorPoles

    @property
    def object_name(self) -> str:
        return "motorAttrTab_RotorPolesBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QSpinBox


class StatorSlots(AbstractParameter):
    """ Class for representing the stator slots parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.UINT8

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.StatorSlots

    @property
    def object_name(self) -> str:
        return "motorAttrTab_StatorSlotsBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QSpinBox


class PhaseResistance(AbstractParameter):
    """ Class for representing the phase resistance parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.StatorResistance

    @property
    def object_name(self) -> str:
        return "motorAttrTab_PhaseResistanceBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class PhaseInductance(AbstractParameter):
    """ Class for representing the phase inductance parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.StatorInductance

    @property
    def object_name(self) -> str:
        return "motorAttrTab_PhaseInductanceBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class BootCount(AbstractParameter):
    """ Class for representing the boot count parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.UINT32

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.BootCount

    @property
    def object_name(self) -> str:
        return "globalBootCountBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QLineEdit

    @property
    def read_only(self) -> bool:
        return True


class SlidingModeControlGain(AbstractParameter):
    """ Class for representing the sliding mode control gain parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.SlidingModeControlGain

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_SlidingModeControlGainBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox


class SlidingModeControlMaximumError(AbstractParameter):
    """ Class for representing the sliding mode control maximum error parameter """

    def __init__(self):
        super().__init__()

    @property
    def value_encoding(self) -> ParameterEncoding:
        return ParameterEncoding.FLOAT

    @property
    def parameter_id(self) -> ParameterId:
        return ParameterId.SlidingModeControlMaxError

    @property
    def object_name(self) -> str:
        return "motorCtrlTab_SlidingModeControlMaximumErrorBox"

    @property
    def widget_type(self) -> type:
        return QtWidgets.QDoubleSpinBox
