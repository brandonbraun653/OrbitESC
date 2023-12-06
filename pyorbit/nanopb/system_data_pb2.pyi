"""
@generated by mypy-protobuf.  Do not edit manually!
isort:skip_file
"""
import builtins
import google.protobuf.descriptor
import google.protobuf.internal.enum_type_wrapper
import google.protobuf.message
import motor_control_pb2
import serial_interface_pb2
import typing
import typing_extensions

DESCRIPTOR: google.protobuf.descriptor.FileDescriptor = ...

class _SystemDataId:
    ValueType = typing.NewType('ValueType', builtins.int)
    V: typing_extensions.TypeAlias = ValueType
class _SystemDataIdEnumTypeWrapper(google.protobuf.internal.enum_type_wrapper._EnumTypeWrapper[_SystemDataId.ValueType], builtins.type):
    DESCRIPTOR: google.protobuf.descriptor.EnumDescriptor = ...
    SYS_DATA_INVALID: SystemDataId.ValueType = ...  # 0
    """Invalid data ID"""

    ADC_PHASE_CURRENTS: SystemDataId.ValueType = ...  # 1
    """ADC readings of the phase currents"""

    ADC_PHASE_VOLTAGES: SystemDataId.ValueType = ...  # 2
    """Voltage commands being sent to the motor"""

    ADC_SYSTEM_VOLTAGES: SystemDataId.ValueType = ...  # 3
    """Measurements of less-critical system voltages"""

class SystemDataId(_SystemDataId, metaclass=_SystemDataIdEnumTypeWrapper):
    pass

SYS_DATA_INVALID: SystemDataId.ValueType = ...  # 0
"""Invalid data ID"""

ADC_PHASE_CURRENTS: SystemDataId.ValueType = ...  # 1
"""ADC readings of the phase currents"""

ADC_PHASE_VOLTAGES: SystemDataId.ValueType = ...  # 2
"""Voltage commands being sent to the motor"""

ADC_SYSTEM_VOLTAGES: SystemDataId.ValueType = ...  # 3
"""Measurements of less-critical system voltages"""

global___SystemDataId = SystemDataId


class SystemTickMessage(google.protobuf.message.Message):
    """Message type for announcing the current system tick"""
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    HEADER_FIELD_NUMBER: builtins.int
    TICK_FIELD_NUMBER: builtins.int
    @property
    def header(self) -> serial_interface_pb2.Header: ...
    tick: builtins.int = ...
    """System time in milliseconds"""

    def __init__(self,
        *,
        header : typing.Optional[serial_interface_pb2.Header] = ...,
        tick : typing.Optional[builtins.int] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["header",b"header","tick",b"tick"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["header",b"header","tick",b"tick"]) -> None: ...
global___SystemTickMessage = SystemTickMessage

class ConsoleMessage(google.protobuf.message.Message):
    """Message type for streaming out console messages in real time"""
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    HEADER_FIELD_NUMBER: builtins.int
    THIS_FRAME_FIELD_NUMBER: builtins.int
    TOTAL_FRAMES_FIELD_NUMBER: builtins.int
    DATA_FIELD_NUMBER: builtins.int
    @property
    def header(self) -> serial_interface_pb2.Header: ...
    this_frame: builtins.int = ...
    """Which frame is this?"""

    total_frames: builtins.int = ...
    """How many frames are there?"""

    data: builtins.bytes = ...
    """Data payload"""

    def __init__(self,
        *,
        header : typing.Optional[serial_interface_pb2.Header] = ...,
        this_frame : typing.Optional[builtins.int] = ...,
        total_frames : typing.Optional[builtins.int] = ...,
        data : typing.Optional[builtins.bytes] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["data",b"data","header",b"header","this_frame",b"this_frame","total_frames",b"total_frames"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["data",b"data","header",b"header","this_frame",b"this_frame","total_frames",b"total_frames"]) -> None: ...
global___ConsoleMessage = ConsoleMessage

class SystemInfoMessage(google.protobuf.message.Message):
    """Message type for announcing some device descriptions"""
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    HEADER_FIELD_NUMBER: builtins.int
    SYSTEMTICK_FIELD_NUMBER: builtins.int
    SWVERSION_FIELD_NUMBER: builtins.int
    DESCRIPTION_FIELD_NUMBER: builtins.int
    SERIALNUMBER_FIELD_NUMBER: builtins.int
    @property
    def header(self) -> serial_interface_pb2.Header: ...
    systemTick: builtins.int = ...
    """System time in milliseconds"""

    swVersion: typing.Text = ...
    """Software version"""

    description: typing.Text = ...
    """Device description"""

    serialNumber: typing.Text = ...
    """Serial number"""

    def __init__(self,
        *,
        header : typing.Optional[serial_interface_pb2.Header] = ...,
        systemTick : typing.Optional[builtins.int] = ...,
        swVersion : typing.Optional[typing.Text] = ...,
        description : typing.Optional[typing.Text] = ...,
        serialNumber : typing.Optional[typing.Text] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["description",b"description","header",b"header","serialNumber",b"serialNumber","swVersion",b"swVersion","systemTick",b"systemTick"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["description",b"description","header",b"header","serialNumber",b"serialNumber","swVersion",b"swVersion","systemTick",b"systemTick"]) -> None: ...
global___SystemInfoMessage = SystemInfoMessage

class SystemStatusMessage(google.protobuf.message.Message):
    """Message type for announcing the current system status"""
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    HEADER_FIELD_NUMBER: builtins.int
    SYSTEMTICK_FIELD_NUMBER: builtins.int
    MOTORCTRLSTATE_FIELD_NUMBER: builtins.int
    @property
    def header(self) -> serial_interface_pb2.Header: ...
    systemTick: builtins.int = ...
    """System time in milliseconds"""

    motorCtrlState: motor_control_pb2.MotorCtrlState.ValueType = ...
    """High level current motor control state"""

    def __init__(self,
        *,
        header : typing.Optional[serial_interface_pb2.Header] = ...,
        systemTick : typing.Optional[builtins.int] = ...,
        motorCtrlState : typing.Optional[motor_control_pb2.MotorCtrlState.ValueType] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["header",b"header","motorCtrlState",b"motorCtrlState","systemTick",b"systemTick"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["header",b"header","motorCtrlState",b"motorCtrlState","systemTick",b"systemTick"]) -> None: ...
global___SystemStatusMessage = SystemStatusMessage

class SystemDataMessage(google.protobuf.message.Message):
    """Message type for streaming out raw data from the system in real time"""
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    HEADER_FIELD_NUMBER: builtins.int
    ID_FIELD_NUMBER: builtins.int
    TIMESTAMP_FIELD_NUMBER: builtins.int
    PAYLOAD_FIELD_NUMBER: builtins.int
    @property
    def header(self) -> serial_interface_pb2.Header: ...
    id: global___SystemDataId.ValueType = ...
    """Which data stream is this?"""

    timestamp: builtins.int = ...
    """System time of measurement in microseconds"""

    payload: builtins.bytes = ...
    """Data payload"""

    def __init__(self,
        *,
        header : typing.Optional[serial_interface_pb2.Header] = ...,
        id : typing.Optional[global___SystemDataId.ValueType] = ...,
        timestamp : typing.Optional[builtins.int] = ...,
        payload : typing.Optional[builtins.bytes] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["header",b"header","id",b"id","payload",b"payload","timestamp",b"timestamp"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["header",b"header","id",b"id","payload",b"payload","timestamp",b"timestamp"]) -> None: ...
global___SystemDataMessage = SystemDataMessage

class ADCPhaseCurrentsPayload(google.protobuf.message.Message):
    """Data payload type for SystemDataId::ADC_PHASE_CURRENTS"""
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    IA_FIELD_NUMBER: builtins.int
    IB_FIELD_NUMBER: builtins.int
    IC_FIELD_NUMBER: builtins.int
    ia: builtins.float = ...
    """Phase A current in Amps"""

    ib: builtins.float = ...
    """Phase B current in Amps"""

    ic: builtins.float = ...
    """Phase C current in Amps"""

    def __init__(self,
        *,
        ia : typing.Optional[builtins.float] = ...,
        ib : typing.Optional[builtins.float] = ...,
        ic : typing.Optional[builtins.float] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["ia",b"ia","ib",b"ib","ic",b"ic"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["ia",b"ia","ib",b"ib","ic",b"ic"]) -> None: ...
global___ADCPhaseCurrentsPayload = ADCPhaseCurrentsPayload

class ADCPhaseVoltagesPayload(google.protobuf.message.Message):
    """Data payload type for SystemDataId::PWM_COMMANDS"""
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    VA_FIELD_NUMBER: builtins.int
    VB_FIELD_NUMBER: builtins.int
    VC_FIELD_NUMBER: builtins.int
    va: builtins.float = ...
    """Phase A voltage command in Volts"""

    vb: builtins.float = ...
    """Phase B voltage command in Volts"""

    vc: builtins.float = ...
    """Phase C voltage command in Volts"""

    def __init__(self,
        *,
        va : typing.Optional[builtins.float] = ...,
        vb : typing.Optional[builtins.float] = ...,
        vc : typing.Optional[builtins.float] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["va",b"va","vb",b"vb","vc",b"vc"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["va",b"va","vb",b"vb","vc",b"vc"]) -> None: ...
global___ADCPhaseVoltagesPayload = ADCPhaseVoltagesPayload

class ADCSystemVoltagesPayload(google.protobuf.message.Message):
    """Data payload type for SystemDataId::ADC_SYSTEM_VOLTAGES"""
    DESCRIPTOR: google.protobuf.descriptor.Descriptor = ...
    V_MCU_FIELD_NUMBER: builtins.int
    V_DC_LINK_FIELD_NUMBER: builtins.int
    V_TEMP_FIELD_NUMBER: builtins.int
    V_ISENSE_FIELD_NUMBER: builtins.int
    v_mcu: builtins.float = ...
    """Logic level supply voltage in Volts"""

    v_dc_link: builtins.float = ...
    """DC link voltage in Volts"""

    v_temp: builtins.float = ...
    """Temperature sensor voltage in Volts"""

    v_isense: builtins.float = ...
    """Current sense amplifier voltage reference in Volts"""

    def __init__(self,
        *,
        v_mcu : typing.Optional[builtins.float] = ...,
        v_dc_link : typing.Optional[builtins.float] = ...,
        v_temp : typing.Optional[builtins.float] = ...,
        v_isense : typing.Optional[builtins.float] = ...,
        ) -> None: ...
    def HasField(self, field_name: typing_extensions.Literal["v_dc_link",b"v_dc_link","v_isense",b"v_isense","v_mcu",b"v_mcu","v_temp",b"v_temp"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing_extensions.Literal["v_dc_link",b"v_dc_link","v_isense",b"v_isense","v_mcu",b"v_mcu","v_temp",b"v_temp"]) -> None: ...
global___ADCSystemVoltagesPayload = ADCSystemVoltagesPayload
