/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_SYSTEM_DATA_PB_H_INCLUDED
#define PB_SYSTEM_DATA_PB_H_INCLUDED
#include <pb.h>
#include "serial_interface.pb.h"
#include "motor_control.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _SystemDataId {
    SystemDataId_SYS_DATA_INVALID = 0, /* Invalid data ID */
    SystemDataId_ADC_PHASE_CURRENTS = 1, /* ADC readings of the phase currents */
    SystemDataId_ADC_PHASE_VOLTAGES = 2, /* Voltage commands being sent to the motor */
    SystemDataId_ADC_SYSTEM_VOLTAGES = 3 /* Measurements of less-critical system voltages */
} SystemDataId;

/* Struct definitions */
/* Message type for announcing the current system tick */
typedef struct _SystemTickMessage {
    Header header;
    uint32_t tick; /* System time in milliseconds */
} SystemTickMessage;

typedef PB_BYTES_ARRAY_T(128) ConsoleMessage_data_t;
/* Message type for streaming out console messages in real time */
typedef struct _ConsoleMessage {
    Header header;
    uint8_t this_frame; /* Which frame is this? */
    uint8_t total_frames; /* How many frames are there? */
    ConsoleMessage_data_t data; /* Data payload */
} ConsoleMessage;

/* Message type for announcing some device descriptions */
typedef struct _SystemInfoMessage {
    Header header;
    uint32_t systemTick; /* System time in milliseconds */
    char swVersion[16]; /* Software version */
    char description[16]; /* Device description */
    char serialNumber[16]; /* Serial number */
} SystemInfoMessage;

/* Message type for announcing the current system status */
typedef struct _SystemStatusMessage {
    Header header;
    uint32_t systemTick; /* System time in milliseconds */
    MotorCtrlState motorCtrlState; /* High level current motor control state */
} SystemStatusMessage;

typedef PB_BYTES_ARRAY_T(32) SystemDataMessage_payload_t;
/* Message type for streaming out raw data from the system in real time */
typedef struct _SystemDataMessage {
    Header header;
    SystemDataId id; /* Which data stream is this? */
    uint32_t timestamp; /* System time of measurement in microseconds */
    bool has_payload;
    SystemDataMessage_payload_t payload; /* Data payload */
} SystemDataMessage;

/* Data payload type for SystemDataId::ADC_PHASE_CURRENTS */
typedef struct _ADCPhaseCurrentsPayload {
    float ia; /* Phase A current in Amps */
    float ib; /* Phase B current in Amps */
    float ic; /* Phase C current in Amps */
} ADCPhaseCurrentsPayload;

/* Data payload type for SystemDataId::PWM_COMMANDS */
typedef struct _ADCPhaseVoltagesPayload {
    float va; /* Phase A voltage command in Volts */
    float vb; /* Phase B voltage command in Volts */
    float vc; /* Phase C voltage command in Volts */
} ADCPhaseVoltagesPayload;

/* Data payload type for SystemDataId::ADC_SYSTEM_VOLTAGES */
typedef struct _ADCSystemVoltagesPayload {
    float v_mcu; /* Logic level supply voltage in Volts */
    float v_dc_link; /* DC link voltage in Volts */
    float v_temp; /* Temperature sensor voltage in Volts */
    float v_isense; /* Current sense amplifier voltage reference in Volts */
} ADCSystemVoltagesPayload;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _SystemDataId_MIN SystemDataId_SYS_DATA_INVALID
#define _SystemDataId_MAX SystemDataId_ADC_SYSTEM_VOLTAGES
#define _SystemDataId_ARRAYSIZE ((SystemDataId)(SystemDataId_ADC_SYSTEM_VOLTAGES+1))




#define SystemStatusMessage_motorCtrlState_ENUMTYPE MotorCtrlState

#define SystemDataMessage_id_ENUMTYPE SystemDataId





/* Initializer values for message structs */
#define SystemTickMessage_init_default           {Header_init_default, 0}
#define ConsoleMessage_init_default              {Header_init_default, 0, 0, {0, {0}}}
#define SystemInfoMessage_init_default           {Header_init_default, 0, "", "", ""}
#define SystemStatusMessage_init_default         {Header_init_default, 0, _MotorCtrlState_MIN}
#define SystemDataMessage_init_default           {Header_init_default, _SystemDataId_MIN, 0, false, {0, {0}}}
#define ADCPhaseCurrentsPayload_init_default     {0, 0, 0}
#define ADCPhaseVoltagesPayload_init_default     {0, 0, 0}
#define ADCSystemVoltagesPayload_init_default    {0, 0, 0, 0}
#define SystemTickMessage_init_zero              {Header_init_zero, 0}
#define ConsoleMessage_init_zero                 {Header_init_zero, 0, 0, {0, {0}}}
#define SystemInfoMessage_init_zero              {Header_init_zero, 0, "", "", ""}
#define SystemStatusMessage_init_zero            {Header_init_zero, 0, _MotorCtrlState_MIN}
#define SystemDataMessage_init_zero              {Header_init_zero, _SystemDataId_MIN, 0, false, {0, {0}}}
#define ADCPhaseCurrentsPayload_init_zero        {0, 0, 0}
#define ADCPhaseVoltagesPayload_init_zero        {0, 0, 0}
#define ADCSystemVoltagesPayload_init_zero       {0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define SystemTickMessage_header_tag             1
#define SystemTickMessage_tick_tag               2
#define ConsoleMessage_header_tag                1
#define ConsoleMessage_this_frame_tag            2
#define ConsoleMessage_total_frames_tag          3
#define ConsoleMessage_data_tag                  4
#define SystemInfoMessage_header_tag             1
#define SystemInfoMessage_systemTick_tag         2
#define SystemInfoMessage_swVersion_tag          3
#define SystemInfoMessage_description_tag        4
#define SystemInfoMessage_serialNumber_tag       5
#define SystemStatusMessage_header_tag           1
#define SystemStatusMessage_systemTick_tag       2
#define SystemStatusMessage_motorCtrlState_tag   3
#define SystemDataMessage_header_tag             1
#define SystemDataMessage_id_tag                 2
#define SystemDataMessage_timestamp_tag          3
#define SystemDataMessage_payload_tag            4
#define ADCPhaseCurrentsPayload_ia_tag           1
#define ADCPhaseCurrentsPayload_ib_tag           2
#define ADCPhaseCurrentsPayload_ic_tag           3
#define ADCPhaseVoltagesPayload_va_tag           1
#define ADCPhaseVoltagesPayload_vb_tag           2
#define ADCPhaseVoltagesPayload_vc_tag           3
#define ADCSystemVoltagesPayload_v_mcu_tag       1
#define ADCSystemVoltagesPayload_v_dc_link_tag   2
#define ADCSystemVoltagesPayload_v_temp_tag      3
#define ADCSystemVoltagesPayload_v_isense_tag    4

/* Struct field encoding specification for nanopb */
#define SystemTickMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   tick,              2)
#define SystemTickMessage_CALLBACK NULL
#define SystemTickMessage_DEFAULT NULL
#define SystemTickMessage_header_MSGTYPE Header

#define ConsoleMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   this_frame,        2) \
X(a, STATIC,   REQUIRED, UINT32,   total_frames,      3) \
X(a, STATIC,   REQUIRED, BYTES,    data,              4)
#define ConsoleMessage_CALLBACK NULL
#define ConsoleMessage_DEFAULT NULL
#define ConsoleMessage_header_MSGTYPE Header

#define SystemInfoMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   systemTick,        2) \
X(a, STATIC,   REQUIRED, STRING,   swVersion,         3) \
X(a, STATIC,   REQUIRED, STRING,   description,       4) \
X(a, STATIC,   REQUIRED, STRING,   serialNumber,      5)
#define SystemInfoMessage_CALLBACK NULL
#define SystemInfoMessage_DEFAULT NULL
#define SystemInfoMessage_header_MSGTYPE Header

#define SystemStatusMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   systemTick,        2) \
X(a, STATIC,   REQUIRED, UENUM,    motorCtrlState,    3)
#define SystemStatusMessage_CALLBACK NULL
#define SystemStatusMessage_DEFAULT NULL
#define SystemStatusMessage_header_MSGTYPE Header

#define SystemDataMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UENUM,    id,                2) \
X(a, STATIC,   REQUIRED, UINT32,   timestamp,         3) \
X(a, STATIC,   OPTIONAL, BYTES,    payload,           4)
#define SystemDataMessage_CALLBACK NULL
#define SystemDataMessage_DEFAULT NULL
#define SystemDataMessage_header_MSGTYPE Header

#define ADCPhaseCurrentsPayload_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, FLOAT,    ia,                1) \
X(a, STATIC,   REQUIRED, FLOAT,    ib,                2) \
X(a, STATIC,   REQUIRED, FLOAT,    ic,                3)
#define ADCPhaseCurrentsPayload_CALLBACK NULL
#define ADCPhaseCurrentsPayload_DEFAULT NULL

#define ADCPhaseVoltagesPayload_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, FLOAT,    va,                1) \
X(a, STATIC,   REQUIRED, FLOAT,    vb,                2) \
X(a, STATIC,   REQUIRED, FLOAT,    vc,                3)
#define ADCPhaseVoltagesPayload_CALLBACK NULL
#define ADCPhaseVoltagesPayload_DEFAULT NULL

#define ADCSystemVoltagesPayload_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, FLOAT,    v_mcu,             1) \
X(a, STATIC,   REQUIRED, FLOAT,    v_dc_link,         2) \
X(a, STATIC,   REQUIRED, FLOAT,    v_temp,            3) \
X(a, STATIC,   REQUIRED, FLOAT,    v_isense,          4)
#define ADCSystemVoltagesPayload_CALLBACK NULL
#define ADCSystemVoltagesPayload_DEFAULT NULL

extern const pb_msgdesc_t SystemTickMessage_msg;
extern const pb_msgdesc_t ConsoleMessage_msg;
extern const pb_msgdesc_t SystemInfoMessage_msg;
extern const pb_msgdesc_t SystemStatusMessage_msg;
extern const pb_msgdesc_t SystemDataMessage_msg;
extern const pb_msgdesc_t ADCPhaseCurrentsPayload_msg;
extern const pb_msgdesc_t ADCPhaseVoltagesPayload_msg;
extern const pb_msgdesc_t ADCSystemVoltagesPayload_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define SystemTickMessage_fields &SystemTickMessage_msg
#define ConsoleMessage_fields &ConsoleMessage_msg
#define SystemInfoMessage_fields &SystemInfoMessage_msg
#define SystemStatusMessage_fields &SystemStatusMessage_msg
#define SystemDataMessage_fields &SystemDataMessage_msg
#define ADCPhaseCurrentsPayload_fields &ADCPhaseCurrentsPayload_msg
#define ADCPhaseVoltagesPayload_fields &ADCPhaseVoltagesPayload_msg
#define ADCSystemVoltagesPayload_fields &ADCSystemVoltagesPayload_msg

/* Maximum encoded size of messages (where known) */
#define ADCPhaseCurrentsPayload_size             15
#define ADCPhaseVoltagesPayload_size             15
#define ADCSystemVoltagesPayload_size            20
#define ConsoleMessage_size                      149
#define SystemDataMessage_size                   54
#define SystemInfoMessage_size                   69
#define SystemStatusMessage_size                 20
#define SystemTickMessage_size                   18

#ifdef __cplusplus
} /* extern "C" */
#endif

#ifdef __cplusplus
/* Message descriptors for nanopb */
namespace nanopb {
template <>
struct MessageDescriptor<SystemTickMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 2;
    static inline const pb_msgdesc_t* fields() {
        return &SystemTickMessage_msg;
    }
};
template <>
struct MessageDescriptor<ConsoleMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 4;
    static inline const pb_msgdesc_t* fields() {
        return &ConsoleMessage_msg;
    }
};
template <>
struct MessageDescriptor<SystemInfoMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 5;
    static inline const pb_msgdesc_t* fields() {
        return &SystemInfoMessage_msg;
    }
};
template <>
struct MessageDescriptor<SystemStatusMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 3;
    static inline const pb_msgdesc_t* fields() {
        return &SystemStatusMessage_msg;
    }
};
template <>
struct MessageDescriptor<SystemDataMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 4;
    static inline const pb_msgdesc_t* fields() {
        return &SystemDataMessage_msg;
    }
};
template <>
struct MessageDescriptor<ADCPhaseCurrentsPayload> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 3;
    static inline const pb_msgdesc_t* fields() {
        return &ADCPhaseCurrentsPayload_msg;
    }
};
template <>
struct MessageDescriptor<ADCPhaseVoltagesPayload> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 3;
    static inline const pb_msgdesc_t* fields() {
        return &ADCPhaseVoltagesPayload_msg;
    }
};
template <>
struct MessageDescriptor<ADCSystemVoltagesPayload> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 4;
    static inline const pb_msgdesc_t* fields() {
        return &ADCSystemVoltagesPayload_msg;
    }
};
}  // namespace nanopb

#endif  /* __cplusplus */


#endif