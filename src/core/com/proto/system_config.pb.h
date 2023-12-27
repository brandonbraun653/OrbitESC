/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_SYSTEM_CONFIG_PB_H_INCLUDED
#define PB_SYSTEM_CONFIG_PB_H_INCLUDED
#include <pb.h>
#include "serial_interface.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _ParamType {
    ParamType_UNKNOWN = 0,
    ParamType_BOOL = 1,
    ParamType_UINT8 = 2,
    ParamType_UINT16 = 3,
    ParamType_UINT32 = 4,
    ParamType_FLOAT = 5,
    ParamType_DOUBLE = 6,
    ParamType_BYTES = 7,
    ParamType_STRING = 8
} ParamType;

typedef enum _ParamId {
    /* Housekeeping parameters */
    ParamId_PARAM_INVALID = -1, /* Invalid parameter */
    /* Read Only Parameters */
    ParamId_PARAM_BOOT_COUNT = 0, /* Number of times the software has booted */
    ParamId_PARAM_HW_VERSION = 1, /* Hardware version of the PCB */
    ParamId_PARAM_SW_VERSION = 2, /* Software version of the firmware */
    ParamId_PARAM_DEVICE_ID = 3, /* Factory programmed unique device ID */
    ParamId_PARAM_BOARD_NAME = 4, /* Name of the board */
    ParamId_PARAM_DESCRIPTION = 5, /* Description of the project */
    /* Read/Write Parameters */
    ParamId_PARAM_SERIAL_NUMBER = 10, /* Serial number of the device */
    ParamId_PARAM_DISK_UPDATE_RATE_MS = 11, /* How often to write parameters to disk */
    ParamId_PARAM_ACTIVITY_LED_SCALER = 12, /* Scale the activity LED blink rate */
    ParamId_PARAM_BOOT_MODE = 13, /* Boot mode of the device */
    ParamId_PARAM_CAN_NODE_ID = 14, /* CAN node ID of the device */
    /* Motor Control Parameters */
    ParamId_PARAM_STATOR_PWM_FREQ = 20, /* PWM frequency of the motor drive in Hz */
    ParamId_PARAM_SPEED_CTRL_FREQ = 21, /* Speed controller core update frequency in Hz */
    ParamId_PARAM_TARGET_IDLE_RPM = 22, /* Target RPM when the motor is idle */
    ParamId_PARAM_SPEED_CTRL_KP = 23, /* Speed controller proportional gain */
    ParamId_PARAM_SPEED_CTRL_KI = 24, /* Speed controller integral gain */
    ParamId_PARAM_SPEED_CTRL_KD = 25, /* Speed controller derivative gain */
    ParamId_PARAM_CURRENT_CTRL_Q_AXIS_KP = 26, /* Current controller Q-axis proportional gain */
    ParamId_PARAM_CURRENT_CTRL_Q_AXIS_KI = 27, /* Current controller Q-axis integral gain */
    ParamId_PARAM_CURRENT_CTRL_Q_AXIS_KD = 28, /* Current controller Q-axis derivative gain */
    ParamId_PARAM_CURRENT_CTRL_D_AXIS_KP = 29, /* Current controller D-axis proportional gain */
    ParamId_PARAM_CURRENT_CTRL_D_AXIS_KI = 30, /* Current controller D-axis integral gain */
    ParamId_PARAM_CURRENT_CTRL_D_AXIS_KD = 31, /* Current controller D-axis derivative gain */
    /* PARAM_CURRENT_CTRL_Q_FIR_COEFFS = 32; // Current controller Q-axis FIR filter coefficients
 PARAM_CURRENT_CTRL_D_FIR_COEFFS = 33; // Current controller D-axis FIR filter coefficients */
    ParamId_PARAM_RAMP_CTRL_FIRST_ORDER_TERM = 34, /* First order term of the ramp controller */
    ParamId_PARAM_RAMP_CTRL_SECOND_ORDER_TERM = 35, /* Second order term of the ramp controller */
    ParamId_PARAM_RAMP_CTRL_RAMP_TIME_SEC = 36, /* Ramp time of the ramp controller in seconds */
    ParamId_PARAM_CURRENT_OBSERVER_KSLIDE = 37, /* Current observer sliding mode controller K gain value */
    ParamId_PARAM_CURRENT_OBSERVER_MAX_ERROR = 38, /* Current observer maximum error value */
    /* Motor Description */
    ParamId_PARAM_ROTOR_POLES = 50, /* Number of poles in the motor */
    ParamId_PARAM_STATOR_SLOTS = 51, /* Number of slots in the motor */
    ParamId_PARAM_STATOR_RESISTANCE = 52, /* Stator resistance in Ohms */
    ParamId_PARAM_STATOR_INDUCTANCE = 53, /* Stator inductance in Henrys */
    /* Monitor Thresholds */
    ParamId_PARAM_PEAK_CURRENT_THRESHOLD = 60, /* Peak current threshold in Amps */
    ParamId_PARAM_PEAK_VOLTAGE_THRESHOLD = 61, /* Peak voltage threshold in Volts */
    /* Engagement Thresholds */
    ParamId_PARAM_MIN_ARM_VOLTAGE = 70, /* Minimum voltage to allow ARM state transition */
    ParamId_PARAM_MAX_ARM_VOLTAGE = 71 /* Maximum voltage to allow ARM state transition */
} ParamId;

typedef enum _ParamIOSubId {
    ParamIOSubId_GET = 1, /* Retrieve the current value of a parameter */
    ParamIOSubId_SET = 2, /* Commit a new value of a parameter */
    ParamIOSubId_SYNC = 3, /* Synchronize all parameters to disk */
    ParamIOSubId_LOAD = 4 /* Load all parameters from disk */
} ParamIOSubId;

/* Struct definitions */
typedef PB_BYTES_ARRAY_T(64) ParamIOMessage_data_t;
typedef struct _ParamIOMessage {
    Header header;
    bool has_id;
    ParamId id;
    bool has_type;
    ParamType type;
    bool has_data;
    ParamIOMessage_data_t data;
} ParamIOMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _ParamType_MIN ParamType_UNKNOWN
#define _ParamType_MAX ParamType_STRING
#define _ParamType_ARRAYSIZE ((ParamType)(ParamType_STRING+1))

#define _ParamId_MIN ParamId_PARAM_INVALID
#define _ParamId_MAX ParamId_PARAM_MAX_ARM_VOLTAGE
#define _ParamId_ARRAYSIZE ((ParamId)(ParamId_PARAM_MAX_ARM_VOLTAGE+1))

#define _ParamIOSubId_MIN ParamIOSubId_GET
#define _ParamIOSubId_MAX ParamIOSubId_LOAD
#define _ParamIOSubId_ARRAYSIZE ((ParamIOSubId)(ParamIOSubId_LOAD+1))

#define ParamIOMessage_id_ENUMTYPE ParamId
#define ParamIOMessage_type_ENUMTYPE ParamType


/* Initializer values for message structs */
#define ParamIOMessage_init_default              {Header_init_default, false, _ParamId_MIN, false, _ParamType_MIN, false, {0, {0}}}
#define ParamIOMessage_init_zero                 {Header_init_zero, false, _ParamId_MIN, false, _ParamType_MIN, false, {0, {0}}}

/* Field tags (for use in manual encoding/decoding) */
#define ParamIOMessage_header_tag                1
#define ParamIOMessage_id_tag                    2
#define ParamIOMessage_type_tag                  3
#define ParamIOMessage_data_tag                  4

/* Struct field encoding specification for nanopb */
#define ParamIOMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   OPTIONAL, ENUM,     id,                2) \
X(a, STATIC,   OPTIONAL, UENUM,    type,              3) \
X(a, STATIC,   OPTIONAL, BYTES,    data,              4)
#define ParamIOMessage_CALLBACK NULL
#define ParamIOMessage_DEFAULT (const pb_byte_t*)"\x10\xff\xff\xff\xff\xff\xff\xff\xff\xff\x01\x00"
#define ParamIOMessage_header_MSGTYPE Header

extern const pb_msgdesc_t ParamIOMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define ParamIOMessage_fields &ParamIOMessage_msg

/* Maximum encoded size of messages (where known) */
#define ParamIOMessage_size                      91

#ifdef __cplusplus
} /* extern "C" */
#endif

#ifdef __cplusplus
/* Message descriptors for nanopb */
namespace nanopb {
template <>
struct MessageDescriptor<ParamIOMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 4;
    static inline const pb_msgdesc_t* fields() {
        return &ParamIOMessage_msg;
    }
};
}  // namespace nanopb

#endif  /* __cplusplus */


#endif
