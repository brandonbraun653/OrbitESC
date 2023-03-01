/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7-dev */

#ifndef PB_SRC_CORE_COM_SERIAL_SERIAL_INTERFACE_PB_H_INCLUDED
#define PB_SRC_CORE_COM_SERIAL_SERIAL_INTERFACE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _MsgId {
    MsgId_MSG_ACK_NACK = 0, /* Generic ack/nack type message */
    MsgId_MSG_PING_CMD = 1, /* Simple PING to see if the node is alive */
    MsgId_MSG_TERMINAL = 2, /* Terminal command for printing text/debug data */
    MsgId_MSG_SYS_TICK = 3, /* System time tick */
    MsgId_MSG_SYS_INFO = 4, /* System information */
    MsgId_MSG_PARAM_IO = 5, /* Do operations on configurable parameters */
    MsgId_MSG_SYS_CTRL = 6, /* Perform system control operations */
    MsgId_MSG_SWITCH_MODE = 7 /* Switch the boot mode of the device */
} MsgId;

typedef enum _SubId {
    /* Parameter IO messages */
    SubId_SUB_MSG_PARAM_IO_GET = 0, /* Retrieve the current value of a parameter */
    SubId_SUB_MSG_PARAM_IO_SET = 1, /* Commit a new value of a parameter */
    SubId_SUB_MSG_PARAM_IO_SYNC = 2, /* Synchronize all parameters to disk */
    SubId_SUB_MSG_PARAM_IO_LOAD = 3, /* Load all parameters from disk */
    /* System control messages */
    SubId_SUB_MSG_SYS_CTRL_RESET = 0 /* Reset the system */
} SubId;

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
    ParamId_PARAM_BOOT_MODE = 13 /* Boot mode of the device */
} ParamId;

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

typedef enum _StatusCode {
    StatusCode_NO_ERROR = 0,
    StatusCode_UNKNOWN_ERROR = 1,
    StatusCode_INVALID_PARAM = 2,
    StatusCode_INVALID_TYPE = 3,
    StatusCode_INVALID_VALUE = 4,
    StatusCode_REQUEST_FAILED = 5
} StatusCode;

typedef enum _BootMode {
    BootMode_BOOT_MODE_NORMAL = 0,
    BootMode_BOOT_MODE_TEST = 1,
    BootMode_BOOT_MODE_CONFIG = 2
} BootMode;

/* Struct definitions */
/* Instrumentation message header common to all types. Each functional message type **must**
 have this first in their list of declarations. */
typedef struct _Header {
    uint8_t msgId; /* Root message identifier */
    uint8_t subId; /* Possible sub-identifier to specify root ID details */
    uint16_t uuid; /* Unique ID for the message */
} Header;

/* Root type that parsers can use to peek at messages */
typedef struct _BaseMessage {
    Header header;
} BaseMessage;

/* Generic ACK or NACK to a previous message */
typedef struct _AckNackMessage {
    Header header;
    bool acknowledge;
    StatusCode status_code;
} AckNackMessage;

typedef struct _PingMessage {
    Header header;
} PingMessage;

typedef struct _SystemTick {
    Header header;
    uint32_t tick;
} SystemTick;

typedef PB_BYTES_ARRAY_T(128) ConsoleMessage_data_t;
typedef struct _ConsoleMessage {
    Header header;
    uint8_t this_frame;
    uint8_t total_frames;
    ConsoleMessage_data_t data;
} ConsoleMessage;

typedef struct _SystemInfoMessage {
    Header header;
    uint32_t systemTick;
    char swVersion[16];
    char description[16];
    char serialNumber[16];
} SystemInfoMessage;

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

typedef struct _SystemControlMessage {
    Header header;
} SystemControlMessage;

typedef struct _SwitchModeMessage {
    Header header;
    BootMode mode;
} SwitchModeMessage;


/* Helper constants for enums */
#define _MsgId_MIN MsgId_MSG_ACK_NACK
#define _MsgId_MAX MsgId_MSG_SWITCH_MODE
#define _MsgId_ARRAYSIZE ((MsgId)(MsgId_MSG_SWITCH_MODE+1))

#define _SubId_MIN SubId_SUB_MSG_PARAM_IO_GET
#define _SubId_MAX SubId_SUB_MSG_PARAM_IO_LOAD
#define _SubId_ARRAYSIZE ((SubId)(SubId_SUB_MSG_PARAM_IO_LOAD+1))

#define _ParamId_MIN ParamId_PARAM_INVALID
#define _ParamId_MAX ParamId_PARAM_BOOT_MODE
#define _ParamId_ARRAYSIZE ((ParamId)(ParamId_PARAM_BOOT_MODE+1))

#define _ParamType_MIN ParamType_UNKNOWN
#define _ParamType_MAX ParamType_STRING
#define _ParamType_ARRAYSIZE ((ParamType)(ParamType_STRING+1))

#define _StatusCode_MIN StatusCode_NO_ERROR
#define _StatusCode_MAX StatusCode_REQUEST_FAILED
#define _StatusCode_ARRAYSIZE ((StatusCode)(StatusCode_REQUEST_FAILED+1))

#define _BootMode_MIN BootMode_BOOT_MODE_NORMAL
#define _BootMode_MAX BootMode_BOOT_MODE_CONFIG
#define _BootMode_ARRAYSIZE ((BootMode)(BootMode_BOOT_MODE_CONFIG+1))



#define AckNackMessage_status_code_ENUMTYPE StatusCode





#define ParamIOMessage_id_ENUMTYPE ParamId
#define ParamIOMessage_type_ENUMTYPE ParamType


#define SwitchModeMessage_mode_ENUMTYPE BootMode


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define Header_init_default                      {0, 0, 0}
#define BaseMessage_init_default                 {Header_init_default}
#define AckNackMessage_init_default              {Header_init_default, 0, _StatusCode_MIN}
#define PingMessage_init_default                 {Header_init_default}
#define SystemTick_init_default                  {Header_init_default, 0}
#define ConsoleMessage_init_default              {Header_init_default, 0, 0, {0, {0}}}
#define SystemInfoMessage_init_default           {Header_init_default, 0, "", "", ""}
#define ParamIOMessage_init_default              {Header_init_default, false, _ParamId_MIN, false, _ParamType_MIN, false, {0, {0}}}
#define SystemControlMessage_init_default        {Header_init_default}
#define SwitchModeMessage_init_default           {Header_init_default, _BootMode_MIN}
#define Header_init_zero                         {0, 0, 0}
#define BaseMessage_init_zero                    {Header_init_zero}
#define AckNackMessage_init_zero                 {Header_init_zero, 0, _StatusCode_MIN}
#define PingMessage_init_zero                    {Header_init_zero}
#define SystemTick_init_zero                     {Header_init_zero, 0}
#define ConsoleMessage_init_zero                 {Header_init_zero, 0, 0, {0, {0}}}
#define SystemInfoMessage_init_zero              {Header_init_zero, 0, "", "", ""}
#define ParamIOMessage_init_zero                 {Header_init_zero, false, _ParamId_MIN, false, _ParamType_MIN, false, {0, {0}}}
#define SystemControlMessage_init_zero           {Header_init_zero}
#define SwitchModeMessage_init_zero              {Header_init_zero, _BootMode_MIN}

/* Field tags (for use in manual encoding/decoding) */
#define Header_msgId_tag                         1
#define Header_subId_tag                         2
#define Header_uuid_tag                          3
#define BaseMessage_header_tag                   1
#define AckNackMessage_header_tag                1
#define AckNackMessage_acknowledge_tag           2
#define AckNackMessage_status_code_tag           3
#define PingMessage_header_tag                   1
#define SystemTick_header_tag                    1
#define SystemTick_tick_tag                      2
#define ConsoleMessage_header_tag                1
#define ConsoleMessage_this_frame_tag            2
#define ConsoleMessage_total_frames_tag          3
#define ConsoleMessage_data_tag                  4
#define SystemInfoMessage_header_tag             1
#define SystemInfoMessage_systemTick_tag         2
#define SystemInfoMessage_swVersion_tag          3
#define SystemInfoMessage_description_tag        4
#define SystemInfoMessage_serialNumber_tag       5
#define ParamIOMessage_header_tag                1
#define ParamIOMessage_id_tag                    2
#define ParamIOMessage_type_tag                  3
#define ParamIOMessage_data_tag                  4
#define SystemControlMessage_header_tag          1
#define SwitchModeMessage_header_tag             1
#define SwitchModeMessage_mode_tag               2

/* Struct field encoding specification for nanopb */
#define Header_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   msgId,             1) \
X(a, STATIC,   REQUIRED, UINT32,   subId,             2) \
X(a, STATIC,   REQUIRED, UINT32,   uuid,              3)
#define Header_CALLBACK NULL
#define Header_DEFAULT NULL

#define BaseMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1)
#define BaseMessage_CALLBACK NULL
#define BaseMessage_DEFAULT NULL
#define BaseMessage_header_MSGTYPE Header

#define AckNackMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, BOOL,     acknowledge,       2) \
X(a, STATIC,   REQUIRED, UENUM,    status_code,       3)
#define AckNackMessage_CALLBACK NULL
#define AckNackMessage_DEFAULT NULL
#define AckNackMessage_header_MSGTYPE Header

#define PingMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1)
#define PingMessage_CALLBACK NULL
#define PingMessage_DEFAULT NULL
#define PingMessage_header_MSGTYPE Header

#define SystemTick_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   tick,              2)
#define SystemTick_CALLBACK NULL
#define SystemTick_DEFAULT NULL
#define SystemTick_header_MSGTYPE Header

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

#define ParamIOMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   OPTIONAL, ENUM,     id,                2) \
X(a, STATIC,   OPTIONAL, UENUM,    type,              3) \
X(a, STATIC,   OPTIONAL, BYTES,    data,              4)
#define ParamIOMessage_CALLBACK NULL
#define ParamIOMessage_DEFAULT (const pb_byte_t*)"\x10\xff\xff\xff\xff\xff\xff\xff\xff\xff\x01\x00"
#define ParamIOMessage_header_MSGTYPE Header

#define SystemControlMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1)
#define SystemControlMessage_CALLBACK NULL
#define SystemControlMessage_DEFAULT NULL
#define SystemControlMessage_header_MSGTYPE Header

#define SwitchModeMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UENUM,    mode,              2)
#define SwitchModeMessage_CALLBACK NULL
#define SwitchModeMessage_DEFAULT NULL
#define SwitchModeMessage_header_MSGTYPE Header

extern const pb_msgdesc_t Header_msg;
extern const pb_msgdesc_t BaseMessage_msg;
extern const pb_msgdesc_t AckNackMessage_msg;
extern const pb_msgdesc_t PingMessage_msg;
extern const pb_msgdesc_t SystemTick_msg;
extern const pb_msgdesc_t ConsoleMessage_msg;
extern const pb_msgdesc_t SystemInfoMessage_msg;
extern const pb_msgdesc_t ParamIOMessage_msg;
extern const pb_msgdesc_t SystemControlMessage_msg;
extern const pb_msgdesc_t SwitchModeMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Header_fields &Header_msg
#define BaseMessage_fields &BaseMessage_msg
#define AckNackMessage_fields &AckNackMessage_msg
#define PingMessage_fields &PingMessage_msg
#define SystemTick_fields &SystemTick_msg
#define ConsoleMessage_fields &ConsoleMessage_msg
#define SystemInfoMessage_fields &SystemInfoMessage_msg
#define ParamIOMessage_fields &ParamIOMessage_msg
#define SystemControlMessage_fields &SystemControlMessage_msg
#define SwitchModeMessage_fields &SwitchModeMessage_msg

/* Maximum encoded size of messages (where known) */
#define AckNackMessage_size                      16
#define BaseMessage_size                         12
#define ConsoleMessage_size                      149
#define Header_size                              10
#define ParamIOMessage_size                      91
#define PingMessage_size                         12
#define SwitchModeMessage_size                   14
#define SystemControlMessage_size                12
#define SystemInfoMessage_size                   69
#define SystemTick_size                          18

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
