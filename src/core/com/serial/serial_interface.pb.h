/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7-dev */

#ifndef PB_SRC_CORE_COM_SERIAL_SERIAL_INTERFACE_PB_H_INCLUDED
#define PB_SRC_CORE_COM_SERIAL_SERIAL_INTERFACE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
/* Instrumentation message header common to all types. Each functional message type **must**
 have this first in their list of declarations. */
typedef struct _InstHeader {
    uint8_t msgId; /* Root message identifier */
    uint8_t subId; /* Possible sub-identifier to specify root ID details */
    uint16_t uuid; /* Unique ID for the message */
} InstHeader;

/* Root type that parsers can use to peek at messages */
typedef struct _BaseMessage {
    InstHeader header;
} BaseMessage;

/* Generic ACK or NACK to a previous message */
typedef struct _AckNackMessage {
    InstHeader header;
    bool acknowledge;
} AckNackMessage;

typedef struct _PingMessage {
    InstHeader header;
} PingMessage;

typedef struct _SystemTick {
    InstHeader header;
    uint32_t tick;
} SystemTick;

typedef PB_BYTES_ARRAY_T(128) ConsoleMessage_data_t;
typedef struct _ConsoleMessage {
    InstHeader header;
    uint8_t this_frame;
    uint8_t total_frames;
    ConsoleMessage_data_t data;
} ConsoleMessage;

typedef struct _SystemInfoMessage {
    InstHeader header;
    uint32_t systemTick;
    char swVersion[16];
    char description[16];
    char serialNumber[16];
} SystemInfoMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define InstHeader_init_default                  {0, 0, 0}
#define BaseMessage_init_default                 {InstHeader_init_default}
#define AckNackMessage_init_default              {InstHeader_init_default, 0}
#define PingMessage_init_default                 {InstHeader_init_default}
#define SystemTick_init_default                  {InstHeader_init_default, 0}
#define ConsoleMessage_init_default              {InstHeader_init_default, 0, 0, {0, {0}}}
#define SystemInfoMessage_init_default           {InstHeader_init_default, 0, "", "", ""}
#define InstHeader_init_zero                     {0, 0, 0}
#define BaseMessage_init_zero                    {InstHeader_init_zero}
#define AckNackMessage_init_zero                 {InstHeader_init_zero, 0}
#define PingMessage_init_zero                    {InstHeader_init_zero}
#define SystemTick_init_zero                     {InstHeader_init_zero, 0}
#define ConsoleMessage_init_zero                 {InstHeader_init_zero, 0, 0, {0, {0}}}
#define SystemInfoMessage_init_zero              {InstHeader_init_zero, 0, "", "", ""}

/* Field tags (for use in manual encoding/decoding) */
#define InstHeader_msgId_tag                     1
#define InstHeader_subId_tag                     2
#define InstHeader_uuid_tag                      3
#define BaseMessage_header_tag                   1
#define AckNackMessage_header_tag                1
#define AckNackMessage_acknowledge_tag           2
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

/* Struct field encoding specification for nanopb */
#define InstHeader_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   msgId,             1) \
X(a, STATIC,   REQUIRED, UINT32,   subId,             2) \
X(a, STATIC,   REQUIRED, UINT32,   uuid,              3)
#define InstHeader_CALLBACK NULL
#define InstHeader_DEFAULT NULL

#define BaseMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1)
#define BaseMessage_CALLBACK NULL
#define BaseMessage_DEFAULT NULL
#define BaseMessage_header_MSGTYPE InstHeader

#define AckNackMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, BOOL,     acknowledge,       2)
#define AckNackMessage_CALLBACK NULL
#define AckNackMessage_DEFAULT NULL
#define AckNackMessage_header_MSGTYPE InstHeader

#define PingMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1)
#define PingMessage_CALLBACK NULL
#define PingMessage_DEFAULT NULL
#define PingMessage_header_MSGTYPE InstHeader

#define SystemTick_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   tick,              2)
#define SystemTick_CALLBACK NULL
#define SystemTick_DEFAULT NULL
#define SystemTick_header_MSGTYPE InstHeader

#define ConsoleMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   this_frame,        2) \
X(a, STATIC,   REQUIRED, UINT32,   total_frames,      3) \
X(a, STATIC,   REQUIRED, BYTES,    data,              4)
#define ConsoleMessage_CALLBACK NULL
#define ConsoleMessage_DEFAULT NULL
#define ConsoleMessage_header_MSGTYPE InstHeader

#define SystemInfoMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   systemTick,        2) \
X(a, STATIC,   REQUIRED, STRING,   swVersion,         3) \
X(a, STATIC,   REQUIRED, STRING,   description,       4) \
X(a, STATIC,   REQUIRED, STRING,   serialNumber,      5)
#define SystemInfoMessage_CALLBACK NULL
#define SystemInfoMessage_DEFAULT NULL
#define SystemInfoMessage_header_MSGTYPE InstHeader

extern const pb_msgdesc_t InstHeader_msg;
extern const pb_msgdesc_t BaseMessage_msg;
extern const pb_msgdesc_t AckNackMessage_msg;
extern const pb_msgdesc_t PingMessage_msg;
extern const pb_msgdesc_t SystemTick_msg;
extern const pb_msgdesc_t ConsoleMessage_msg;
extern const pb_msgdesc_t SystemInfoMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define InstHeader_fields &InstHeader_msg
#define BaseMessage_fields &BaseMessage_msg
#define AckNackMessage_fields &AckNackMessage_msg
#define PingMessage_fields &PingMessage_msg
#define SystemTick_fields &SystemTick_msg
#define ConsoleMessage_fields &ConsoleMessage_msg
#define SystemInfoMessage_fields &SystemInfoMessage_msg

/* Maximum encoded size of messages (where known) */
#define AckNackMessage_size                      14
#define BaseMessage_size                         12
#define ConsoleMessage_size                      149
#define InstHeader_size                          10
#define PingMessage_size                         12
#define SystemInfoMessage_size                   69
#define SystemTick_size                          18

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
