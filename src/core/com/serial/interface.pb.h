/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7-dev */

#ifndef PB_SRC_CORE_COM_SERIAL_INTERFACE_PB_H_INCLUDED
#define PB_SRC_CORE_COM_SERIAL_INTERFACE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
/* Instrumentation message header common to all types */
typedef struct _InstHeader {
    uint8_t msgId;
    uint8_t subId;
    uint8_t size;
} InstHeader;

typedef struct _AckNackMessage {
    InstHeader header;
    bool acknowledge;
} AckNackMessage;

typedef struct _ConsoleMessage {
    InstHeader header;
    pb_byte_t data[128];
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
#define AckNackMessage_init_default              {InstHeader_init_default, 0}
#define ConsoleMessage_init_default              {InstHeader_init_default, {0}}
#define SystemInfoMessage_init_default           {InstHeader_init_default, 0, "", "", ""}
#define InstHeader_init_zero                     {0, 0, 0}
#define AckNackMessage_init_zero                 {InstHeader_init_zero, 0}
#define ConsoleMessage_init_zero                 {InstHeader_init_zero, {0}}
#define SystemInfoMessage_init_zero              {InstHeader_init_zero, 0, "", "", ""}

/* Field tags (for use in manual encoding/decoding) */
#define InstHeader_msgId_tag                     1
#define InstHeader_subId_tag                     2
#define InstHeader_size_tag                      3
#define AckNackMessage_header_tag                1
#define AckNackMessage_acknowledge_tag           2
#define ConsoleMessage_header_tag                1
#define ConsoleMessage_data_tag                  2
#define SystemInfoMessage_header_tag             1
#define SystemInfoMessage_systemTick_tag         2
#define SystemInfoMessage_swVersion_tag          3
#define SystemInfoMessage_description_tag        4
#define SystemInfoMessage_serialNumber_tag       5

/* Struct field encoding specification for nanopb */
#define InstHeader_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   msgId,             1) \
X(a, STATIC,   REQUIRED, UINT32,   subId,             2) \
X(a, STATIC,   REQUIRED, UINT32,   size,              3)
#define InstHeader_CALLBACK NULL
#define InstHeader_DEFAULT NULL

#define AckNackMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, BOOL,     acknowledge,       2)
#define AckNackMessage_CALLBACK NULL
#define AckNackMessage_DEFAULT NULL
#define AckNackMessage_header_MSGTYPE InstHeader

#define ConsoleMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, FIXED_LENGTH_BYTES, data,              2)
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
extern const pb_msgdesc_t AckNackMessage_msg;
extern const pb_msgdesc_t ConsoleMessage_msg;
extern const pb_msgdesc_t SystemInfoMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define InstHeader_fields &InstHeader_msg
#define AckNackMessage_fields &AckNackMessage_msg
#define ConsoleMessage_fields &ConsoleMessage_msg
#define SystemInfoMessage_fields &SystemInfoMessage_msg

/* Maximum encoded size of messages (where known) */
#define AckNackMessage_size                      13
#define ConsoleMessage_size                      142
#define InstHeader_size                          9
#define SystemInfoMessage_size                   68

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
