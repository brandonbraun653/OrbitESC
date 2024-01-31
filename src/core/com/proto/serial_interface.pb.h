/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8 */

#ifndef PB_SERIAL_INTERFACE_PB_H_INCLUDED
#define PB_SERIAL_INTERFACE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
/* Message IDs for all the core message types sent between the host and the device. */
typedef enum _MsgId {
    MsgId_MSG_ACK_NACK = 0, /* Generic ack/nack type message */
    MsgId_MSG_PING_CMD = 1, /* Simple PING to see if the node is alive */
    MsgId_MSG_TERMINAL = 2, /* Terminal command for printing text/debug data */
    MsgId_MSG_SYS_TICK = 3, /* System time tick */
    MsgId_MSG_SYS_INFO = 4, /* System information */
    MsgId_MSG_PARAM_IO = 5, /* Do operations on configurable parameters */
    MsgId_MSG_SYS_CTRL = 6, /* Perform system control operations */
    MsgId_MSG_SYS_DATA = 7, /* System data stream */
    MsgId_MSG_SYS_STATUS = 8 /* System status annunciation, essentially a snapshot of observable system state */
} MsgId;

/* Specialization sub-IDs to further specify the root message ID. */
typedef enum _SubId {
    SubId_SUB_MSG_NONE = 0 /* Invalid/empty sub-message ID */
} SubId;

/* Status codes for ACK/NACK messages */
typedef enum _StatusCode {
    StatusCode_NO_ERROR = 0,
    StatusCode_UNKNOWN_ERROR = 1,
    StatusCode_INVALID_PARAM = 2,
    StatusCode_INVALID_TYPE = 3,
    StatusCode_INVALID_VALUE = 4,
    StatusCode_REQUEST_FAILED = 5
} StatusCode;

/* Struct definitions */
/* Core message header common to all types. Each functional message type **must**
 have this first in their list of declarations. */
typedef struct _Header {
    uint8_t msgId; /* Root message identifier */
    uint8_t subId; /* Possible sub-identifier to specify root ID details */
    uint16_t uuid; /* Unique ID for the message */
} Header;

/* Root type that parsers can use to peek at messages and figure out what type the full message is. */
typedef struct _BaseMessage {
    Header header;
} BaseMessage;

typedef PB_BYTES_ARRAY_T(64) AckNackMessage_data_t;
/* Generic ACK or NACK to a previous message, with optional data payload */
typedef struct _AckNackMessage {
    Header header;
    bool acknowledge; /* True if this is an ACK, false if it's a NACK */
    StatusCode status_code; /* Status code for the ACK/NACK */
    bool has_data;
    AckNackMessage_data_t data; /* Optional data payload */
} AckNackMessage;

/* Simple PING message to see if the node is alive, client or server can send this. */
typedef struct _PingMessage {
    Header header;
} PingMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _MsgId_MIN MsgId_MSG_ACK_NACK
#define _MsgId_MAX MsgId_MSG_SYS_STATUS
#define _MsgId_ARRAYSIZE ((MsgId)(MsgId_MSG_SYS_STATUS+1))

#define _SubId_MIN SubId_SUB_MSG_NONE
#define _SubId_MAX SubId_SUB_MSG_NONE
#define _SubId_ARRAYSIZE ((SubId)(SubId_SUB_MSG_NONE+1))

#define _StatusCode_MIN StatusCode_NO_ERROR
#define _StatusCode_MAX StatusCode_REQUEST_FAILED
#define _StatusCode_ARRAYSIZE ((StatusCode)(StatusCode_REQUEST_FAILED+1))



#define AckNackMessage_status_code_ENUMTYPE StatusCode



/* Initializer values for message structs */
#define Header_init_default                      {0, 0, 0}
#define BaseMessage_init_default                 {Header_init_default}
#define AckNackMessage_init_default              {Header_init_default, 0, _StatusCode_MIN, false, {0, {0}}}
#define PingMessage_init_default                 {Header_init_default}
#define Header_init_zero                         {0, 0, 0}
#define BaseMessage_init_zero                    {Header_init_zero}
#define AckNackMessage_init_zero                 {Header_init_zero, 0, _StatusCode_MIN, false, {0, {0}}}
#define PingMessage_init_zero                    {Header_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define Header_msgId_tag                         1
#define Header_subId_tag                         2
#define Header_uuid_tag                          3
#define BaseMessage_header_tag                   1
#define AckNackMessage_header_tag                1
#define AckNackMessage_acknowledge_tag           2
#define AckNackMessage_status_code_tag           3
#define AckNackMessage_data_tag                  4
#define PingMessage_header_tag                   1

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
X(a, STATIC,   REQUIRED, UENUM,    status_code,       3) \
X(a, STATIC,   OPTIONAL, BYTES,    data,              4)
#define AckNackMessage_CALLBACK NULL
#define AckNackMessage_DEFAULT NULL
#define AckNackMessage_header_MSGTYPE Header

#define PingMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1)
#define PingMessage_CALLBACK NULL
#define PingMessage_DEFAULT NULL
#define PingMessage_header_MSGTYPE Header

extern const pb_msgdesc_t Header_msg;
extern const pb_msgdesc_t BaseMessage_msg;
extern const pb_msgdesc_t AckNackMessage_msg;
extern const pb_msgdesc_t PingMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Header_fields &Header_msg
#define BaseMessage_fields &BaseMessage_msg
#define AckNackMessage_fields &AckNackMessage_msg
#define PingMessage_fields &PingMessage_msg

/* Maximum encoded size of messages (where known) */
#define AckNackMessage_size                      82
#define BaseMessage_size                         12
#define Header_size                              10
#define PingMessage_size                         12
#define SERIAL_INTERFACE_PB_H_MAX_SIZE           AckNackMessage_size

#ifdef __cplusplus
} /* extern "C" */
#endif

#ifdef __cplusplus
/* Message descriptors for nanopb */
namespace nanopb {
template <>
struct MessageDescriptor<Header> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 3;
    static inline const pb_msgdesc_t* fields() {
        return &Header_msg;
    }
};
template <>
struct MessageDescriptor<BaseMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 1;
    static inline const pb_msgdesc_t* fields() {
        return &BaseMessage_msg;
    }
};
template <>
struct MessageDescriptor<AckNackMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 4;
    static inline const pb_msgdesc_t* fields() {
        return &AckNackMessage_msg;
    }
};
template <>
struct MessageDescriptor<PingMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 1;
    static inline const pb_msgdesc_t* fields() {
        return &PingMessage_msg;
    }
};
}  // namespace nanopb

#endif  /* __cplusplus */


#endif
