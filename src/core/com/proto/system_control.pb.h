/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_SRC_CORE_COM_PROTO_SYSTEM_CONTROL_PB_H_INCLUDED
#define PB_SRC_CORE_COM_PROTO_SYSTEM_CONTROL_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _SystemControlSubId {
    SystemControlSubId_SUB_MSG_SYS_CTRL_RESET = 1, /* Reset the system */
    SystemControlSubId_SUB_MSG_SYS_CTRL_MOTOR = 2, /* Inject manual motor control commands */
    SystemControlSubId_SUB_MSG_SYS_CTRL_CAL_ADC = 3, /* Calibrate the ADC */
    SystemControlSubId_SUB_MSG_SYS_CTRL_MANUAL_INNER_LOOP = 4, /* Manual inner loop control enable/disable */
    SystemControlSubId_SUB_MSG_SYS_CTRL_MANUAL_INNER_LOOP_REF = 5, /* New references to use for manual inner loop control */
    SystemControlSubId_SUB_MSG_ENABLE_STREAM_PHASE_CURRENTS = 6, /* Enable streaming of phase currents */
    SystemControlSubId_SUB_MSG_DISABLE_STREAM_PHASE_CURRENTS = 7, /* Disable streaming of phase currents */
    SystemControlSubId_SUB_MSG_ENABLE_STREAM_SYSTEM_VOLTAGES = 8, /* Enable streaming of system voltages */
    SystemControlSubId_SUB_MSG_DISABLE_STREAM_SYSTEM_VOLTAGES = 9, /* Disable streaming of system voltages */
    SystemControlSubId_SUB_MSG_ENABLE_STREAM_PHASE_VOLTAGES = 10, /* Enable streaming of phase voltage commands */
    SystemControlSubId_SUB_MSG_DISABLE_STREAM_PHASE_VOLTAGES = 11, /* Disable streaming of phase voltage commands */
    SystemControlSubId_SUB_MSG_ENABLE_STREAM_STATE_ESTIMATES = 12, /* Enable streaming of state estimates */
    SystemControlSubId_SUB_MSG_DISABLE_STREAM_STATE_ESTIMATES = 13, /* Disable streaming of state estimates */
    SystemControlSubId_SUB_MSG_SYS_CTRL_ARM = 14, /* Arm the motor control system */
    SystemControlSubId_SUB_MSG_SYS_CTRL_DISARM = 15, /* Disarm the motor control system */
    SystemControlSubId_SUB_MSG_SYS_CTRL_ENGAGE = 16, /* Engage the motor control system */
    SystemControlSubId_SUB_MSG_SYS_CTRL_DISENGAGE = 17, /* Disengage the motor control system */
    SystemControlSubId_SUB_MSG_SYS_CTRL_FAULT = 18, /* Panic the motor control system */
    SystemControlSubId_SUB_MSG_SYS_CTRL_EMERGENCY_STOP = 19 /* Emergency stop the motor control system */
} SystemControlSubId;

#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _SystemControlSubId_MIN SystemControlSubId_SUB_MSG_SYS_CTRL_RESET
#define _SystemControlSubId_MAX SystemControlSubId_SUB_MSG_SYS_CTRL_EMERGENCY_STOP
#define _SystemControlSubId_ARRAYSIZE ((SystemControlSubId)(SystemControlSubId_SUB_MSG_SYS_CTRL_EMERGENCY_STOP+1))


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
