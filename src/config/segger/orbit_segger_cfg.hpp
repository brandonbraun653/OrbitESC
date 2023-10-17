/******************************************************************************
 *  File Name:
 *    orbit_segger_cfg.hpp
 *
 *  Description:
 *    Segger configuration options
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SEGGER_CONFIG_HPP
#define ORBIT_ESC_SEGGER_CONFIG_HPP

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "SEGGER_SYSVIEW.h"

/*-------------------------------------------------------------------------------
Disable SysView events. Cortex-M4 + JLink can only handle about 5k events/sec
before overflows start occurring. The mask disables enough items to get the event
rate to ~2.5k/s while gathering data to rebuild the task switching structure. This
should leave enough room for user profiling/debugging in the future.

Reasoning for Disabled:
SYSVIEW_EVTMASK_ALL_INTERRUPTS    | Project uses a large number of ISRs
SYSVIEW_EVTMASK_SYSTIME_CYCLES    | Irrelevant to build system timing info
SYSVIEW_EVTMASK_SYSTIME_US        | Irrelevant to build system timing info
SYSVIEW_EVTMASK_TIMER_ENTER       | Irrelevant to build system timing info
SYSVIEW_EVTMASK_TIMER_EXIT        | Irrelevant to build system timing info
SYSVIEW_EVTMASK_TASK_STOP_EXEC    | SysView can re-build task sequences without this
SYSVIEW_EVTMASK_TASK_START_READY  | SysView can re-build task sequences without this
SYSVIEW_EVTMASK_TASK_STOP_READY   | SysView can re-build task sequences without this
-------------------------------------------------------------------------------*/
/* clang-format off */
#define SEGGER_DISABLE_MASK             \
  ( SYSVIEW_EVTMASK_SYSTIME_CYCLES    | \
    SYSVIEW_EVTMASK_SYSTIME_US        | \
    SYSVIEW_EVTMASK_TIMER_ENTER       | \
    SYSVIEW_EVTMASK_TIMER_EXIT        | \
    SYSVIEW_EVTMASK_TASK_STOP_EXEC    | \
    SYSVIEW_EVTMASK_TASK_START_READY  | \
    SYSVIEW_EVTMASK_TASK_STOP_READY )
/* clang-format on */

/*-----------------------------------------------------------------------------
Enumerations
-----------------------------------------------------------------------------*/
#endif  /* !ORBIT_ESC_SEGGER_CONFIG_HPP */

#ifdef __cplusplus
}
#endif
