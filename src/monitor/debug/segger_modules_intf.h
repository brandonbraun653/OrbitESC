/******************************************************************************
 *  File Name:
 *    segger_modules.hpp
 *
 *  Description:
 *    Segger SystemView module definitions. Declared as a C style header to
 *    allow easier instrumentation of project C-libs.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SEGGER_MODULES_INTF_HPP
#define ORBIT_SEGGER_MODULES_INTF_HPP

#ifdef __cplusplus
extern "C"
{
#endif

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "stdint.h"

/*-----------------------------------------------------------------------------
Enumerations
-----------------------------------------------------------------------------*/

typedef uint8_t OrbitMonitor_TUSB_Events;
enum
{
  TUSB_Init,
  TUSB_Mount,
  TUSB_Unmount,

  /*---------------------------------------------------------------------------
  Task Events: Handled by the TUSB task
  ---------------------------------------------------------------------------*/
  TUSB_TSK_EVENT_BUS_RESET,
  TUSB_TSK_EVENT_UNPLUGGED,
  TUSB_TSK_EVENT_SETUP_RECEIVED,
  TUSB_TSK_EVENT_XFER_COMPLETE,
  TUSB_TSK_EVENT_SUSPEND,
  TUSB_TSK_EVENT_RESUME,
  TUSB_TSK_EVENT_UNHANDLED,

  /*---------------------------------------------------------------------------
  DCD Events: Handled by the Synopsys Device Control Driver (DCD)
  ---------------------------------------------------------------------------*/
  TUSB_DCD_ISR_RESET,
  TUSB_DCD_ISR_ENUMERATION_DONE,
  TUSB_DCD_ISR_SUSPEND,
  TUSB_DCD_ISR_WAKEUP_DETECTED,
  TUSB_DCD_ISR_OTG_INTERRUPT,
  TUSB_DCD_ISR_START_OF_FRAME,
  TUSB_DCD_ISR_RX_FIFO_NOT_EMPTY,
  TUSB_DCD_ISR_OUT_ENDPOINT_INTERRUPT,
  TUSB_DCD_ISR_IN_ENDPOINT_INTERRUPT,

  /*---------------------------------------------------------------------------
  HW Events
  ---------------------------------------------------------------------------*/
  TUSB_DP_PULLUP_ENABLE,
  TUSB_DP_PULLUP_DISABLE,

  OrbitMonitor_TUSB_NumEvents
};

/*-----------------------------------------------------------------------------
Public Functions
-----------------------------------------------------------------------------*/

/**
 * @brief Records an event with the TinyUSB module.
 *
 * @param event   Which event occured
 * @return void
 */
void OrbitMonitorRecordEvent_TUSB( const OrbitMonitor_TUSB_Events event );

#ifdef __cplusplus
}
#endif

#endif  /* !ORBIT_SEGGER_MODULES_INTF_HPP */
