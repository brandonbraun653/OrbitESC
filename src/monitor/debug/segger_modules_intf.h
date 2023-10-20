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
Enumerations
-----------------------------------------------------------------------------*/

enum OrbitMonitor_TUSB_Events
{
  TUSB_Init,

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
