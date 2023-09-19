/******************************************************************************
 *  File Name:
 *    orbit_tusb.h
 *
 *  Description:
 *    TinyUSB driver hook interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_TINYUSB_DRIVER_HPP
#define ORBIT_ESC_TINYUSB_DRIVER_HPP

#ifdef __cplusplus
extern "C"
{
#endif

/*-----------------------------------------------------------------------------
Public Functions
-----------------------------------------------------------------------------*/

/**
 * @brief Powers on the device clocks
 * @return void
 */
void tusb_configure_clocks();

#ifdef __cplusplus
}
#endif

#endif  /* !ORBIT_ESC_TINYUSB_DRIVER_HPP */
