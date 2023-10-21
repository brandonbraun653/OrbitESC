/******************************************************************************
 *  File Name:
 *    orbit_usb_intf.h
 *
 *  Description:
 *    C interface to the USB driver
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_USB_INTF_HPP
#define ORBIT_USB_INTF_HPP

#ifdef __cplusplus
extern "C"
{
#endif

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Enables/disables the D+ line pullup resistor
   *
   * @param state  True to enable, false to disable
   * @return void
   */
  void OrbitSetDPPullupState( const bool state );

#ifdef __cplusplus
}
#endif

#endif  /* !ORBIT_USB_INTF_HPP */
