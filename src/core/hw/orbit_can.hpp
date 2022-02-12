/******************************************************************************
 *  File Name:
 *    orbit_can.hpp
 *
 *  Description:
 *    Orbit ESC CAN Driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_CAN_HPP
#define ORBIT_ESC_CAN_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/can>


namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
    /**
   * @brief Powers up the CAN driver subsystem
   */
  void powerUp();

}  // namespace Orbit::CAN

#endif  /* !ORBIT_ESC_CAN_HPP */
