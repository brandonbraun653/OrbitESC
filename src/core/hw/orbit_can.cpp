/******************************************************************************
 *  File Name:
 *    orbit_can.cpp
 *
 *  Description:
 *    Orbit CAN bus driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/hw/orbit_can.hpp>


namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    Chimera::CAN::DriverConfig cfg;
  }

}  // namespace Orbit::CAN
