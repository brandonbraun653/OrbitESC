/******************************************************************************
 *  File Name:
 *    orbit_data.cpp
 *
 *  Description:
 *    Insert Description
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Aurora/memory>
#include <src/core/data/orbit_data.hpp>
#include <src/config/bsp/board_map.hpp>

namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Aurora::Flash::EEPROM::Driver sEEPROMFlash; /**< Flash memory driver supporting the file system */

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool initialize()
  {
    /*-------------------------------------------------------------------------
    Initialize the EEPROM driver
    -------------------------------------------------------------------------*/
    // Aurora::Flash::EEPROM::DeviceConfig cfg;
    // cfg.clear();
    // cfg.whichChip     = Aurora::Flash::EEPROM::Chip::AT24C02;
    // cfg.deviceAddress = 0x53;
    // cfg.i2cChannel    = IO::I2C::channel;

    // RT_HARD_ASSERT( sEEPROMFlash.configure( cfg ) );

    /*-------------------------------------------------------------------------
    Power on the file system
    -------------------------------------------------------------------------*/
    return true;
  }

}    // namespace Orbit::Data
