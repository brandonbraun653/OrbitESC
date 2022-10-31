/******************************************************************************
 *  File Name:
 *    orbit_spi.cpp
 *
 *  Description:
 *    Orbit SPI bus driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/spi>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_spi.hpp>


namespace Orbit::SPI
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Build the configuration
    -------------------------------------------------------------------------*/
    Chimera::SPI::DriverConfig cfg;
    cfg.clear();
    cfg.SCKInit    = IO::SPI::sckPinInit;
    cfg.MOSIInit   = IO::SPI::mosiPinInit;
    cfg.MISOInit   = IO::SPI::misoPinInit;
    cfg.HWInit     = IO::SPI::spiHwInit;
    cfg.validity   = true;

    /*-------------------------------------------------------------------------
    Configure the GPIO pins
    -------------------------------------------------------------------------*/
    Chimera::GPIO::Driver_rPtr pin = nullptr;

    pin = Chimera::GPIO::getDriver( IO::SPI::ledCSPort, IO::SPI::ledCSPin );
    RT_HARD_ASSERT( pin );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( IO::SPI::ledCSPinInit ) );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->setState( Chimera::GPIO::State::HIGH ) );

    pin = Chimera::GPIO::getDriver( IO::SPI::norCSPort, IO::SPI::norCSPin );
    RT_HARD_ASSERT( pin );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( IO::SPI::norCSPinInit ) );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->setState( Chimera::GPIO::State::HIGH ) );

    /*-------------------------------------------------------------------------
    Initialize the driver
    -------------------------------------------------------------------------*/
    auto driver = Chimera::SPI::getDriver( Orbit::IO::SPI::spiChannel );
    RT_HARD_ASSERT( driver );
    RT_HARD_ASSERT( Chimera::Status::OK == driver->init( cfg ) );
  }
}  // namespace Orbit::SPI
