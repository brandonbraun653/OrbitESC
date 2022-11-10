/******************************************************************************
 *  File Name:
 *    orbit_i2c.cpp
 *
 *  Description:
 *    Orbit I2C bus driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/gpio>
#include <Chimera/i2c>
#include <src/core/hw/orbit_i2c.hpp>
#include <src/config/bsp/board_map.hpp>


namespace Orbit::I2C
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Build up the config
    -------------------------------------------------------------------------*/
    Chimera::I2C::DriverConfig cfg;
    cfg.clear();
    cfg.validity         = true;
    cfg.HWInit.channel   = IO::I2C::channel;
    cfg.HWInit.frequency = IO::I2C::frequency;

    cfg.SCLInit.clear();
    cfg.SCLInit.alternate = Chimera::GPIO::Alternate::I2C1_SCL;
    cfg.SCLInit.drive     = Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN;
    cfg.SCLInit.pull      = Chimera::GPIO::Pull::NO_PULL;
    cfg.SCLInit.pin       = IO::I2C::sclPin;
    cfg.SCLInit.port      = IO::I2C::sclPort;
    cfg.SCLInit.validity  = true;

    cfg.SDAInit.clear();
    cfg.SDAInit.alternate = Chimera::GPIO::Alternate::I2C1_SDA;
    cfg.SDAInit.drive     = Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN;
    cfg.SDAInit.pull      = Chimera::GPIO::Pull::NO_PULL;
    cfg.SDAInit.pin       = IO::I2C::sdaPin;
    cfg.SDAInit.port      = IO::I2C::sdaPort;
    cfg.SDAInit.validity  = true;

#if defined( SIMULATOR )
    cfg.memFile = std::filesystem::current_path() / "orbit_esc_eeprom.bin";
    cfg.memSize = 1 * 1024 * 1024;
#endif

    /*-------------------------------------------------------------------------
    Power on the hardware peripheral
    -------------------------------------------------------------------------*/
    auto i2c = Chimera::I2C::getDriver( IO::I2C::channel );
    RT_HARD_ASSERT( i2c );
    RT_HARD_ASSERT( Chimera::Status::OK == i2c->open( cfg ) );
  }

}  // namespace Orbit::I2C
