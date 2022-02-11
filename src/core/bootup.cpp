/******************************************************************************
 *  File Name:
 *    bootup.cpp
 *
 *  Description:
 *    Device driver power up procedures
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/assert>
#include <Chimera/gpio>
#include <Chimera/serial>
#include <etl/circular_buffer.h>
#include <src/config/bsp/board_map.hpp>
#include <src/core/bootup.hpp>


namespace Orbit::Boot
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t HWBufferSize  = 32;
  static constexpr size_t CircleBufSize = 2 * HWBufferSize;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  // Serial Transmit Buffers
  static std::array<uint8_t, HWBufferSize>            sTXHWBuffer;
  static etl::circular_buffer<uint8_t, CircleBufSize> sTXCircularBuffer;

  // Serial Receive Buffers
  static std::array<uint8_t, HWBufferSize>            sRXHWBuffer;
  static etl::circular_buffer<uint8_t, CircleBufSize> sRXCircularBuffer;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void power_up_adc()
  {
  }


  static void power_up_can()
  {
  }


  static void power_up_gpio()
  {
    using namespace Chimera::GPIO;

    PinInit     cfg;
    Driver_rPtr pin = nullptr;

    /*-------------------------------------------------------------------------
    Heartbeat/Status LED
    -------------------------------------------------------------------------*/
    cfg.clear();
    cfg.validity  = true;
    cfg.threaded  = true;
    cfg.alternate = Alternate::NONE;
    cfg.drive     = Drive::OUTPUT_PUSH_PULL;
    cfg.pin       = IO::GPIO::pinHeartbeat;
    cfg.port      = IO::GPIO::portHeartbeat;
    cfg.pull      = Pull::NO_PULL;
    cfg.state     = State::LOW;

    pin = getDriver( cfg.port, cfg.pin );
    RT_HARD_ASSERT( pin != nullptr );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( cfg ) );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->setState( State::LOW ) );

    /*-------------------------------------------------------------------------
    User Button
    -------------------------------------------------------------------------*/
    cfg.clear();
    cfg.threaded  = true;
    cfg.validity  = true;
    cfg.alternate = Chimera::GPIO::Alternate::NONE;
    cfg.drive     = Chimera::GPIO::Drive::INPUT;
    cfg.pin       = IO::GPIO::pinButton;
    cfg.port      = IO::GPIO::portButton;
    cfg.pull      = Chimera::GPIO::Pull::PULL_UP;
    cfg.state     = Chimera::GPIO::State::HIGH;

    pin = getDriver( cfg.port, cfg.pin );
    RT_HARD_ASSERT( pin != nullptr );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( cfg ) );
  }


  static void power_up_i2c()
  {
  }


  static void power_up_timers()
  {
  }


  static void power_up_usart()
  {
    using namespace Chimera::Serial;
    using namespace Chimera::Hardware;

    /*-------------------------------------------------------------------------
    Configuration info for the serial object
    -------------------------------------------------------------------------*/
    IOPins pins;
    pins.tx = IO::DBG::txPinInit;
    pins.rx = IO::DBG::rxPinInit;

    Config cfg = IO::DBG::comConfig;

    /*-------------------------------------------------------------------------
    Create the serial object and initialize it
    -------------------------------------------------------------------------*/
    auto result = Chimera::Status::OK;
    auto serial = Chimera::Serial::getDriver( IO::DBG::serialChannel );
    RT_HARD_ASSERT( serial );

    result |= serial->assignHW( IO::DBG::serialChannel, pins );
    result |= serial->configure( cfg );
    result |= serial->enableBuffering( SubPeripheral::TX, sTXCircularBuffer, sTXHWBuffer.data(), sTXHWBuffer.size() );
    result |= serial->enableBuffering( SubPeripheral::RX, sRXCircularBuffer, sRXHWBuffer.data(), sRXHWBuffer.size() );
    RT_HARD_ASSERT( result == Chimera::Status::OK );

    result = serial->begin( PeripheralMode::INTERRUPT, PeripheralMode::INTERRUPT );
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUpSystemDrivers()
  {
    power_up_adc();
    power_up_can();
    power_up_gpio();
    power_up_i2c();
    power_up_timers();
    power_up_usart();
  }


  void startTasks()
  {
  }

}    // namespace Orbit::Boot
