/******************************************************************************
 *  File Name:
 *    orbit_usart.cpp
 *
 *  Description:
 *    Orbit USART bus driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/usart>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_usart.hpp>


namespace Orbit::USART
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

  // Logger Sink Handles
  static Aurora::Logging::SerialSink s_serial_sink;
  static Aurora::Logging::SinkHandle s_serial_handle;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
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

    /*-------------------------------------------------------------------------
    Start the logging framework
    -------------------------------------------------------------------------*/
    Aurora::Logging::initialize();
    Aurora::Logging::setGlobalLogLevel( Aurora::Logging::Level::LVL_TRACE );

    s_serial_sink.assignChannel( IO::DBG::serialChannel );
    s_serial_sink.setLogLevel( Aurora::Logging::Level::LVL_TRACE );
    s_serial_sink.enable();
    s_serial_sink.setName( "HWLogger" );

    if ( !s_serial_handle )
    {
      s_serial_handle = Aurora::Logging::SinkHandle( &s_serial_sink );
      registerSink( s_serial_handle );
    }

    Aurora::Logging::setRootSink( s_serial_handle );

    /*-------------------------------------------------------------------------
    Clear the terminal screen
    -------------------------------------------------------------------------*/
    serial->write( Aurora::Logging::Terminal::CmdClearScreen.data(), Aurora::Logging::Terminal::CmdClearScreen.size() );
    serial->await( Chimera::Event::Trigger::TRIGGER_WRITE_COMPLETE, Chimera::Thread::TIMEOUT_BLOCK );
  }

}    // namespace Orbit::USART
