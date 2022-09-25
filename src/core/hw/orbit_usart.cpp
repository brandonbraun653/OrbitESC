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
  static Aurora::Logging::SerialSink      s_serial_sink;
  static Aurora::Logging::SinkHandle_rPtr s_serial_handle;

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
    pins.tx = IO::USART::txPinInit;
    pins.rx = IO::USART::rxPinInit;

    Config cfg = IO::USART::comConfig;

    /*-------------------------------------------------------------------------
    Create the serial object and initialize it
    -------------------------------------------------------------------------*/
    auto result = Chimera::Status::OK;
    result |= attach( Chimera::Peripheral::Type::PERIPH_USART, IO::USART::serialChannel );

    auto usart  = Chimera::USART::getDriver( IO::USART::serialChannel );
    auto serial = Chimera::Serial::getDriver( IO::USART::serialChannel );
    RT_HARD_ASSERT( serial && usart );

    result |= usart->assignHW( IO::USART::serialChannel, pins );
    result |= usart->configure( cfg );
    result |= usart->enableBuffering( SubPeripheral::TX, sTXCircularBuffer, sTXHWBuffer.data(), sTXHWBuffer.size() );
    result |= usart->enableBuffering( SubPeripheral::RX, sRXCircularBuffer, sRXHWBuffer.data(), sRXHWBuffer.size() );
    RT_HARD_ASSERT( result == Chimera::Status::OK );

    result |= usart->begin( PeripheralMode::INTERRUPT, PeripheralMode::INTERRUPT );

    /*-------------------------------------------------------------------------
    Start the logging framework
    -------------------------------------------------------------------------*/
    Aurora::Logging::initialize();
    Aurora::Logging::setGlobalLogLevel( Aurora::Logging::Level::LVL_TRACE );

    s_serial_sink.assignChannel( IO::USART::serialChannel );
    s_serial_sink.logLevel = Aurora::Logging::Level::LVL_TRACE;
    s_serial_sink.enabled  = true;
    s_serial_sink.name     = "HWLogger";

    if ( !s_serial_handle )
    {
      s_serial_handle = Aurora::Logging::SinkHandle_rPtr( &s_serial_sink );
      registerSink( s_serial_handle );
    }

    Aurora::Logging::setRootSink( s_serial_handle );

    /*-------------------------------------------------------------------------
    Clear the terminal screen
    -------------------------------------------------------------------------*/
    serial->write( Aurora::Logging::Terminal::CmdClearScreen.data(), Aurora::Logging::Terminal::CmdClearScreen.size() );
  }

}    // namespace Orbit::USART
