/******************************************************************************
 *  File Name:
 *    orbit_usart.cpp
 *
 *  Description:
 *    Orbit USART bus driver
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/serial>
#include <Chimera/usart>
#include <src/config/bsp/board_map.hpp>
#include <src/core/com/serial/serial_config.hpp>
#include <src/core/com/serial/serial_sink.hpp>
#include <src/core/hw/orbit_isr_config.hpp>
#include <src/core/hw/orbit_usart.hpp>


namespace Orbit::USART
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  Chimera::Serial::Driver_rPtr SerialDriver;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static etl::bip_buffer_spsc_atomic<uint8_t, 1024> sTxBuffer;
  static etl::bip_buffer_spsc_atomic<uint8_t, 256>  sRxBuffer;

  // Logger Sink Handles
  static Orbit::Serial::EncodedLogSink    s_serial_sink;
  static Aurora::Logging::SinkHandle_rPtr s_serial_handle;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    using namespace Chimera::Serial;
    using namespace Chimera::Hardware;

    Chimera::GPIO::Driver_rPtr pin = nullptr;

    /*-------------------------------------------------------------------------
    Configure the CP2104 reset line to be disabled
    -------------------------------------------------------------------------*/
#if defined( ORBIT_ESC_V1 )
    pin = Chimera::GPIO::getDriver( IO::USART::resetPort, IO::USART::resetPin );

    RT_HARD_ASSERT( pin );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( IO::USART::resetPinInit ) );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->setState( Chimera::GPIO::State::HIGH ) );
#endif /* ORBIT_ESC_V1 */

    /*-------------------------------------------------------------------------
    Configure the peripheral IO pins
    -------------------------------------------------------------------------*/
    pin = Chimera::GPIO::getDriver( IO::USART::txPort, IO::USART::txPin );
    RT_HARD_ASSERT( pin );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( IO::USART::txPinInit ) );

    pin = Chimera::GPIO::getDriver( IO::USART::rxPort, IO::USART::rxPin );
    RT_HARD_ASSERT( pin );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( IO::USART::rxPinInit ) );

    /*-------------------------------------------------------------------------
    Create the serial object and initialize it
    -------------------------------------------------------------------------*/
    Chimera::Serial::Config comConfig;
    comConfig.baud     = IO::USART::baudRate;
    comConfig.channel  = IO::USART::serialChannel;
    comConfig.width    = Chimera::Serial::CharWid::CW_8BIT;
    comConfig.parity   = Chimera::Serial::Parity::PAR_NONE;
    comConfig.stopBits = Chimera::Serial::StopBits::SBITS_ONE;
    comConfig.flow     = Chimera::Serial::FlowControl::FCTRL_NONE;
    comConfig.txfrMode = Chimera::Serial::TxfrMode::DMA;
    comConfig.txBuffer = dynamic_cast<Chimera::Serial::BipBuffer *>( &sTxBuffer );
    comConfig.rxBuffer = dynamic_cast<Chimera::Serial::BipBuffer *>( &sRxBuffer );

    SerialDriver = Chimera::Serial::getDriver( IO::USART::serialChannel );
    RT_HARD_ASSERT( SerialDriver );
    RT_HARD_ASSERT( Chimera::Status::OK == SerialDriver->open( comConfig ) );

    /*-------------------------------------------------------------------------
    Start the logging framework
    -------------------------------------------------------------------------*/
    Aurora::Logging::initialize();
    Aurora::Logging::setGlobalLogLevel( Aurora::Logging::Level::LVL_TRACE );

    s_serial_sink.assignChannel( IO::USART::serialChannel );
    s_serial_sink.logLevel = Aurora::Logging::Level::LVL_TRACE;
    s_serial_sink.enabled  = true;
    s_serial_sink.name     = "SerialLog";

    if( !s_serial_handle )
    {
      s_serial_handle = Aurora::Logging::SinkHandle_rPtr( &s_serial_sink );
      registerSink( s_serial_handle );
    }

    RT_HARD_ASSERT( Aurora::Logging::Result::RESULT_SUCCESS == Aurora::Logging::setRootSink( s_serial_handle ) );
  }

}    // namespace Orbit::USART


namespace Orbit::Serial::Config
{
  Chimera::Serial::Driver_rPtr getDebugPort()
  {
    return ::Orbit::USART::SerialDriver;
  }
}    // namespace Orbit::Serial::Config
