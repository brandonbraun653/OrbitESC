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
#include <Chimera/can>
#include <Chimera/common>
#include <Chimera/gpio>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_can.hpp>
#include <string_view>


namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t CAN_FRAME_BUF_SIZE = 8;

  static constexpr Chimera::CAN::Filter s_filter_list[] = {
    { .id = 0xFFFF, .mask = 0, .extended = false }  // Accept everything
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::CAN::BasicFrame s_tx_frame_buffer[ CAN_FRAME_BUF_SIZE ];
  static Chimera::CAN::BasicFrame s_rx_frame_buffer[ CAN_FRAME_BUF_SIZE ];
  static const std::array<std::string_view, EnumValue( NodeId::NUM_SUPPORTED_NODES ) + 1u> s_node_names = {
    "Node 0", "Node 1", "Node 2", "Node 3", "PC", "All", "Unknown"
  };

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  Chimera::CAN::Driver_rPtr CANDriver;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUp()
  {
    Chimera::GPIO::Driver_rPtr pin = nullptr;
    Chimera::CAN::DriverConfig cfg;

    /*-------------------------------------------------------------------------
    TX GPIO
    -------------------------------------------------------------------------*/
    cfg.TXInit.clear();
    cfg.TXInit.threaded  = true;
    cfg.TXInit.validity  = true;
    cfg.TXInit.alternate = Chimera::GPIO::Alternate::CAN1_TX;
    cfg.TXInit.drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL;
    cfg.TXInit.pin       = IO::CAN::pinTX;
    cfg.TXInit.port      = IO::CAN::portTX;
    cfg.TXInit.pull      = Chimera::GPIO::Pull::PULL_UP;

    pin = Chimera::GPIO::getDriver( cfg.TXInit.port, cfg.TXInit.pin );
    RT_HARD_ASSERT( pin != nullptr );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( cfg.TXInit ) );

    /*-------------------------------------------------------------------------
    RX GPIO
    -------------------------------------------------------------------------*/
    cfg.RXInit.clear();
    cfg.RXInit.threaded  = true;
    cfg.RXInit.validity  = true;
    cfg.RXInit.alternate = Chimera::GPIO::Alternate::CAN1_RX;
    cfg.RXInit.drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL;
    cfg.RXInit.pin       = IO::CAN::pinRX;
    cfg.RXInit.port      = IO::CAN::portRX;
    cfg.RXInit.pull      = Chimera::GPIO::Pull::PULL_UP;

    pin = Chimera::GPIO::getDriver( cfg.RXInit.port, cfg.RXInit.pin );
    RT_HARD_ASSERT( pin != nullptr );
    RT_HARD_ASSERT( Chimera::Status::OK == pin->init( cfg.RXInit ) );

    /*-------------------------------------------------------------------------
    Device Initialization
    -------------------------------------------------------------------------*/
    cfg.validity                  = true;
    cfg.HWInit.channel            = Chimera::CAN::Channel::CAN0;
    cfg.HWInit.txBuffer           = s_tx_frame_buffer;
    cfg.HWInit.txElements         = ARRAY_COUNT( s_tx_frame_buffer );
    cfg.HWInit.rxBuffer           = s_rx_frame_buffer;
    cfg.HWInit.rxElements         = ARRAY_COUNT( s_rx_frame_buffer );
    cfg.HWInit.samplePointPercent = 0.875f;
    cfg.HWInit.baudRate           = 1000000;
    cfg.HWInit.timeQuanta         = 16;
    cfg.HWInit.resyncJumpWidth    = 1;
    cfg.HWInit.maxBaudError       = 0.05;

    CANDriver = Chimera::CAN::getDriver( cfg.HWInit.channel );
    RT_HARD_ASSERT( CANDriver != nullptr );
    RT_HARD_ASSERT( Chimera::Status::OK == CANDriver->open( cfg ) );
    RT_HARD_ASSERT( Chimera::Status::OK == CANDriver->filter( s_filter_list, ARRAY_COUNT( s_filter_list ) ) );
  }


  const std::string_view& getNodeName( const NodeId id )
  {
    if( id < NodeId::NUM_SUPPORTED_NODES )
    {
      return s_node_names[ EnumValue( id ) ];
    }
    else
    {
      return s_node_names.back();
    }
  }

}    // namespace Orbit::CAN
