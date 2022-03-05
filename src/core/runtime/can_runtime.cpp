/******************************************************************************
 *  File Name:
 *    can_runtime.cpp
 *
 *  Description:
 *    CAN bus runtime functionality
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/can>
#include <Chimera/scheduler>
#include <Chimera/system>
#include <src/core/runtime/can_runtime.hpp>


namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static const char * NODE_PC_STR = "PC";

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::CAN::Driver_rPtr  s_can_driver;
  static Chimera::Scheduler::Polled s_polled_events[ Message::NUM_PERIODIC_MSG ];
  static MessageHandler             s_rx_handlers[ Message::NUM_SUPPORTED_MSG ];
  static NodeId                     s_this_node;


  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void dispatch_handler( const Chimera::CAN::BasicFrame &frame );

  static void rx_handle_ping( const Chimera::CAN::BasicFrame &frame );


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initRuntime()
  {
    /*-------------------------------------------------------------------------
    Grab a reference to the CAN bus driver. This will already be initialized.
    -------------------------------------------------------------------------*/
    s_can_driver = Chimera::CAN::getDriver( Chimera::CAN::Channel::CAN0 );
    RT_HARD_ASSERT( s_can_driver );

    /*-------------------------------------------------------------------------
    Pull the STM32 unique ID and assign a node to it. For now this is just for
    development purposes. Eventually node assignment will be handled by the
    flight controller and stored in NVM.
    -------------------------------------------------------------------------*/
    Chimera::System::Information *sys_info = nullptr;
    if ( Chimera::System::getSystemInformation( sys_info ); sys_info != nullptr )
    {
      uint32_t unique_id = 0;
      memcpy( &unique_id, sys_info->uniqueId.data(), sizeof( unique_id ) );

      switch ( unique_id )
      {
        case 0x0043002e:
          s_this_node = NodeId::NODE_0;
          break;

        default:
          s_this_node = NodeId::NUM_SUPPORTED_NODES;
          RT_HARD_ASSERT( false );
          break;
      }
    }

    /*-------------------------------------------------------------------------
    Initialize the controllers for periodic data
    -------------------------------------------------------------------------*/
    for ( uint8_t idx = Message::MSG_PERIODIC_START; idx < Message::MSG_PERIODIC_END; idx++ )
    {
      uint8_t offset = Message::periodicOffset( idx );
      RT_HARD_ASSERT( ( offset != Message::MSG_INVALID ) && ( offset < ARRAY_COUNT( s_polled_events ) ) );

      Chimera::Status_t init_result;
      switch ( idx )
      {
        case Message::MSG_SYSTEM_TICK:
          init_result = s_polled_events[ offset ].periodic( Chimera::Function::Opaque::create<periodicTXSystemTick>(),
                                                            Message::SystemTick::Period );
          break;

        default:
          LOG_ERROR( "Unhandled periodic msg init: %d\r\n", idx );
          RT_HARD_ASSERT( false );    // Missed case!
          break;
      };

      RT_HARD_ASSERT( init_result == Chimera::Status::OK );
    }

    /*-------------------------------------------------------------------------
    Reset the message handlers back to defaults
    -------------------------------------------------------------------------*/
    memset( s_rx_handlers, 0, sizeof( s_rx_handlers ) );

    s_rx_handlers[ Message::MSG_PING ] = rx_handle_ping;
  }


  bool setHandler( uint8_t msg_enum, MessageHandler handler )
  {
    /*-------------------------------------------------------------------------
    Input Protections
    -------------------------------------------------------------------------*/
    if( msg_enum >= Message::NUM_SUPPORTED_MSG )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Assign the handler
    -------------------------------------------------------------------------*/
    s_rx_handlers[ msg_enum ] = handler;
    return true;
  }


  NodeId thisNode()
  {
    return s_this_node;
  }


  void processCANBus()
  {
    /*-------------------------------------------------------------------------
    Input Protections
    -------------------------------------------------------------------------*/
    if ( !s_can_driver )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Pull any new messages off the queue and dispatch to their handlers
    -------------------------------------------------------------------------*/
    size_t available_frames = s_can_driver->available();
    while ( available_frames )
    {
      Chimera::CAN::BasicFrame frame;
      Chimera::Status_t        result = s_can_driver->receive( frame );
      if ( Chimera::Status::OK == result )
      {
        dispatch_handler( frame );
      }
      else
      {
        LOG_ERROR( "Failed CAN frame reception with error code: %d\r\n", result );
      }

      available_frames = s_can_driver->available();
    }

    /*-------------------------------------------------------------------------
    Process periodic transmit events
    -------------------------------------------------------------------------*/
    for ( auto &event : s_polled_events )
    {
      event.poll();
    }
  }


  void periodicTXSystemTick()
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !s_can_driver )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Fill in the payload
    -------------------------------------------------------------------------*/
    Message::SystemTick::PayloadType payload;
    payload.src.nodeId = thisNode();
    payload.tick       = Chimera::millis();

    /*-------------------------------------------------------------------------
    Build the message frame
    -------------------------------------------------------------------------*/
    auto msg       = defaultFrame();
    msg.id         = Message::SystemTick::CanID;
    msg.dataLength = sizeof( payload );
    memcpy( msg.data, &payload, sizeof( payload ) );

    /*-------------------------------------------------------------------------
    Ship it!
    -------------------------------------------------------------------------*/
    s_can_driver->send( msg );
  }


  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void dispatch_handler( const Chimera::CAN::BasicFrame &frame )
  {
    /*-------------------------------------------------------------------------
    Determine the index used to look up the handler
    -------------------------------------------------------------------------*/
    auto lookup_idx = Message::MSG_INVALID;
    switch( frame.id )
    {
      case Message::Ping::CanID:
        lookup_idx = Message::MSG_PING;
        break;

      case Message::SystemTick::CanID:
        lookup_idx = Message::MSG_SYSTEM_TICK;
        break;

      default:
        return;
    };

    /*-------------------------------------------------------------------------
    Invoke the handler
    -------------------------------------------------------------------------*/
    if( s_rx_handlers[ lookup_idx ] )
    {
      s_rx_handlers[ lookup_idx ]( frame );
    }
  }


  static void rx_handle_ping( const Chimera::CAN::BasicFrame &frame )
  {
    /*-------------------------------------------------------------------------
    Send the frame right back to the sender
    -------------------------------------------------------------------------*/
    Message::Ping::PayloadType data;
    Chimera::CAN::BasicFrame tx_frame;

    if( ( Chimera::Status::OK == unpack( frame, &data, sizeof( data ) ) ) && ( data.dst.nodeId == thisNode() ) )
    {
      data.dst.nodeId = data.src.nodeId;
      data.src.nodeId = thisNode();

      pack( Message::Ping::CanID, &data, sizeof( data ), tx_frame );
      s_can_driver->send( tx_frame );

      if( data.dst.nodeId == NodeId::NODE_PC )
      {
        LOG_DEBUG( "RX ping from %s\r\n", NODE_PC_STR );
      }
      else
      {
        LOG_DEBUG( "RX ping from node %d\r\n", data.dst.nodeId );
      }
    }
  }
}    // namespace Orbit::CAN
