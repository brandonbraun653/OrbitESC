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
#include <src/config/bsp/board_map.hpp>
#include <src/core/com/can_message.hpp>
#include <src/core/com/can_router.hpp>
#include <src/core/com/can_server.hpp>
#include <src/core/runtime/can_runtime.hpp>


namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static const char *NODE_PC_STR = "PC";

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static NodeId s_this_node;
  static Server s_can_server;

  /*---------------------------------------------------------------------------
  Periodic Message Declarations
  ---------------------------------------------------------------------------*/
  static Message::SystemTick         s_msg_system_tick;
  static Message::PowerSupplyVoltage s_msg_power_supply_voltage;
  static Message::PhaseACurrent      s_msg_phase_a_current;
  static Message::PhaseBCurrent      s_msg_phase_b_current;

  /*---------------------------------------------------------------------------
  Router Declarations
  ---------------------------------------------------------------------------*/
  static Router::PingRouter           s_ping_router;
  static Router::ArmDisarmMotorRouter s_arm_disarm_motor_router;
  static Router::SetMotorSpeedRouter  s_set_motor_speed_router;
  static Router::GetMotorSpeedRouter  s_get_motor_speed_router;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initRuntime()
  {
    /*-------------------------------------------------------------------------
    Initialize the CAN bus dispatch server
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( s_can_server.initialize( Orbit::IO::CAN::channel ) == Chimera::Status::OK );

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
        case 0x0043002E:
        case 0x005A002D:
          s_this_node = NodeId::NODE_0;
          break;

        default:
          s_this_node = NodeId::NUM_SUPPORTED_NODES;
          RT_HARD_ASSERT( false );
          break;
      }
    }

    /*-------------------------------------------------------------------------
    Register update functions for periodic messages
    -------------------------------------------------------------------------*/
    /* System Tick */
    auto sys_tick_func =
        Chimera::Function::Opaque::create<Message::SystemTick, &Message::SystemTick::update>( s_msg_system_tick );
    s_can_server.registerPeriodic( sys_tick_func, s_msg_system_tick.period() );

    /* Power Supply Voltage */
    auto pwr_supply_func = Chimera::Function::Opaque::create<Message::PowerSupplyVoltage, &Message::PowerSupplyVoltage::update>(
        s_msg_power_supply_voltage );
    s_can_server.registerPeriodic( pwr_supply_func, s_msg_power_supply_voltage.period() );

    /* Phase A Current */
    auto phase_a_current_func =
        Chimera::Function::Opaque::create<Message::PhaseACurrent, &Message::PhaseACurrent::update>( s_msg_phase_a_current );
    s_can_server.registerPeriodic( phase_a_current_func, s_msg_phase_a_current.period() );

    /* Phase B Current */
    auto phase_b_current_func =
        Chimera::Function::Opaque::create<Message::PhaseBCurrent, &Message::PhaseBCurrent::update>( s_msg_phase_b_current );
    s_can_server.registerPeriodic( phase_b_current_func, s_msg_phase_b_current.period() );

    /*-------------------------------------------------------------------------
    Register the routers
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( s_can_server.subscribe( s_ping_router ) );
    RT_HARD_ASSERT( s_can_server.subscribe( s_arm_disarm_motor_router ) );
    RT_HARD_ASSERT( s_can_server.subscribe( s_set_motor_speed_router ) );
    RT_HARD_ASSERT( s_can_server.subscribe( s_get_motor_speed_router ) );
  }


  NodeId thisNode()
  {
    return s_this_node;
  }


  void processCANBus()
  {
    s_can_server.processRTX();
  }

}    // namespace Orbit::CAN
