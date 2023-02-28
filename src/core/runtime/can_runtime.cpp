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
#include <src/core/com/can/can_async_message.hpp>
#include <src/core/com/can/can_message.hpp>
#include <src/core/com/can/can_periodic_message.hpp>
#include <src/core/com/can/can_router.hpp>
#include <src/core/com/can/can_server.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/runtime/can_runtime.hpp>


namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  Server MessageServer;  /**< Driver for the board's CAN message server */

  /*---------------------------------------------------------------------------
  Periodic Message Declarations
  ---------------------------------------------------------------------------*/
  static Message::SystemTick         s_msg_system_tick;
  static Message::SystemMode         s_msg_system_mode;
  static Message::PowerSupplyVoltage s_msg_power_supply_voltage;
  static Message::PhaseACurrent      s_msg_phase_a_current;
  static Message::PhaseBCurrent      s_msg_phase_b_current;
  static Message::PhaseCCurrent      s_msg_phase_c_current;
  static Message::MotorSpeed         s_msg_motor_speed;
  static Message::SpeedReference     s_msg_speed_reference;

  /*---------------------------------------------------------------------------
  Router Declarations
  ---------------------------------------------------------------------------*/
  static Router::PingRouter           s_ping_router;
  static Router::SetSystemModeRouter  s_set_system_mode_router;
  static Router::SetMotorSpeedRouter  s_set_motor_speed_router;
  static Router::EmergencyHaltRouter  s_emergency_halt_router;
  static Router::SystemResetRouter    s_system_reset_router;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initRuntime()
  {
    /*-------------------------------------------------------------------------
    Initialize the CAN bus dispatch server
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( MessageServer.initialize( Orbit::IO::CAN::channel ) == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    Register update functions for periodic messages
    -------------------------------------------------------------------------*/
    /* System Tick */
    auto sys_tick_func =
        Chimera::Function::Opaque::create<Message::SystemTick, &Message::SystemTick::update>( s_msg_system_tick );
    MessageServer.registerPeriodic( sys_tick_func, s_msg_system_tick.period() );

    /* System Mode */
    auto sys_mode_func =
        Chimera::Function::Opaque::create<Message::SystemMode, &Message::SystemMode::update>( s_msg_system_mode );
    MessageServer.registerPeriodic( sys_mode_func, s_msg_system_mode.period() );

    /* Power Supply Voltage */
    auto pwr_supply_func = Chimera::Function::Opaque::create<Message::PowerSupplyVoltage, &Message::PowerSupplyVoltage::update>(
        s_msg_power_supply_voltage );
    MessageServer.registerPeriodic( pwr_supply_func, s_msg_power_supply_voltage.period() );

    /* Phase A Current */
    auto phase_a_current_func =
        Chimera::Function::Opaque::create<Message::PhaseACurrent, &Message::PhaseACurrent::update>( s_msg_phase_a_current );
    MessageServer.registerPeriodic( phase_a_current_func, s_msg_phase_a_current.period() );

    /* Phase B Current */
    auto phase_b_current_func =
        Chimera::Function::Opaque::create<Message::PhaseBCurrent, &Message::PhaseBCurrent::update>( s_msg_phase_b_current );
    MessageServer.registerPeriodic( phase_b_current_func, s_msg_phase_b_current.period() );

    /* Phase C Current */
    auto phase_c_current_func =
        Chimera::Function::Opaque::create<Message::PhaseCCurrent, &Message::PhaseCCurrent::update>( s_msg_phase_c_current );
    MessageServer.registerPeriodic( phase_c_current_func, s_msg_phase_c_current.period() );

    /* Motor Speed */
    auto motor_speed_func =
        Chimera::Function::Opaque::create<Message::MotorSpeed, &Message::MotorSpeed::update>( s_msg_motor_speed );
    MessageServer.registerPeriodic( motor_speed_func, s_msg_motor_speed.period() );

    /* Speed Reference */
    auto speed_ref_func =
        Chimera::Function::Opaque::create<Message::SpeedReference, &Message::SpeedReference::update>( s_msg_speed_reference );
    MessageServer.registerPeriodic( speed_ref_func, s_msg_speed_reference.period() );

    /*-------------------------------------------------------------------------
    Register the routers for incoming messages
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( MessageServer.subscribe( s_ping_router ) );
    RT_HARD_ASSERT( MessageServer.subscribe( s_set_system_mode_router ) );
    RT_HARD_ASSERT( MessageServer.subscribe( s_set_motor_speed_router ) );
    RT_HARD_ASSERT( MessageServer.subscribe( s_emergency_halt_router ) );
    RT_HARD_ASSERT( MessageServer.subscribe( s_system_reset_router ) );
  }


  NodeId thisNode()
  {
    return Data::SysConfig.canNodeId;
  }


  void processCANBus()
  {
    MessageServer.processRTX();
  }

}    // namespace Orbit::CAN
