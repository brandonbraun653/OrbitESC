/******************************************************************************
 *  File Name:
 *    sys_mode_base.cpp
 *
 *  Description:
 *    Root class for all system modes
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <string_view>
#include <Aurora/logging>
#include <Chimera/assert>
#include <src/control/modes/sys_mode_base.hpp>

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static constexpr std::array<std::string_view, EventId::NUM_EVENTS> s_event_names = {
    /* clang-format off */
    "EMERGENCY_HALT",
    "ARM",
    "DISARM",
    "ALIGN",
    "RAMP",
    "RUN",
    "DISENGAGE",
    "FAULT"
    /* clang-format on */
  };

  static_assert( s_event_names[ EventId::EMERGENCY_HALT ] == "EMERGENCY_HALT", "Event name mismatch" );
  static_assert( s_event_names[ EventId::ARM ] == "ARM", "Event name mismatch" );
  static_assert( s_event_names[ EventId::DISARM ] == "DISARM", "Event name mismatch" );
  static_assert( s_event_names[ EventId::ALIGN ] == "ALIGN", "Event name mismatch" );
  static_assert( s_event_names[ EventId::RAMP ] == "RAMP", "Event name mismatch" );
  static_assert( s_event_names[ EventId::RUN ] == "RUN", "Event name mismatch" );
  static_assert( s_event_names[ EventId::DISENGAGE ] == "DISENGAGE", "Event name mismatch" );
  static_assert( s_event_names[ EventId::FAULT ] == "FAULT", "Event name mismatch" );

  static constexpr std::array<std::string_view, StateId::NUM_STATES> s_state_names = {
    /* clang-format off */
    "IDLE",
    "ARMED",
    "PARK",
    "RAMP",
    "RUN",
    "FAULT"
    /* clang-format on */
  };

  static_assert( s_state_names[ StateId::IDLE ] == "IDLE", "State name mismatch" );
  static_assert( s_state_names[ StateId::ARMED ] == "ARMED", "State name mismatch" );
  static_assert( s_state_names[ StateId::ENGAGED_PARK ] == "PARK", "State name mismatch" );
  static_assert( s_state_names[ StateId::ENGAGED_RAMP ] == "RAMP", "State name mismatch" );
  static_assert( s_state_names[ StateId::ENGAGED_RUN ] == "RUN", "State name mismatch" );
  static_assert( s_state_names[ StateId::FAULT ] == "FAULT", "State name mismatch" );

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  const std::string_view &getMessageString( const etl::message_id_t event )
  {
    RT_DBG_ASSERT( event < s_event_names.size() );
    return s_event_names[ event ];
  }


  const std::string_view &getStateString( const etl::fsm_state_id_t state )
  {
    RT_DBG_ASSERT( state < s_state_names.size() );
    return s_state_names[ state ];
  }

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  void FSMMotorControl::attachControllerData( SuperState *const state )
  {
    RT_DBG_ASSERT( state != nullptr );
    mState = state;
  }


  void FSMMotorControl::logUnhandledMessage( const etl::imessage &msg )
  {
    LOG_WARN( "%s message not handled from state %s\r\n", getMessageString( msg.get_message_id() ),
              getStateString( get_state_id() ) );
  }

}    // namespace Orbit::Control