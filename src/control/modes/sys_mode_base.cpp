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
    "ENGAGE",
    "DISENGAGE",
    "FAULT"
    /* clang-format on */
  };

  static_assert( s_event_names[ EventId::EMERGENCY_HALT ] == "EMERGENCY_HALT", "Event name mismatch" );
  static_assert( s_event_names[ EventId::ARM ] == "ARM", "Event name mismatch" );
  static_assert( s_event_names[ EventId::DISARM ] == "DISARM", "Event name mismatch" );
  static_assert( s_event_names[ EventId::ENGAGE ] == "ENGAGE", "Event name mismatch" );
  static_assert( s_event_names[ EventId::DISENGAGE ] == "DISENGAGE", "Event name mismatch" );
  static_assert( s_event_names[ EventId::FAULT ] == "FAULT", "Event name mismatch" );

  static constexpr std::array<std::string_view, ModeId::NUM_STATES> s_state_names = {
    /* clang-format off */
    "IDLE",
    "ARMED",
    "ENGAGED",
    "FAULT"
    /* clang-format on */
  };

  static_assert( s_state_names[ ModeId::IDLE ] == "IDLE", "State name mismatch" );
  static_assert( s_state_names[ ModeId::ARMED ] == "ARMED", "State name mismatch" );
  static_assert( s_state_names[ ModeId::ENGAGED ] == "ENGAGED", "State name mismatch" );
  static_assert( s_state_names[ ModeId::FAULT ] == "FAULT", "State name mismatch" );

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  const std::string_view &getMessageString( const EventId_t event )
  {
    RT_DBG_ASSERT( event < s_event_names.size() );
    return s_event_names[ event ];
  }


  const std::string_view &getModeString( const ModeId_t mode )
  {
    RT_DBG_ASSERT( mode < s_state_names.size() );
    return s_state_names[ mode ];
  }

}    // namespace Orbit::Control
