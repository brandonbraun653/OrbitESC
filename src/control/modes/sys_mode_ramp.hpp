/******************************************************************************
 *  File Name:
 *    sys_mode_ramp.hpp
 *
 *  Description:
 *    Ramp mode
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CONTROL_RAMP_MODE_HPP
#define ORBIT_CONTROL_RAMP_MODE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/modes/sys_mode_base.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  class EngagedRamp : public etl::fsm_state<FSMMotorControl, EngagedRamp, StateId::ENGAGED_RAMP, MsgEmergencyHalt, MsgDisengage,
                                            MsgRun, MsgFault>
  {
  public:
    etl::fsm_state_id_t on_enter_state();
    etl::fsm_state_id_t on_event( const MsgEmergencyHalt & );
    etl::fsm_state_id_t on_event( const MsgDisengage & );
    etl::fsm_state_id_t on_event( const MsgRun & );
    etl::fsm_state_id_t on_event( const MsgFault & );
    etl::fsm_state_id_t on_event_unknown( const etl::imessage & );
  };
}    // namespace Orbit::Control::State

#endif /* !ORBIT_CONTROL_RAMP_MODE_HPP */