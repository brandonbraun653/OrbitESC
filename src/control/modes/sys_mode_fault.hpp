/******************************************************************************
 *  File Name:
 *    sys_mode_fault.hpp
 *
 *  Description:
 *    Fault mode
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_CONTROL_FAULT_MODE_HPP
#define ORBIT_CONTROL_FAULT_MODE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/foc_driver.hpp>
#include <src/control/modes/sys_mode_base.hpp>

namespace Orbit::Control::State
{
  /*---------------------------------------------------------------------------
  State Class
  ---------------------------------------------------------------------------*/
  class Fault : public etl::fsm_state<FOC::StateMachine, Fault, ModeId::FAULT, MsgDisable, MsgFault>
  {
  public:
    void on_exit_state() final override;
    etl::fsm_state_id_t on_enter_state() final override;
    etl::fsm_state_id_t on_event( const MsgDisable & );
    etl::fsm_state_id_t on_event( const MsgFault & );
    etl::fsm_state_id_t on_event_unknown( const etl::imessage & );
  };

}    // namespace Orbit::Control::State

#endif /* !ORBIT_CONTROL_FAULT_MODE_HPP */
