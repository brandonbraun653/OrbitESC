/******************************************************************************
 *  File Name:
 *    foc_driver.hpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Driver
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_FOC_CONTROL_HPP
#define ORBIT_ESC_FOC_CONTROL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/fsm.h>
#include <etl/message.h>
#include <src/control/foc_data.hpp>
#include <src/control/modes/sys_mode_base.hpp>


namespace Orbit::Control::FOC
{
  /*---------------------------------------------------------------------------
  Public Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief State machine for the FOC controller.
   *
   * This class forms the shared context for all of the state controllers. It
   * is responsible for managing the state transitions and dispatching messages.
   *
   * All state controllers use CRTP to inherit from this class. This allows the
   * state controllers to access the shared context and handle messages.
   */
  class StateMachine : public etl::fsm
  {
  public:
    StateMachine();
    void logUnhandledMessage( const etl::imessage &msg );

  protected:
    etl::fsm_state_id_t dst_state; /* Destination state of the FSM */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Power up the FOC library
   *
   * @return void
   */
  void initialize();

  /**
   * @brief Executes the high level controller
   * @note Expects to be run periodically
   */
  void process();

  /**
   * @brief Injects an event to the operational mode of the controller
   *
   * @param event System event being sent
   * @return int  Zero if OK, negative on error
   */
  int sendSystemEvent( const EventId_t event );

  /**
   * @brief Set a new speed reference for the motor
   * @note Requires arm or engage mode
   *
   * Assigns a new set-point for the motor control system. Only takes physical effect on
   * the motor once the system is engaged.
   *
   * @param ref        New speed reference in rpm
   * @return int
   */
  int setSpeedRef( const float ref );

  /**
   * @brief Gets a view of the internal state of the FOC driver
   *
   * @return const SuperState&
   */
  const SuperState &dbgGetState();

  /**
   * @brief Gets the current system mode of the FOC driver
   * @return ModeId_t
   */
  ModeId_t currentMode();

  /**
   * @brief Drive a test signal on the power stage of the ESC
   * @note Must be in the Armed state for this to work
   *
   * @param commCycle   Commutation cycle to execute
   * @param dutyCycle   Duty cycle to drive the output
   */
  void driveTestSignal( const uint8_t commCycle, const float dutyCycle );

}    // namespace Orbit::Control::FOC

#endif /* !ORBIT_ESC_FOC_CONTROL_HPP */
