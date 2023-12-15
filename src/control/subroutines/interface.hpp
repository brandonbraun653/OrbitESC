/******************************************************************************
 *  File Name:
 *    subroutines.hpp
 *
 *  Description:
 *    Main interface for the control subroutines
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_CONTROL_SUBROUTINES_HPP
#define ORBIT_ESC_CONTROL_SUBROUTINES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>


namespace Orbit::Control::Subroutine
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  enum class Routine : uint8_t
  {
    IDLE,
    ALIGNMENT_DETECTION,
    PARAMETER_ESTIMATOR,
    FORCE_ALIGNMENT,
    OPEN_LOOP_RAMP_FOC,
    CLOSED_LOOP_LOCK_FOC,
    CLOSED_LOOP_CTRL_FOC,
    FAULT_DETECTION,

    NUM_OPTIONS
  };

  /**
   * @brief Operational request to interact with the subroutines
   */
  enum class Request : uint8_t
  {
    START,
    STOP,
    DESTROY,
    INVALID_REQUEST
  };

  /**
   * @brief The various states a subroutine can be in
   */
  enum class State : uint8_t
  {
    UNINITIALIZED,  /**< Routine is idle and not ready */
    INITIALIZED,    /**< Init sequence has been performed */
    RUNNING,        /**< System is running */
    STOPPED,        /**< System has terminated */
    PANIC,          /**< System has encountered an error */
    INVALID_STATE
  };

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  /**
   * @brief Core interface all subroutines must implement.
   *
   * This provides the control surface for the main control loop to interact
   * with the subroutines and dynamically load/unload them as needed.
   */
  class ISubroutine
  {
  public:
    virtual ~ISubroutine() = default;

    Routine id; /**< Which routine this is */

    /**
     * @brief Prepare the subroutine for use
     * @return void
     */
    virtual void initialize() = 0;

    /**
     * @brief Perform any startup tasks to transition to the run state.
     *
     * Once the system has finished starting, it's expected it will report a
     * new state of RUNNING or PANIC.
     *
     * @return void
     */
    virtual void start() = 0;

    /**
     * @brief Perform any shutdown tasks to transition to the stopped state.
     *
     * Once the system has finished stopping, it's expected it will report a
     * new state of STOPPED or PANIC.
     *
     * @return void
     */
    virtual void stop() = 0;

    /**
     * @brief Tear down resources and relinquish all control.
     *
     * Once the system has finished destroying, it's expected it will report a
     * new state of UNINITIALIZED.
     *
     * @return void
     */
    virtual void destroy() = 0;

    /**
     * @brief Periodic processing to execute the main subroutine logic.
     *
     * This is the main entry point for the subroutine to execute its logic. The only
     * way to exit this state is to indicate STOPPED or PANIC.
     *
     * @return void
     */
    virtual void process() = 0;

    /**
     * @brief Perform any cleanup tasks to transition to the uninitialized state.
     * @return void
     */
    virtual State state() = 0;

  protected:
    State mState; /**< Current state of the routine */
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize the subroutine driver
   * @return void
   */
  void initialize();

  /**
   * @brief Maps a subroutine instance to a particular routine ID
   *
   * @param routine Which routine to map against
   * @param pimpl   Implementation to bind
   * @return void
   */
  void bind( const Routine routine, ISubroutine *const pimpl );

  /**
   * @brief Process the currently registered subroutine
   * @return void
   */
  void process();

  /**
   * @brief Gets the currently executing routine
   * @return Routine
   */
  Routine currentRoutine();

  /**
   * @brief Modify the currently executing routine's state
   *
   * @param request The request to send to the controller
   * @return bool True if modified, false otherwise
   */
  bool modifyState( const Request request );

  /**
   * @brief Switch to the next routine to be executed
   *
   * @param next  The next routine to execute
   * @return bool True if accepted, false otherwise
   */
  bool switchRoutine( const Routine next );

}    // namespace Orbit::Control::Subroutine

#endif /* !ORBIT_ESC_CONTROL_SUBROUTINES_HPP */
