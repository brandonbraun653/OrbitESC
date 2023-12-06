/******************************************************************************
 *  File Name:
 *    tsk_idle.cpp
 *
 *  Description:
 *    Idle task implementation
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/assert>
#include <Chimera/common>
#include <Chimera/function>
#include <Chimera/gpio>
#include <Chimera/thread>
#include <src/config/bsp/board_map.hpp>
#include <src/core/com/com_scheduler.hpp>
#include <src/core/hw/orbit_led.hpp>
#include <src/core/hw/orbit_timer.hpp>
#include <src/core/tasks.hpp>


namespace Orbit::Tasks::BKD
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Main thread to run background tasks whenever excessive CPU is available.
   *
   * @param arg   Unused
   * @return void
   */
  void IdleThread( void *arg )
  {
    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Get the status/heartbeat pin and flash a quick boot up sequence
    -------------------------------------------------------------------------*/
    for( auto x = 0; x < 8; x++ )
    {
      LED::toggleChannel( LED::Channel::ALL );
      Chimera::delayMilliseconds( 35 );
    }

    LED::clearChannel( LED::Channel::ALL );
    Chimera::delayMilliseconds( 500 );

    /*-------------------------------------------------------------------------
    Main loop. Don't add any blocking calls here. It's expected that this
    will run as fast as possible and consume all available CPU time.
    -------------------------------------------------------------------------*/
    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Process the LED state machine
      -----------------------------------------------------------------------*/
      LED::process();

      /*-----------------------------------------------------------------------
      Publish available data to the remote host
      -----------------------------------------------------------------------*/
      Orbit::COM::Scheduler::process();
    }
  }
}    // namespace Orbit::Tasks::BKD
