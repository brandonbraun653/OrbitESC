/******************************************************************************
 *  File Name:
 *    tsk_idle.cpp
 *
 *  Description:
 *    Idle task implementation
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/assert>
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/thread>
#include <src/config/bsp/board_map.hpp>
#include <src/core/bootup.hpp>
#include <src/core/hw/orbit_timer.hpp>


namespace Orbit::Tasks::Idle
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void IdleThread( void *arg )
  {
    /*-------------------------------------------------------------------------
    Power up the hardware drivers, then kick off the system tasks
    -------------------------------------------------------------------------*/
    Boot::powerUpSystemDrivers();
    Boot::startTasks();

    /*-------------------------------------------------------------------------
    Get the status/heartbeat pin and flash a quick boot up sequence
    -------------------------------------------------------------------------*/
    LOG_INFO( "Powering up OrbitESC\r\n" );
    auto pin = Chimera::GPIO::getDriver( IO::GPIO::portHeartbeat, IO::GPIO::pinHeartbeat );
    for ( auto x = 0; x < 8; x++ )
    {
      pin->toggle();
      Chimera::delayMilliseconds( 35 );
    }
    Chimera::delayMilliseconds( 500 );

    /*-------------------------------------------------------------------------
    Main loop
    -------------------------------------------------------------------------*/
    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      High Pulse #1
      -----------------------------------------------------------------------*/
      pin->setState( Chimera::GPIO::State::HIGH );
      Chimera::delayMilliseconds( 100 );
      pin->setState( Chimera::GPIO::State::LOW );
      Chimera::delayMilliseconds( 100 );

      /*-----------------------------------------------------------------------
      High Pulse #2
      -----------------------------------------------------------------------*/
      pin->setState( Chimera::GPIO::State::HIGH );
      Chimera::delayMilliseconds( 100 );
      pin->setState( Chimera::GPIO::State::LOW );
      Chimera::delayMilliseconds( 100 );

      /*-----------------------------------------------------------------------
      Hold longer in the off state
      -----------------------------------------------------------------------*/
      Chimera::delayMilliseconds( 450 );
    }
  }
}    // namespace Orbit::Tasks::Idle
