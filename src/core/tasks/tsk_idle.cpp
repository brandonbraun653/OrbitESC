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
#include <src/core/hw/orbit_led.hpp>
#include <src/core/data/orbit_data.hpp>


namespace Orbit::Tasks::BKD
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
    for ( auto x = 0; x < 8; x++ )
    {
      LED::toggleChannel( LED::Channel::HEARTBEAT );
      LED::sendUpdate();
      Chimera::delayMilliseconds( 35 );
    }
    Chimera::delayMilliseconds( 500 );

    /*-------------------------------------------------------------------------
    Main loop
    -------------------------------------------------------------------------*/
    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Compute the flash and hold times
      -----------------------------------------------------------------------*/
      const uint32_t flash_delay = static_cast<uint32_t>( 100.0f * Data::SysConfig.activityLedScaler );
      const uint32_t hold_delay  = static_cast<uint32_t>( 450.0f * Data::SysConfig.activityLedScaler );

      /*-----------------------------------------------------------------------
      High Pulse #1
      -----------------------------------------------------------------------*/
      LED::setChannel( LED::Channel::HEARTBEAT );
      Chimera::delayMilliseconds( flash_delay );
      LED::clrChannel( LED::Channel::HEARTBEAT );
      Chimera::delayMilliseconds( flash_delay );

      /*-----------------------------------------------------------------------
      High Pulse #2
      -----------------------------------------------------------------------*/
      LED::setChannel( LED::Channel::HEARTBEAT );
      Chimera::delayMilliseconds( flash_delay );
      LED::clrChannel( LED::Channel::HEARTBEAT );
      Chimera::delayMilliseconds( flash_delay );

      /*-----------------------------------------------------------------------
      Hold longer in the off state
      -----------------------------------------------------------------------*/
      Chimera::delayMilliseconds( hold_delay );
    }
  }
}    // namespace Orbit::Tasks::BKD
