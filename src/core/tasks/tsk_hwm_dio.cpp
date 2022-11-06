/******************************************************************************
 *  File Name:
 *    tsk_hwm_dio.cpp
 *
 *  Description:
 *    HWM delayed IO task
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/thread>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_hwm_dio.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_log_io.hpp>

namespace Orbit::Tasks::DIO
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void DIOThread( void *arg )
  {
    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Initialize data controllers
    -------------------------------------------------------------------------*/
    Orbit::Data::bootFileSystem();
    Log::initialize();
    Log::enable();

    // testing
    size_t last_assert = Chimera::millis();
    size_t last_dump = Chimera::millis();

    /*-------------------------------------------------------------------------
    Run the HWM thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Do long-running operations
      -----------------------------------------------------------------------*/
      Log::flushCache();

      /*-----------------------------------------------------------------------
      TESTING!
      -----------------------------------------------------------------------*/
      if( wake_up_tick >= ( last_assert + Chimera::Thread::TIMEOUT_1S ) )
      {
        last_assert = wake_up_tick;
        LOG_ERROR( "Test file logger %d\r\n", wake_up_tick );
      }

      if( wake_up_tick >= ( last_dump + ( 10 * Chimera::Thread::TIMEOUT_1S ) ) )
      {
        last_dump = wake_up_tick;
        Log::dumpToConsole();
      }

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::DIO
