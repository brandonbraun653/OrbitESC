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
    Data::bootFileSystem();
    Log::initialize();
    Log::enable();

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
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::DIO
