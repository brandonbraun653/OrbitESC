/******************************************************************************
 *  File Name:
 *    tsk_dio.cpp
 *
 *  Description:
 *    HWM delayed IO task
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/thread>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_data_storage.hpp>
#include <src/core/data/orbit_log_io.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_dio.hpp>

namespace Orbit::Tasks::DIO
{
  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static void incrementBootCount()
  {


  }


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
    Orbit::Data::printSystemInfo();
    Log::initialize();
    Log::enable();

    /*-------------------------------------------------------------------------
    Run the HWM thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    size_t next_sync    = wake_up_tick + Data::SysConfig.disk_update_period;
    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Do long-running operations
      -----------------------------------------------------------------------*/
      Log::flushCache();

      if( wake_up_tick >= next_sync )
      {
        next_sync = wake_up_tick + Data::SysConfig.disk_update_period;
        Data::syncDisk();
      }

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::DIO
