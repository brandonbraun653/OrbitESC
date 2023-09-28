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
#include <src/core/bootup.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_log_io.hpp>
#include <src/core/data/volatile/orbit_parameter.hpp>
#include <src/core/runtime/serial_runtime.hpp>
#include <src/core/tasks.hpp>
#include <src/core/tasks/tsk_dio.hpp>

namespace Orbit::Tasks::DIO
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void DIOThread( void *arg )
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Power up the hardware drivers and spawn remaining tasks
    -------------------------------------------------------------------------*/
    Boot::powerUpSystemDrivers();
    Boot::startTasks();

    /*-------------------------------------------------------------------------
    Finalize the power up sequence
    -------------------------------------------------------------------------*/
    Data::SysInfo.bootCount++;
    Data::Param::write( ParamId_PARAM_BOOT_COUNT, &Data::SysInfo.bootCount, sizeof( Data::SysInfo.bootCount ) );

    /*-------------------------------------------------------------------------
    Run the delayed-io thread
    -------------------------------------------------------------------------*/
    size_t wake_up_tick = Chimera::millis();
    size_t next_sync    = wake_up_tick + Data::SysConfig.diskUpdateRateMs;
    TaskMsg tsk_msg     = TASK_MSG_NUM_OPTIONS;
    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Process any task messages that may have arrived
      -----------------------------------------------------------------------*/
      if ( this_thread::receiveTaskMsg( tsk_msg, TIMEOUT_DONT_WAIT ) )
      {
        switch( tsk_msg )
        {
          case TASK_MSG_PARAM_IO_EVENT:
            Serial::handleParamIOEvent();
            break;

          default:
            break;
        }
      }

      /*-----------------------------------------------------------------------
      Push any pending log messages
      -----------------------------------------------------------------------*/
      Log::flushCache();

      /*-----------------------------------------------------------------------
      Synchronize any updates to the configuration backing store
      -----------------------------------------------------------------------*/
      if( wake_up_tick >= next_sync )
      {
        next_sync = wake_up_tick + Data::SysConfig.diskUpdateRateMs;
        Data::Param::flush();
      }

      /*-----------------------------------------------------------------------
      Pseudo attempt to run this task periodically
      -----------------------------------------------------------------------*/
      Chimera::delayUntil( wake_up_tick + PERIOD_MS );
      wake_up_tick = Chimera::millis();
    }
  }
}  // namespace Orbit::Tasks::DIO
