/********************************************************************************
 *  File Name:
 *    startup.cpp
 *
 *  Description:
 *    OrbitESC firmware entry point
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/thread>
#include <src/core/tasks.hpp>

#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
#include "SEGGER_SYSVIEW.h"

/*-------------------------------------------------------------------------------
Disable SysView events. Cortex-M4 + JLink can only handle about 5k events/sec
before overflows start occurring. The mask disables enough items to get the event
rate to ~2.5k/s while gathering data to rebuild the task switching structure. This
should leave enough room for user profiling/debugging in the future.

Reasoning for Disabled:
SYSVIEW_EVTMASK_ALL_INTERRUPTS    | Project uses a large number of ISRs
SYSVIEW_EVTMASK_SYSTIME_CYCLES    | Irrelevant to build system timing info
SYSVIEW_EVTMASK_SYSTIME_US        | Irrelevant to build system timing info
SYSVIEW_EVTMASK_TIMER_ENTER       | Irrelevant to build system timing info
SYSVIEW_EVTMASK_TIMER_EXIT        | Irrelevant to build system timing info
SYSVIEW_EVTMASK_TASK_STOP_EXEC    | SysView can re-build task sequences without this
SYSVIEW_EVTMASK_TASK_START_READY  | SysView can re-build task sequences without this
SYSVIEW_EVTMASK_TASK_STOP_READY   | SysView can re-build task sequences without this
-------------------------------------------------------------------------------*/
/* clang-format off */
#define SEGGER_DISABLE_MASK             \
  ( SYSVIEW_EVTMASK_ALL_INTERRUPTS    | \
    SYSVIEW_EVTMASK_SYSTIME_CYCLES    | \
    SYSVIEW_EVTMASK_SYSTIME_US        | \
    SYSVIEW_EVTMASK_TIMER_ENTER       | \
    SYSVIEW_EVTMASK_TIMER_EXIT        | \
    SYSVIEW_EVTMASK_TASK_STOP_EXEC    | \
    SYSVIEW_EVTMASK_TASK_START_READY  | \
    SYSVIEW_EVTMASK_TASK_STOP_READY )
/* clang-format on */
#endif

/*-----------------------------------------------------------------------------
Public Functions
-----------------------------------------------------------------------------*/
int main()
{
  /*---------------------------------------------------------------------------
  Perform peripheral driver system initialization
  ---------------------------------------------------------------------------*/
  ChimeraInit();

  /*---------------------------------------------------------------------------
  Project Level Task Initialization
  ---------------------------------------------------------------------------*/
  Orbit::Tasks::initialize();

  /*---------------------------------------------------------------------------
  Initialize the SystemView driver
  ---------------------------------------------------------------------------*/
#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_DisableEvents( SEGGER_DISABLE_MASK );
#endif

  /*---------------------------------------------------------------------------
  Start the system threads. Never returns.
  ---------------------------------------------------------------------------*/
  Chimera::Thread::startScheduler();
  return 0;
}
