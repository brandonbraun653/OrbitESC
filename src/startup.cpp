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

#if defined( EMBEDDED )
#include <Thor/lld/common/cortex-m4/debug.hpp>

#if defined( SEGGER_SYS_VIEW )
#include <src/config/segger/orbit_segger_cfg.hpp>
#endif /* SEGGER_SYS_VIEW */

#endif  /* EMBEDDED */


/*-----------------------------------------------------------------------------
Public Functions
-----------------------------------------------------------------------------*/

/**
 * @brief Main entry point to the application.
 *
 * This isn't the first function called, but it is the first one that the
 * programmer has control over. The system will call the Reset_Handler first,
 * which will then call this function.
 *
 * @return int  Never returns
 */
int main()
{
  /*---------------------------------------------------------------------------
  Enable the debug trace counter
  ---------------------------------------------------------------------------*/
#if defined( EMBEDDED )
  CortexM4::Debug::enableCounter();
#endif

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
