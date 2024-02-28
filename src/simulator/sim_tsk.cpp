/******************************************************************************
 *  File Name:
 *    sim_tsk.cpp
 *
 *  Description:
 *    Simulator thread for the test harness and simulation environment
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/thread>
#include <src/core/tasks.hpp>
#include <src/simulator/sim_tsk.hpp>
#include <src/simulator/sim_adc.hpp>

namespace Orbit::Tasks::SIM
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void SIMThread( void *arg )
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    while( 1 )
    {
      /*-----------------------------------------------------------------------
      Run simulation events
      -----------------------------------------------------------------------*/
      Orbit::Sim::ADC::triggerInstrumentationADC();


      Chimera::delayMilliseconds( PERIOD_MS );
    }
  }
}    // namespace Orbit::Tasks::USB::CDC
