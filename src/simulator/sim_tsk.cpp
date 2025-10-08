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
#include <src/simulator/sim_tcp_server.hpp>

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

      Orbit::Sim::Matlab::RxMessage rx_message;
      if( Orbit::Sim::Matlab::getLastMessage( rx_message ) )
      {
        Orbit::Sim::Matlab::TxMessage tx_message{};
        tx_message.value = rx_message.value;
        Orbit::Sim::Matlab::pushMessage( tx_message );
      }

      Chimera::delayMilliseconds( PERIOD_MS );
    }
  }
}    // namespace Orbit::Tasks::SIM
