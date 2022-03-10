/******************************************************************************
 *  File Name:
 *    adc_runtime.cpp
 *
 *  Description:
 *    Runtime for processing the ADC
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/adc>
#include <src/core/runtime/adc_runtime.hpp>
#include <src/config/bsp/board_map.hpp>

namespace Orbit::ADC
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::ADC::Driver_rPtr s_adc_driver;


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initRuntime()
  {
    /*-------------------------------------------------------------------------
    Grab a reference to the ADC driver
    -------------------------------------------------------------------------*/
    s_adc_driver = Chimera::ADC::getDriver( Chimera::ADC::Peripheral::ADC_0 );
    RT_HARD_ASSERT( s_adc_driver );
  }


  void processADC()
  {
    /*-------------------------------------------------------------------------
    Input Protections
    -------------------------------------------------------------------------*/
    if( !s_adc_driver )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Pull the latest data off the bus
    -------------------------------------------------------------------------*/
    Chimera::ADC::Sample raw_sample;

    if( s_adc_driver->nextSample( IO::Analog::adcPhaseA, raw_sample ) )
    {
      LOG_INFO( "Phase A: %d\r\n", raw_sample.counts );
    }
  }

}  // namespace Orbit::ADC
