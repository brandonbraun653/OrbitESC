/******************************************************************************
 *  File Name:
 *    foc_driver.cpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <src/control/foc_driver.hpp>
#include <src/config/bsp/board_map.hpp>

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  FOC::FOC()
  {
    memset( &mPrvState, 0, sizeof( mPrvState ) );
  }

  FOC::~FOC()
  {
    mTimerDriver.emergencyBreak();
  }


  int FOC::initialize( const FOCConfig &cfg )
  {
    /*
    Assumes a few things are set up:
      - ADC/Timer/DMA are configured to always run on debugger connection
      - HW fully initialized
      - Interrupt priorities are set properly

    Things To Do:
      - Use local memory for the ADC DMA transfer. Don't want to look up from a queue in an ISR.
      - Set up ISRs for execution with DMA EOT and periodic Timer events.
    */

    /*-------------------------------------------------------------------------
    Link the ADC's DMA end-of-transfer interrupt to this class's ISR handler
    -------------------------------------------------------------------------*/
    Chimera::ADC::ISRCallback callback = Chimera::ADC::ISRCallback::create<FOC, &FOC::dma_isr_current_controller>( *this );

    mADCDriver = Chimera::ADC::getDriver( cfg.adcSource );
    mADCDriver->onInterrupt( Chimera::ADC::Interrupt::EOC_SEQUENCE, callback );

    return 0;
  }


  void FOC::dma_isr_current_controller( const Chimera::ADC::InterruptDetail &isr )
  {
    mPrvState.adc_samples[ 0 ] = isr.samples[ 0 ];
    mPrvState.adc_samples[ 1 ] = isr.samples[ 1 ];
    mPrvState.adc_samples[ 2 ] = isr.samples[ 2 ];
    //mPrvState.adc_samples[ 3 ] = isr.samples[ 3 ];

  }


  void FOC::timer_isr_speed_controller()
  {
  }

}    // namespace Orbit::Control




/*
Notes and Ideas:

- Dynamically adjust the PWM slicing frequency based on the system input voltage. This drives the theoretical maximum
  performance of the current controller. Can optimize for power dissipation or complete sine wave approximation. Could use
  sine wave control at lower speeds and then move into coarser control as speed increases.

    - System kV estimator (drives predictions of ISR limits)
    - Programmable kV "hint"
    - Measure runtime ISR speed and adjust sine approximation based on real performance limits.

  Ideally this will extend the control range of the ESC to very high kV/battery systems while also achieving the efficiency
  of sinusoidal drives.

    - Limiting factors will be overall system bandwidth. Acceleration may be a problem.
*/