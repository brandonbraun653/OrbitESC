/******************************************************************************
 *  File Name:
 *    orbit_monitors.cpp
 *
 *  Description:
 *    Implementation of high level monitor details
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/monitor/orbit_monitors.hpp>

namespace Orbit::Monitor
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  // VoltageMonitor VBusMonitor;
  // CurrentMonitor IPhAMonitor;
  // CurrentMonitor IPhBMonitor;
  // CurrentMonitor IPhCMonitor;

  // const etl::array<IAnalogMonitor *const, Orbit::Control::ADC_CH_NUM_OPTIONS> MonitorArray = {
  //   &IPhAMonitor, /* ADC_CH_MOTOR_PHASE_A_CURRENT */
  //   &IPhBMonitor, /* ADC_CH_MOTOR_PHASE_B_CURRENT */
  //   &IPhCMonitor, /* ADC_CH_MOTOR_PHASE_C_CURRENT */
  //   &VBusMonitor  /* ADC_CH_MOTOR_SUPPLY_VOLTAGE */
  // };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  // void initialize()
  // {
  //   using namespace Orbit::Control;

  //   RT_DBG_ASSERT( MonitorArray[ ADC_CH_MOTOR_SUPPLY_VOLTAGE ] == &VBusMonitor );
  //   RT_DBG_ASSERT( MonitorArray[ ADC_CH_MOTOR_PHASE_A_CURRENT ] == &IPhAMonitor );
  //   RT_DBG_ASSERT( MonitorArray[ ADC_CH_MOTOR_PHASE_B_CURRENT ] == &IPhBMonitor );
  //   RT_DBG_ASSERT( MonitorArray[ ADC_CH_MOTOR_PHASE_C_CURRENT ] == &IPhCMonitor );

  //   VBusMonitor.setThresholds( 0.0f, 14.0f );
  //   IPhAMonitor.setThresholds( -3.0f, 3.0f );
  //   IPhBMonitor.setThresholds( -3.0f, 3.0f );
  //   IPhCMonitor.setThresholds( -3.0f, 3.0f );

  //   for( auto mon : MonitorArray )
  //   {
  //     mon->setEngageState( EngageState::INACTIVE );
  //   }
  // }
}    // namespace Orbit::Monitor
