/******************************************************************************
 *  File Name:
 *    system_control.cpp
 *
 *  Description:
 *    High level system control interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/foc_driver.hpp>
#include <src/control/subroutines/declarations.hpp>
#include <src/control/subroutines/interface.hpp>
#include <src/control/subroutines/interface.hpp>
#include <src/control/subroutines/optimize_sample_point.hpp>
#include <src/control/subroutines/rotor_detector.hpp>
#include <src/control/subroutines/rotor_ramp.hpp>
#include <src/control/system_control.hpp>

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static Control::Subroutine::IdleSubroutine      s_idle_routine;
  static Control::Subroutine::RotorDetector       s_rotor_pos_detector_routine;
  static Control::Subroutine::SampleTimeOptimizer s_sample_time_routine;
  static Control::Subroutine::RotorRamp           s_rotor_ramp_routine;


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
    /*-------------------------------------------------------------------------
    Initialize the subroutine manager
    -------------------------------------------------------------------------*/
    Orbit::Control::Subroutine::initialize();
    Control::Subroutine::bind( Control::Subroutine::Routine::IDLE, &s_idle_routine );
    Control::Subroutine::bind( Control::Subroutine::Routine::ALIGNMENT_DETECTION, &s_rotor_pos_detector_routine );
    Control::Subroutine::bind( Control::Subroutine::Routine::ADC_SAMPLE_POINT_OPTIMIZER, &s_sample_time_routine );
    Control::Subroutine::bind( Control::Subroutine::Routine::OPEN_LOOP_RAMP_FOC, &s_rotor_ramp_routine );

    /*-------------------------------------------------------------------------
    Initialize the FOC driver
    -------------------------------------------------------------------------*/
    Orbit::Control::FOC::initialize();
  }

}    // namespace Orbit::Control
