/******************************************************************************
 *  File Name:
 *    optimize_sample_point.hpp
 *
 *  Description:
 *    Subroutine for development purposes only. Used to find the optimal
 *    sample timing for the ADC triggering.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_OPTIMAL_ADC_SAMPLING_TIME_HPP
#define ORBIT_ESC_OPTIMAL_ADC_SAMPLING_TIME_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/subroutines/interface.hpp>

namespace Orbit::Control::Subroutine
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  class SampleTimeOptimizer : public ISubroutine
  {
  public:
    SampleTimeOptimizer();
    ~SampleTimeOptimizer();

    void  initialize() final override;
    void  start() final override;
    void  stop() final override;
    void  destroy() final override;
    void  process() final override;
    RunState state() final override;
  };

}  // namespace Orbit::Control::Subroutine

#endif  /* !ORBIT_ESC_OPTIMAL_ADC_SAMPLING_TIME_HPP */
