/******************************************************************************
 *  File Name:
 *    sim_observer.hpp
 *
 *  Description:
 *    Simulator injection interface for the FOC observer.
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SIM_OBSERVER_HPP
#define ORBIT_ESC_SIM_OBSERVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/foc_observer.hpp>


namespace Orbit::Sim::Control::Observer
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Injects the estimated rotor position and speed into the observer
   *
   * @param theta_elec  The estimated rotor position in radians
   * @param omega_elec  The estimated rotor speed in radians per second
   * @return void
   */
  void inject( const float theta_elec, const float omega_elec );

  /**
   * @brief Executes the observer policy
   *
   * Same interface as the real observer, but with simulated data.
   *
   * @param input   The input data to the observer
   * @param output  The output data from the observer
   * @return void
   */
  void execute( const Orbit::Control::Observer::Input &input, Orbit::Control::Observer::Output &output );

}    // namespace Orbit::Sim::Control::Observer

#endif /* !ORBIT_ESC_SIM_OBSERVER_HPP */
