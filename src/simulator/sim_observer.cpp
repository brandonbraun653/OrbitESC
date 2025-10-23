/******************************************************************************
 *  File Name:
 *    sim_observer.cpp
 *
 *  Description:
 *    Simulator injection implementation for the FOC observer.
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/simulator/sim_observer.hpp>

namespace Orbit::Sim::Control::Observer
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static float s_theta_elec = 0.0f;
  static float s_omega_elec = 0.0f;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void inject( const float theta_elec, const float omega_elec )
  {
    s_theta_elec = theta_elec;
    s_omega_elec = omega_elec;
  }

  void execute( const Orbit::Control::Observer::Input &input, Orbit::Control::Observer::Output &output )
  {
    output.theta_elec = s_theta_elec;
    output.omega_elec = s_omega_elec;
  }
}    // namespace Orbit::Sim::Control::Observer
