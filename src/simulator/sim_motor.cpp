/******************************************************************************
 *  File Name:
 *    sim_motor.cpp
 *
 *  Description:
 *    Motor simulation model for the test harness
 *
 *  Citations:
 *    1. This is inspired from the VESC firmware. The original source can
 *    be found at:
 *      https://github.com/vedderb/bldc/blob/master/motor/s_state.c
 *    2. https://ieeexplore.ieee.org/document/6687627 (PMSM DQ Model)
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/utility>
#include <src/simulator/sim_adc.hpp>
#include <src/simulator/sim_motor.hpp>
#include <src/control/foc_math.hpp>
#include <src/core/data/orbit_data.hpp>

namespace Orbit::Sim::Motor
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Parameters s_params;
  static State      s_state;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void connect( const Parameters &params )
  {
    s_params = params;
    CLEAR_STRUCT( s_state );

    s_state.Ts        = 1.0f / Data::SysControl.statorPWMFreq;
    s_state.tsj       = s_state.Ts / s_params.J;
    s_state.ml        = 0.0f;
    s_state.connected = true;
  }


  void disconnect()
  {
    s_state.connected = false;
    CLEAR_STRUCT( s_state );
  }


  void stepModel( const float alpha, const float beta )
  {
    using namespace Orbit::Control::Math;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if( !s_state.connected )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Electrical Simulation
    -------------------------------------------------------------------------*/
    park_transform( alpha, beta, s_state.phi, s_state.vq, s_state.vd );

    // d axis current
    s_state.id += ( ( s_state.vd + s_state.we * s_params.pole_pairs * s_params.lq * s_state.iq - s_params.r * s_state.id ) *
                        s_state.Ts ) /
                      s_params.ld;

    s_state.id -= s_params.lpm / s_params.ld;

    // q axis current
    s_state.iq += ( s_state.vq - s_state.we * s_params.pole_pairs * ( s_params.ld * s_state.id + s_params.lpm ) -
                    s_params.r * s_state.iq ) *
                  s_state.Ts / s_params.lq;

    // TODO: Add current limiting
    // truncate_fabs( s_state.iq, 20.0f );
    // truncate_fabs( s_state.id, 20.0f );

    /*-------------------------------------------------------------------------
    Mechanical Simulation
    -------------------------------------------------------------------------*/
    s_state.me = s_params.km * ( s_params.lpm + ( s_params.ld - s_params.lq ) * s_state.id ) * s_state.iq;
    // omega
    s_state.we += s_state.tsj * ( s_state.me - s_state.ml );

    // phi
    s_state.phi += s_state.we * s_state.Ts;

    // phi limits
    while( s_state.phi > M_PI_F )
    {
      s_state.phi -= M_2PI_F;
    }

    while( s_state.phi < -1.0f * M_PI_F )
    {
      s_state.phi += M_2PI_F;
    }

    /*-------------------------------------------------------------------------
    Convert id/iq calculated values into ADC measurements
    -------------------------------------------------------------------------*/
    // Current
    inverse_park_transform( s_state.iq, s_state.id, s_state.phi, s_state.i_alpha, s_state.i_beta );
    inverse_clarke_transform( s_state.i_alpha, s_state.i_beta, s_state.ia, s_state.ib, s_state.ic );

    // Voltage
    inverse_park_transform( s_state.vq, s_state.vd, s_state.phi, s_state.v_alpha, s_state.v_beta );
    inverse_clarke_transform( s_state.v_alpha, s_state.v_beta, s_state.va, s_state.vb, s_state.vc );

    /*-------------------------------------------------------------------------
    Update the ADC for the next measurement cycle
    -------------------------------------------------------------------------*/
    ADC::setPhaseData( s_state.va, s_state.vb, s_state.vc, s_state.ia, s_state.ib, s_state.ic );
  }


  State modelState()
  {
    // TODO: Add mutex protection
    return s_state;
  }

}    // namespace Orbit::Sim::Motor
