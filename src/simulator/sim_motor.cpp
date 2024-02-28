/******************************************************************************
 *  File Name:
 *    sim_motor.cpp
 *
 *  Description:
 *    Motor simulation model for the test harness
 *
 *  Citation:
 *    This is inspired/copied from the VESC firmware. The original source can
 *    be found at:
 *      https://github.com/vedderb/bldc/blob/master/motor/s_state.c
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/utility>
#include <src/simulator/sim_motor.hpp>
#include <src/control/foc_math.hpp>

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

    s_state.connected = true;
  }


  void disconnect()
  {
    s_state.connected = false;
    CLEAR_STRUCT( s_state );
  }


  void stepModel( const float alpha, const float beta, const float flux_linkage )
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
    s_state.vd = s_state.cos_phi * alpha + s_state.sin_phi * beta;
    s_state.vq = s_state.cos_phi * beta - s_state.sin_phi * alpha;

    // d axis current
    s_state.id_int += ( ( s_state.vd + s_state.we * s_params.pole_pairs * s_params.lq * s_state.iq - s_params.r * s_state.id ) *
                        s_params.Ts ) /
                      s_params.ld;
    s_state.id = s_state.id_int - flux_linkage / s_params.ld;

    // q axis current
    s_state.iq += ( s_state.vq - s_state.we * s_params.pole_pairs * ( s_params.ld * s_state.id + flux_linkage ) -
                    s_params.r * s_state.iq ) *
                  s_params.Ts / s_params.lq;

    // TODO: Add current limiting
    // truncate_fabs(s_state.iq, (2048 * FAC_CURRENT) );
    // truncate_fabs(s_state.id, (2048 * FAC_CURRENT) );

    /*-------------------------------------------------------------------------
    Mechanical Simulation
    -------------------------------------------------------------------------*/
    s_state.me = s_params.km * ( flux_linkage + ( s_params.ld - s_params.lq ) * s_state.id ) * s_state.iq;
    // omega
    s_state.we += s_state.tsj * ( s_state.me - s_state.ml );

    // phi
    s_state.phi += s_state.we * s_params.Ts;

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
    fast_sin_cos( s_state.phi, &s_state.sin_phi, &s_state.cos_phi );

    //	Park Inverse
    s_state.i_alpha = s_state.cos_phi * s_state.id - s_state.sin_phi * s_state.iq;
    s_state.i_beta  = s_state.cos_phi * s_state.iq + s_state.sin_phi * s_state.id;

    s_state.v_alpha = s_state.cos_phi * s_state.vd - s_state.sin_phi * s_state.vq;
    s_state.v_beta  = s_state.cos_phi * s_state.vq + s_state.sin_phi * s_state.vd;

    //	Clark Inverse
    s_state.ia = s_state.i_alpha;
    s_state.ib = -0.5 * s_state.i_alpha + SQRT3_OVER_2 * s_state.i_beta;
    s_state.ic = -0.5 * s_state.i_alpha - SQRT3_OVER_2 * s_state.i_beta;

    s_state.va = s_state.v_alpha;
    s_state.vb = -0.5 * s_state.v_alpha + SQRT3_OVER_2 * s_state.v_beta;
    s_state.vc = -0.5 * s_state.v_alpha - SQRT3_OVER_2 * s_state.v_beta;

    // //	simulate current samples
    // ADC_Value[ ADC_IND_CURR1 ] =  s_state.ia / FAC_CURRENT + 2048;
    // ADC_Value[ ADC_IND_CURR2 ] =  s_state.ib / FAC_CURRENT + 2048;
    // ADC_Value[ ADC_IND_CURR3 ] =  s_state.ic / FAC_CURRENT + 2048;

    // //	simulate voltage samples
    // ADC_Value[ ADC_IND_SENS1 ] = s_state.va * VOLTAGE_TO_ADC_FACTOR + 2048;
    // ADC_Value[ ADC_IND_SENS2 ] = s_state.vb * VOLTAGE_TO_ADC_FACTOR + 2048;
    // ADC_Value[ ADC_IND_SENS3 ] = s_state.vc * VOLTAGE_TO_ADC_FACTOR + 2048;
  }


  State modelState()
  {
    return s_state;
  }

}    // namespace Orbit::Sim::Motor
