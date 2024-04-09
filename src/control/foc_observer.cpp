/******************************************************************************
 *  File Name:
 *    foc_observer.cpp
 *
 *  Description:
 *    Observer implementations for the Field Oriented Control (FOC) system.
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/utility>
#include "src/control/foc_math.hpp"
#include "src/control/foc_observer.hpp"
#include "src/core/data/orbit_data.hpp"

namespace Orbit::Control::Observer
{
  /*---------------------------------------------------------------------------
  Alises
  ---------------------------------------------------------------------------*/

  using PolicyFuncType = void ( * )( const Input &, Output & );

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ObserverState
  {
    /* Speed observer state */
    float z1;
    float z2;

    /* Phase observer state */
    float x1;
    float x2;
    float lambda_est;
    float i_alpha_last;
    float i_beta_last;

    /* Stator parameters */
    float R;
    float L;
    float lambda;
    float L_ia;
    float L_ib;
    float R_ia;
    float R_ib;
    float gamma_half;
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static Output         sEstimates;
  static PolicyFuncType sPolicyFunc;
  static ObserverState  sState;

  /*---------------------------------------------------------------------------
  Static Function Declaration
  ---------------------------------------------------------------------------*/

  static void luenberger_policy( const Input &input, Output &output );

  static void speed_observer( const Input &input, Output &output );

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initialize()
  {
    CLEAR_STRUCT( sEstimates );
    CLEAR_STRUCT( sState );
    sPolicyFunc = nullptr;
  }


  void setPolicy( const Policy policy )
  {
    switch( policy )
    {
      case Policy::LUENBERGER:
        sPolicyFunc = luenberger_policy;
        break;

      case Policy::NONE:
      default:
        sPolicyFunc = nullptr;
        break;
    }
  }


  void execute( const Input &input, Output &output )
  {
    using namespace Orbit::Control::Math;

    if( sPolicyFunc == nullptr )
    {
      return;
    }


    sState.R          = Data::SysConfig.statorResistance;
    sState.L          = Data::SysConfig.statorInductance;
    sState.lambda     = 0.075f;    // Permanent magnet flux linkage
    sState.L_ia       = sState.L * input.iAlpha;
    sState.L_ib       = sState.L * input.iBeta;
    sState.R_ia       = sState.R * input.iAlpha;
    sState.R_ib       = sState.R * input.iBeta;
    sState.gamma_half = 4.0f;    // Observer gain scaling. Probably not needed???

    /*-------------------------------------------------------------------------
    Execute the observer policy function
    -------------------------------------------------------------------------*/
    sPolicyFunc( input, output );

    /*-------------------------------------------------------------------------
    Update output state
    -------------------------------------------------------------------------*/
    sState.i_alpha_last = input.iAlpha;
    sState.i_beta_last  = input.iBeta;

    clear_if_nan( sState.x1 );
    clear_if_nan( sState.x2 );

    // Prevent the magnitude from getting too low, as that makes the angle very unstable.
    float mag = NORM2_f( sState.x1, sState.x2 );
    if( mag < ( sState.lambda * 0.5 ) )
    {
      sState.x1 *= 1.1;
      sState.x2 *= 1.1;
    }

    /*-------------------------------------------------------------------------
    Compute theta estimate from the observer state. (Equation 9)
    -------------------------------------------------------------------------*/
    output.theta = fast_atan2_with_norm( sState.x2 - sState.L_ib, sState.x1 - sState.L_ia );

    /*-------------------------------------------------------------------------
    Compute omega estimate from the observer state.
    -------------------------------------------------------------------------*/
    speed_observer( input, output );
  }


  void reset()
  {
  }


  Output estimates()
  {
    return sEstimates;
  }

  /*---------------------------------------------------------------------------
  Static Function Definitions
  ---------------------------------------------------------------------------*/

  /**
   * @brief
   * @see https://cas.mines-paristech.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
   *
   * @param input   Input parameters to the observer.
   * @param output  Output parameters from the observer.
   */
  static void luenberger_policy( const Input &input, Output &output )
  {
    float err = SQ( sState.lambda ) - ( SQ( sState.x1 - sState.L_ia ) + SQ( sState.x2 - sState.L_ib ) );

    // Forcing this term to stay negative helps convergence according to
    //
    // http://cas.ensmp.fr/Publications/Publications/Papers/ObserverPermanentMagnet.pdf
    // and
    // https://arxiv.org/pdf/1905.00833.pdf
    if( err > 0.0 )
    {
      err = 0.0;
    }

    float x1_dot = input.vAlpha - sState.R_ia + sState.gamma_half * ( sState.x1 - sState.L_ia ) * err;
    float x2_dot = input.vBeta  - sState.R_ib + sState.gamma_half * ( sState.x2 - sState.L_ib ) * err;

    sState.x1 += x1_dot * input.dt;
    sState.x2 += x2_dot * input.dt;
  }


  static void speed_observer( const Input &input, Output &output )
  {
    static constexpr float kp = 5.0;
    static constexpr float ki = 0.1;

    static float theta_last = 0.0f;
    static float filtered_omega = 0.0f;

    /*-------------------------------------------------------------------------
    Compute the observer state derivatives
    -------------------------------------------------------------------------*/
    float err_term = output.theta - sState.z1;
    Math::normalize_radians( err_term );

    /* Equation 11 */
    float z1_dot = kp * err_term + ki * sState.z2;

    /* Equation 12 */
    float z2_dot = ki * err_term;

    /*-------------------------------------------------------------------------
    Update the observer state
    -------------------------------------------------------------------------*/
    sState.z1 += z1_dot * input.dt;
    Math::normalize_radians( sState.z1 );

    sState.z2 += z2_dot * input.dt;

    // Testing
    output.omega = z1_dot;
  }
}    // namespace Orbit::Control::Observer
