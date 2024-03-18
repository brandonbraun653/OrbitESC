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
    float x1;
    float x2;
    float lambda_est;
    float i_alpha_last;
    float i_beta_last;

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
    sState.lambda     = 0.075;    // Permanent magnet flux linkage
    sState.L_ia       = sState.L * input.iAlpha;
    sState.L_ib       = sState.L * input.iBeta;
    sState.R_ia       = sState.R * input.iAlpha;
    sState.R_ib       = sState.R * input.iAlpha;
    sState.gamma_half = 4.0 * 0.5;    // Observer gain scaling. Probably not needed???

    /*-------------------------------------------------------------------------
    Execute the observer policy function
    -------------------------------------------------------------------------*/
    sPolicyFunc( input, output );

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

    output.theta = fast_atan2_with_norm( sState.x2 - sState.L_ib, sState.x1 - sState.L_ia );

    // TODO BMB: foc_pll_run() from vedder is the speed estimator. Need to implement that.
    // TODO BMB: Also see the equations from part B of the observer paper, eq 11, 12, 13.
    // TODO BMB: Apparently this is a tracking controller? Need to look into that.

    // TODO BMB: I also need to publish the estimates over USB. Time to modify the data struct and decrease size.
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
    float x2_dot = input.vBeta - sState.R_ib + sState.gamma_half * ( sState.x2 - sState.L_ib ) * err;

    sState.x1 += x1_dot * input.dt;
    sState.x2 += x2_dot * input.dt;
  }
}    // namespace Orbit::Control::Observer
