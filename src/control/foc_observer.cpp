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
#include "dsp/filtering_functions.h"
#include "src/control/gen/iir_coeffs.h"
#include "src/control/gen/fir_coeffs.h"

namespace Orbit::Control::Observer
{
  /*---------------------------------------------------------------------------
  Alises
  ---------------------------------------------------------------------------*/

  using PolicyFuncType = void ( * )( const Input &, Output & );

  // static constexpr size_t IIR_STAGES = 2;
  static constexpr size_t IIR_FILTER = 3; /**< Number of IIR filters used */
  static constexpr size_t IIR_STAGES = STAGES;
  static constexpr size_t FIR_TAPS = M_FIR;
  static constexpr size_t BLOCK_SIZE = 1;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ObserverState
  {
    /* Speed observer state */
    float z1;
    float z2;
    arm_fir_instance_f32 speed_filter;
    float32_t speed_filter_state[ FIR_TAPS + BLOCK_SIZE - 1 ];

    arm_biquad_cascade_df2T_instance_f32 iir_filter[ IIR_FILTER ];
    float32_t iir_filter_state[ IIR_FILTER ][ 2 * IIR_STAGES ];



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

    arm_fir_init_f32( &sState.speed_filter, FIR_TAPS, ( float32_t * )h_FIR, sState.speed_filter_state, BLOCK_SIZE );

    for( size_t i = 0; i < 3; i++ )
    {
      arm_biquad_cascade_df2T_init_f32( &sState.iir_filter[ i ],
                                        IIR_STAGES,
                                        ba_coeff,
                                        &sState.iir_filter_state[ i ][ 0 ] );
    }
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
    sState.gamma_half = 1.0f;    // Observer gain scaling. Probably not needed???

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
    using namespace Orbit::Control::Math;
    static float theta_last = 0.0f;
    static float filtered_omega = 0.0f;

    // /*-------------------------------------------------------------------------
    // Compute the observer state derivatives
    // -------------------------------------------------------------------------*/
    // float err_term = output.theta - sState.z1;
    // Math::normalize_radians( err_term );

    // /* Equation 11 */
    // float z1_dot = kp * err_term + ki * sState.z2;

    // /* Equation 12 */
    // float z2_dot = ki * err_term;

    // /*-------------------------------------------------------------------------
    // Update the observer state
    // -------------------------------------------------------------------------*/
    // sState.z1 += z1_dot * input.dt;
    // Math::normalize_radians( sState.z1 );

    // sState.z2 += z2_dot * input.dt;

    // // Testing
    // output.omega = z1_dot;

    // Take the derivative of the angle to get the angular rate
    float dTheta = ( output.theta - theta_last ) / input.dt;

    // Update the last angle
    theta_last = output.theta;

    // Filter the angular rate to remove noise
    arm_fir_f32( &sState.speed_filter, &dTheta, &filtered_omega, BLOCK_SIZE );

    float stage1_output = 0.0f;
    arm_biquad_cascade_df2T_f32( &sState.iir_filter[ 0 ], &filtered_omega, &stage1_output, BLOCK_SIZE );

    float stage2_output = 0.0f;
    arm_biquad_cascade_df2T_f32( &sState.iir_filter[ 1 ], &stage1_output, &stage2_output, BLOCK_SIZE );

    float stage3_output = 0.0f;
    arm_biquad_cascade_df2T_f32( &sState.iir_filter[ 2 ], &stage2_output, &stage3_output, BLOCK_SIZE );

    // Update the output
    output.omega = stage3_output;
  }
}    // namespace Orbit::Control::Observer
