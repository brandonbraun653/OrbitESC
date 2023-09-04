/******************************************************************************
 *  File Name:
 *    system_observer.cpp
 *
 *  Description:
 *    Observer implementations for FOC control
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/system_observer.hpp>

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initObservers()
  {
    // /*-------------------------------------------------------------------------
    // Reset module memory
    // -------------------------------------------------------------------------*/
    // s_state.isrControlActive   = false;
    // s_state.switchToClosedLoop = false;
    // s_state.iLoop.clear();
    // s_state.iObserve.clear();
    // s_state.wControl.clear();

    // // !TESTING
    // s_state.observer.clear();
    // s_state.motor.clear();

    // s_state.motor.m_gamma_now = 2500.0f;
    // // !TESTING


    // /*-------------------------------------------------------------------------
    // Initialize the current observer
    // -------------------------------------------------------------------------*/
    // float initial_alpha = Control::Math::M_PI_F / ( Data::SysControl.statorPWMFreq / Data::SysControl.speedCtrlUpdateFreq );

    // /* Pre-calculate F/G gain terms for the current observer */
    // s_state.iObserve.f_gain = 1.0f - s_state.iLoop.dt * ( Data::SysConfig.statorResistance / Data::SysConfig.statorInductance );
    // s_state.iObserve.g_gain = s_state.iLoop.dt / Data::SysConfig.statorInductance;

    // /* Map the phase current alpha/beta references */
    // s_state.iObserve.phase[ 0 ].p_act_i = &s_state.iLoop.ia;
    // s_state.iObserve.phase[ 1 ].p_act_i = &s_state.iLoop.ib;

    // /* Map the phase voltage alpha/beta commands */
    // s_state.iObserve.phase[ 0 ].p_cmd_v = &s_state.iLoop.va;
    // s_state.iObserve.phase[ 1 ].p_cmd_v = &s_state.iLoop.vb;

    // /* Initialize the lpf gains */
    // s_state.iObserve.phase[ 0 ].lpf_alpha = initial_alpha;
    // s_state.iObserve.phase[ 1 ].lpf_alpha = initial_alpha;
    // s_state.sObserve.lpf_alpha            = initial_alpha;
  }


  void updateCurrentObserver()
  {
    // /*-------------------------------------------------------------------------
    // Pre-compute motor dependent constants
    // -------------------------------------------------------------------------*/
    // static const float nrl  = -1.0f * ( Data::SysConfig.statorResistance / Data::SysConfig.statorInductance );
    // static const float invL = 1.0f / Data::SysConfig.statorInductance;

    // /*-------------------------------------------------------------------------
    // Update the current observer for each phase
    // -------------------------------------------------------------------------*/
    // for ( uint32_t idx = 0; idx < ARRAY_COUNT( s_state.iObserve.phase ); idx++ )
    // {
    //   CurrentObserverState::AlgVars *pAlg = &s_state.iObserve.phase[ idx ];

    //   /*-----------------------------------------------------------------------
    //   Step the virtual motor model
    //   -----------------------------------------------------------------------*/
    //   pAlg->est_i = ( nrl * pAlg->est_i ) + ( invL * ( *pAlg->p_cmd_v - pAlg->est_e - pAlg->comp_z ) );

    //   /*-----------------------------------------------------------------------
    //   Compute the error signal
    //   -----------------------------------------------------------------------*/
    //   pAlg->err_i = pAlg->est_i - *pAlg->p_act_i;
    //   pAlg->err_i = Control::Math::clamp( pAlg->err_i, -25.0f, 25.0f );

    //   /*-----------------------------------------------------------------------
    //   Update the correction factor
    //   -----------------------------------------------------------------------*/
    //   if ( fabs( pAlg->err_i ) < Data::SysControl.currentObserver_MaxError )
    //   {
    //     pAlg->comp_z = ( pAlg->err_i * Data::SysControl.currentObserver_KSlide ) / Data::SysControl.currentObserver_MaxError;
    //   }
    //   else if ( pAlg->err_i > 0 )
    //   {
    //     pAlg->comp_z = Data::SysControl.currentObserver_KSlide;
    //   }
    //   else
    //   {
    //     pAlg->comp_z = -1.0f * Data::SysControl.currentObserver_KSlide;
    //   }

    //   /*-----------------------------------------------------------------------
    //   Update the estimated back EMF w/First Order LPF:
    //     y[k] = alpha * x[k] + ( 1 - alpha ) * y[k-1]
    //   -----------------------------------------------------------------------*/
    //   pAlg->est_e = pAlg->lpf_alpha * pAlg->comp_z + ( 1.0f - pAlg->lpf_alpha ) * pAlg->est_e;

    //   /*-----------------------------------------------------------------------
    //   Filter the back EMF estimate for use in the speed/position observer.
    //   -----------------------------------------------------------------------*/
    //   pAlg->est_e_filtered = pAlg->lpf_alpha * pAlg->est_e + ( 1.0f - pAlg->lpf_alpha ) * pAlg->est_e_filtered;
    // }
  }


  void updateSpeedPosObserver()
  {
    // SpeedPosObserverState *pAlg = &s_state.sObserve;

    // /*-------------------------------------------------------------------------
    // Compute estimated position using arctan(eAlpha/eBeta)
    // -------------------------------------------------------------------------*/
    // float theta_prv = pAlg->theta_est;
    // pAlg->theta_est = Control::Math::fast_atan2_with_norm( s_state.iObserve.phase[ 0 ].est_e_filtered,
    //                                                        s_state.iObserve.phase[ 1 ].est_e_filtered );

    // /*-------------------------------------------------------------------------
    // Accumulate change in position over time
    // -------------------------------------------------------------------------*/
    // pAlg->acc_theta_delta += ( pAlg->theta_est - theta_prv );
  }


  void observerUpdate()
  {
    //   float R          = Data::SysConfig.statorResistance;
    //   float L          = Data::SysConfig.statorInductance;
    //   float lambda     = state.motor.foc_motor_flux_linkage;
    //   float ld_lq_diff = state.motor.foc_motor_ld_lq_diff;
    //   float id         = state.iLoop.id;
    //   float iq         = state.iLoop.iq;

    //   /*-------------------------------------------------------------------------
    //   Adjust inductance for saliency
    //   -------------------------------------------------------------------------*/
    //   if ( fabsf( id ) > 0.1 || fabsf( iq ) > 0.1 )
    //   {
    //     L = L - ld_lq_diff / 2.0f + ld_lq_diff * SQ( iq ) / ( SQ( id ) + SQ( iq ) );
    //   }

    //   /*-------------------------------------------------------------------------
    //   Pre-compute motor dependent constants
    //   -------------------------------------------------------------------------*/
    //   float       L_ia       = L * state.iLoop.ia;
    //   float       L_ib       = L * state.iLoop.ib;
    //   const float R_ia       = R * state.iLoop.ia;
    //   const float R_ib       = R * state.iLoop.ib;
    //   const float gamma_half = state.motor.m_gamma_now * 0.5;

    //   /*-------------------------------------------------------------------------
    //   Run the flux-linkage observer

    //   This implements equation 8 from:
    //   https://cas.mines-paristech.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
    //   -------------------------------------------------------------------------*/
    //   float err = SQ( lambda ) - ( SQ( state.observer.x1 - L_ia ) + SQ( state.observer.x2 - L_ib ) );

    //   // Forcing this term to stay negative helps convergence according to
    //   //
    //   // http://cas.ensmp.fr/Publications/Publications/Papers/ObserverPermanentMagnet.pdf
    //   // and
    //   // https://arxiv.org/pdf/1905.00833.pdf
    //   if ( err > 0.0 )
    //   {
    //     err = 0.0;
    //   }

    //   float x1_dot = state.iLoop.va - R_ia + gamma_half * ( state.observer.x1 - L_ia ) * err;
    //   float x2_dot = state.iLoop.vb - R_ib + gamma_half * ( state.observer.x2 - L_ib ) * err;

    //   state.observer.x1 += x1_dot * state.iLoop.dt;
    //   state.observer.x2 += x2_dot * state.iLoop.dt;

    //   /*-------------------------------------------------------------------------
    //   Update state variables and massage bad numbers
    //   -------------------------------------------------------------------------*/
    //   state.observer.i_alpha_last = state.iLoop.ia;
    //   state.observer.i_beta_last  = state.iLoop.ib;

    //   Control::Math::clear_if_nan( state.observer.x1 );
    //   Control::Math::clear_if_nan( state.observer.x2 );

    //   /*-------------------------------------------------------------------------
    //   Prevent the magnitude from getting too low, which leads to unstable angles
    //   -------------------------------------------------------------------------*/
    //   float mag = NORM2_f( state.observer.x1, state.observer.x2 );
    //   if ( mag < ( lambda * 0.5f ) )
    //   {
    //     state.observer.x1 *= 1.1f;
    //     state.observer.x2 *= 1.1f;
    //   }

    //   /*-------------------------------------------------------------------------
    //   Update phase angle
    //   -------------------------------------------------------------------------*/
    //   float y                          = state.observer.x2 - L_ib;
    //   float x                          = state.observer.x1 - L_ia;
    //   state.motor.m_phase_now_observer = Control::Math::fast_atan2_with_norm( y, x );
  }
}    // namespace Orbit::Control
