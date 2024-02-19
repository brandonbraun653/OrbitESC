/******************************************************************************
 *  File Name:
 *    foc_data.cpp
 *
 *  Description:
 *    Data storage for the FOC algorithms
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/foc_data.hpp>

namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  SystemState         foc_motor_state;
  CurrentControlState foc_ireg_state;


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initFOCData()
  {
    /*-------------------------------------------------------------------------
    Reset motor system state
    -------------------------------------------------------------------------*/
    foc_motor_state.theta    = 0.0f;
    foc_motor_state.omega    = 0.0f;
    foc_motor_state.thetaEst = 0.0f;
    foc_motor_state.omegaEst = 0.0f;

    /*-------------------------------------------------------------------------
    Reset current control state
    -------------------------------------------------------------------------*/
    foc_ireg_state.dt        = 0.0f;
    foc_ireg_state.ima       = 0.0f;
    foc_ireg_state.imb       = 0.0f;
    foc_ireg_state.imc       = 0.0f;
    foc_ireg_state.vma       = 0.0f;
    foc_ireg_state.vmb       = 0.0f;
    foc_ireg_state.vmc       = 0.0f;
    foc_ireg_state.iq        = 0.0f;
    foc_ireg_state.id        = 0.0f;
    foc_ireg_state.vq        = 0.0f;
    foc_ireg_state.vd        = 0.0f;
    foc_ireg_state.va        = 0.0f;
    foc_ireg_state.vb        = 0.0f;
    foc_ireg_state.ia        = 0.0f;
    foc_ireg_state.ib        = 0.0f;
    foc_ireg_state.iqRef     = 0.0f;
    foc_ireg_state.idRef     = 0.0f;
    foc_ireg_state.max_drive = 0.0f;
    foc_ireg_state.iqPID.init();
    foc_ireg_state.idPID.init();
  }
}    // namespace Orbit::Control
