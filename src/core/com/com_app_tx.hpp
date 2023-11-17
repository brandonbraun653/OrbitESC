/******************************************************************************
 *  File Name:
 *    com_app_tx.hpp
 *
 *  Description:
 *    Transport agnostic interface for publishing data to a remote host
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_COM_APP_TX_HPP
#define ORBIT_COM_APP_TX_HPP

namespace Orbit::Com
{
  /*---------------------------------------------------------------------------
  Public Functions

  These functions are used to publish data to the remote host. They are called
  from the main application loop and should not be called from an interrupt.
  ---------------------------------------------------------------------------*/
  void publishPhaseCurrents();
  void publishPhaseVoltages();
  void publishStateEstimates();
  void publishSystemADCMeasurements();

}    // namespace Orbit::Com

#endif /* !ORBIT_COM_APP_TX_HPP */
