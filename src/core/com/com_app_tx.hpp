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

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/


namespace Orbit::Com
{
  /**
   * @brief Captures the latest phase current samples and enqueues it for transmission
   * @return void
   */
  void publishPhaseCurrents();

  /**
   * @brief Captures the latest phase voltage samples and enqueues it for transmission
   * @return void
   */
  void publishPhaseVoltages();

  /**
   * @brief Captures the latest state estimates and enqueues it for transmission
   * @return void
   */
  void publishStateEstimates();

}    // namespace Orbit::Com

#endif /* !ORBIT_COM_APP_TX_HPP */
