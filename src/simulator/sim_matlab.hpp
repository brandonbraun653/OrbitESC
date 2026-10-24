/******************************************************************************
 *  File Name:
 *    sim_matlab.hpp
 *
 *  Description:
 *    Matlab simulator callback interfaces
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef SIM_MATLAB_HPP
#define SIM_MATLAB_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/simulator/sim_tcp_server.hpp>

namespace Orbit::Sim::Matlab
{
  /**
   * @brief Callback function for motor simulation TCP server
   * @param server Reference to the TCP server instance
   * @param data Pointer to received data
   * @param size Size of received data
   */
  void motorSimulationCallback( Orbit::Sim::TCP::Server &server, const void *data, size_t size );

  /**
   * @brief Callback function for ESC control TCP server
   * @param server Reference to the TCP server instance
   * @param data Pointer to received data
   * @param size Size of received data
   */
  void escControlCallback( Orbit::Sim::TCP::Server &server, const void *data, size_t size );

  /**
   * @brief Connection callback for ESC control TCP server
   * @param server Reference to the TCP server instance
   * @param state  Connection state change
   */
  void escControlConnectionCallback( Orbit::Sim::TCP::Server &server, Orbit::Sim::TCP::ConnectionState state );

}    // namespace Orbit::Sim::Matlab

#endif /* !SIM_MATLAB_HPP */
