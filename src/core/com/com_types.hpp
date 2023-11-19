/******************************************************************************
 *  File Name:
 *    com_types.hpp
 *
 *  Description:
 *    Types for the communication stack
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_COM_TYPES_HPP
#define ORBIT_ESC_COM_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace Orbit::COM::Scheduler
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Communication endpoints supported in HW.
   * @note OrbitESC only has one of each endpoint.
   */
  enum Endpoint : uint8_t
  {
    CAN         = 1u << 0,
    UART        = 1u << 1,
    USB         = 1u << 2,
    NUM_OPTIONS = 3u
  };

}    // namespace Orbit::COM::Scheduler

#endif /* !ORBIT_ESC_COM_TYPES_HPP */
