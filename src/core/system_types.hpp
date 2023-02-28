/******************************************************************************
 *  File Name:
 *    system_types.hpp
 *
 *  Description:
 *    Types for the system module
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SYSTEM_TYPES_HPP
#define ORBIT_SYSTEM_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <src/core/com/serial/serial_interface.pb.h>

namespace Orbit::System
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class Mode : uint8_t
  {
    NORMAL = BootMode_BOOT_MODE_NORMAL, /**< Standard operational mode for flight */
    CONFIG = BootMode_BOOT_MODE_CONFIG, /**< Non-flight mode for configuration and control */
    TEST   = BootMode_BOOT_MODE_TEST,   /**< Non-flight mode for testing */
    NUM_OPTIONS
  };
}  // namespace Orbit::System

#endif  /* !ORBIT_SYSTEM_TYPES_HPP */
