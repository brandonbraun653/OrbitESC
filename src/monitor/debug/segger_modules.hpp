/******************************************************************************
 *  File Name:
 *    segger_modules.hpp
 *
 *  Description:
 *    Segger SystemView module definitions
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_SEGGER_MODULES_HPP
#define ORBIT_SEGGER_MODULES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "SEGGER_SYSVIEW.h"
#include <cstdint>

namespace Orbit::Monitor::Segger
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum SeggerModuleID : uint8_t
  {
    TUSB_ID,

    NumModules
  };

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/

  /**
   * @brief Information of all modules on OrbitESC
   */
  extern SEGGER_SYSVIEW_MODULE OSM[ SeggerModuleID::NumModules ];

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initialize all Segger SystemView modules
   * @return void
   */
  void initialize( void );

  /**
   * @brief Registers all Segger SystemView modules
   * @return void
   */
  void registerModules( void );
}

#endif  /* !ORBIT_SEGGER_MODULES_HPP */
