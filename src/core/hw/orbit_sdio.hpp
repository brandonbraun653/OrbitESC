/******************************************************************************
 *  File Name:
 *    orbit_sdio.hpp
 *
 *  Description:
 *    OrbitESC SDIO driver interface
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_SDIO_HPP
#define ORBIT_ESC_SDIO_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/function>

namespace Orbit::SDIO
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Initializes the SDIO driver subsystem.
   * @return void
   */
  void powerUp();

  /**
   * @brief Handles insert/remove events from the card detect pin.
   * @return void
   */
  void handleCardStatusChange();

  /**
   * @brief Register a callback to be executed when a card is detected.
   *
   * @param func Function to be called
   * @return void
   */
  void onCardInsert( Chimera::Function::vGeneric &func );

  /**
   * @brief Register a callback to be executed when a card is ejected.
   *
   * @param func Function to be called
   * @return void
   */
  void onCardRemove( Chimera::Function::vGeneric &func );

  /**
   * @brief Checks if a card is present in the slot.
   * @return bool
   */
  bool isCardPresent();

}  // namespace Orbit::SDIO

#endif  /* !ORBIT_ESC_SDIO_HPP */
