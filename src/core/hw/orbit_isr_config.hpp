/******************************************************************************
 *  File Name:
 *    orbit_isr_config.hpp
 *
 *  Description:
 *    ISR configuration for OrbitESC
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_ISR_CONFIGURATION_HPP
#define ORBIT_ESC_ISR_CONFIGURATION_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <src/config/freertos/FreeRTOSConfig.h>

namespace Orbit
{
  /*---------------------------------------------------------------------------
  Bare Metal Priorities
  ---------------------------------------------------------------------------*/

  /**
   * @brief Motor control ADC DMA ISR priority
   *
   * The motor control callback runs from the ADC DMA stream interrupt. TIM8
   * triggers conversions in hardware and does not require an NVIC priority.
   */
  static constexpr std::uint32_t FOC_ISR_CHAIN_PRI_LVL = 1;
  static_assert( FOC_ISR_CHAIN_PRI_LVL <
                 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );

  // R1362
  static constexpr std::uint32_t FOC_DMA_ISR_PRI_LVL = FOC_ISR_CHAIN_PRI_LVL;

  /*---------------------------------------------------------------------------
  RTOS Aware Priorities
  ---------------------------------------------------------------------------*/

  // R1363
  static constexpr std::uint32_t RTOS_ISR_PRI_LVL = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;

  static constexpr std::uint32_t ADC_ISR_PRI_LVL        = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t CAN_ISR_PRI_LVL        = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t DMA_STREAM_ISR_PRI_LVL = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t EXTI_ISR_PRI_LVL       = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t I2C_ISR_PRI_LVL        = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t SDIO_ISR_PRI_LVL       = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t SPI_ISR_PRI_LVL        = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t TIMER_ISR_PRI_LVL      = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t UART_DMA_ISR_PRI_LVL   = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t UART_ISR_PRI_LVL       = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t USART_DMA_ISR_PRI_LVL  = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t USART_ISR_PRI_LVL      = RTOS_ISR_PRI_LVL;
  static constexpr std::uint32_t USB_ISR_PRI_LVL        = RTOS_ISR_PRI_LVL;

  static_assert( RTOS_ISR_PRI_LVL >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
  static_assert( RTOS_ISR_PRI_LVL <= configLIBRARY_LOWEST_INTERRUPT_PRIORITY );

}    // namespace Orbit

#endif /* !ORBIT_ESC_ISR_CONFIGURATION_HPP */
