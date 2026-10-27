/******************************************************************************
 *  File Name:
 *    test_isr_config.cpp
 *
 *  Description:
 *    Unit tests for interrupt priority configuration
 *
 *  2026 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#include <gtest/gtest.h>
#include <src/core/hw/orbit_isr_config.hpp>

// R1362
TEST( ISRConfig, FOC_DMA_UsesHighPriority )
{
  EXPECT_EQ( 1u, Orbit::FOC_DMA_ISR_PRI_LVL );
  EXPECT_EQ( Orbit::FOC_ISR_CHAIN_PRI_LVL, Orbit::FOC_DMA_ISR_PRI_LVL );
  EXPECT_LT( Orbit::FOC_DMA_ISR_PRI_LVL, Orbit::RTOS_ISR_PRI_LVL );
}


// R1363
TEST( ISRConfig, RTOSAwareInterruptsRespectSyscallPriorityBoundary )
{
  const auto syscallPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;

  EXPECT_GE( Orbit::ADC_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::CAN_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::DMA_STREAM_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::EXTI_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::I2C_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::SDIO_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::SPI_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::TIMER_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::UART_DMA_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::UART_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::USART_DMA_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::USART_ISR_PRI_LVL, syscallPriority );
  EXPECT_GE( Orbit::USB_ISR_PRI_LVL, syscallPriority );
}
