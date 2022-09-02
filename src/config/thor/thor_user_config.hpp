/********************************************************************************
 *  File Name:
 *    thor_user_config.hpp
 *
 *  Description:
 *    Enables only the necessary peripherals for configuring Thor
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef DC_THOR_PERIPHERAL_CONFIGURATION_HPP
#define DC_THOR_PERIPHERAL_CONFIGURATION_HPP

/*-----------------------------------------------------------------------------
Peripheral Support
-----------------------------------------------------------------------------*/
#define DC_NEED_ADC   1
#define DC_NEED_CAN   1
#define DC_NEED_DMA   1
#define DC_NEED_EXTI  1
#define DC_NEED_GPIO  1
#define DC_NEED_I2C   1
#define DC_NEED_INT   1
#define DC_NEED_SPI   1
#define DC_NEED_TIMER 1
#define DC_NEED_UART  1
#define DC_NEED_USART 1
#define DC_NEED_WDG   1

/*-----------------------------------------------------------------------------
Configure High/Low level driver pairings
-----------------------------------------------------------------------------*/
#if DC_NEED_ADC
#define THOR_ADC
#endif

#if DC_NEED_CAN
#define THOR_CAN
#endif

#if DC_NEED_DMA
#define THOR_DMA
#endif

#if DC_NEED_EXTI
#define THOR_EXTI
#endif

#if DC_NEED_GPIO
#define THOR_GPIO
#endif

#if DC_NEED_I2C
#define THOR_I2C
#endif

#if DC_NEED_INT
#define THOR_INT
#endif

#if DC_NEED_SPI
#define THOR_SPI
#endif

#if DC_NEED_TIMER
#define THOR_TIMER
#endif

#if DC_NEED_UART
#define THOR_UART
#endif

#if DC_NEED_USART
#define THOR_USART
#endif

#if DC_NEED_WDG
#define THOR_IWDG
#define THOR_WWDG
#endif

#ifndef THOR_CLK
#define THOR_CLK
#endif

#ifndef THOR_SYSTEM
#define THOR_SYSTEM
#endif

/*-----------------------------------------------------------------------------
Explicit Low Level Driver Support
-----------------------------------------------------------------------------*/
#ifndef THOR_DES
#define THOR_DES
#endif

#ifndef THOR_FLASH
#define THOR_FLASH
#endif

#ifndef THOR_PWR
#define THOR_PWR
#endif

#ifndef THOR_RCC
#define THOR_RCC
#endif

#ifndef THOR_SYSCFG
#define THOR_SYSCFG
#endif

#endif /* !DC_THOR_PERIPHERAL_CONFIGURATION_HPP */
