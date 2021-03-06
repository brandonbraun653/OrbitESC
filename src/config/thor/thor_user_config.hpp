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
#define DC_NEED_TIMER 1
#define DC_NEED_USART 1
#define DC_NEED_WDG   1

/*-----------------------------------------------------------------------------
Configure High/Low level driver pairings
-----------------------------------------------------------------------------*/
#if DC_NEED_ADC
#define THOR_HLD_ADC
#define THOR_LLD_ADC
#endif

#if DC_NEED_CAN
#define THOR_HLD_CAN
#define THOR_LLD_CAN
#endif

#if DC_NEED_DMA
#define THOR_HLD_DMA
#define THOR_LLD_DMA
#endif

#if DC_NEED_EXTI
#define THOR_HLD_EXTI
#define THOR_LLD_EXTI
#endif

#if DC_NEED_GPIO
#define THOR_HLD_GPIO
#define THOR_LLD_GPIO
#endif

#if DC_NEED_I2C
#define THOR_HLD_I2C
#define THOR_LLD_I2C
#endif

#if DC_NEED_INT
#define THOR_HLD_INT
#define THOR_LLD_INT
#endif

#if DC_NEED_TIMER
#define THOR_HLD_TIMER
#define THOR_LLD_TIMER
#endif

#if DC_NEED_USART
#define THOR_HLD_USART
#define THOR_LLD_USART
#endif

#if DC_NEED_WDG
#define THOR_HLD_IWDG
#define THOR_HLD_WWDG
#define THOR_LLD_IWDG
#define THOR_LLD_WWDG
#endif

#ifndef THOR_HLD_CLK
#define THOR_HLD_CLK
#endif

#ifndef THOR_HLD_SYSTEM
#define THOR_HLD_SYSTEM
#endif

/*-----------------------------------------------------------------------------
Explicit Low Level Driver Support
-----------------------------------------------------------------------------*/
#ifndef THOR_LLD_DES
#define THOR_LLD_DES
#endif

#ifndef THOR_LLD_FLASH
#define THOR_LLD_FLASH
#endif

#ifndef THOR_LLD_PWR
#define THOR_LLD_PWR
#endif

#ifndef THOR_LLD_RCC
#define THOR_LLD_RCC
#endif

#ifndef THOR_LLD_SYSCFG
#define THOR_LLD_SYSCFG
#endif

#endif /* !DC_THOR_PERIPHERAL_CONFIGURATION_HPP */
