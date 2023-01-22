/******************************************************************************
 *  File Name:
 *    bootup.cpp
 *
 *  Description:
 *    Device driver power up procedures
 *
 *  2022-2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/adc>
#include <Chimera/assert>
#include <Chimera/gpio>
#include <Chimera/serial>
#include <etl/circular_buffer.h>
#include <src/config/bsp/board_map.hpp>
#include <src/core/bootup.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/hw/orbit_adc.hpp>
#include <src/core/hw/orbit_can.hpp>
#include <src/core/hw/orbit_gpio.hpp>
#include <src/core/hw/orbit_i2c.hpp>
#include <src/core/hw/orbit_led.hpp>
#include <src/core/hw/orbit_spi.hpp>
#include <src/core/hw/orbit_timer.hpp>
#include <src/core/hw/orbit_usart.hpp>
#include <src/core/tasks.hpp>
#include <src/monitor/orbit_monitors.hpp>


namespace Orbit::Boot
{
  /*---------------------------------------------------------------------------
  Local Literals
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t DBGMCU_APB2FZR_ADDR  = 0xE0042010;
  static constexpr uint32_t DBGMCU_TIM1_STOP_EN  = 1u << 11;
  static constexpr uint32_t DBGMCU_TIM15_STOP_EN = 1u << 16;

  static constexpr uint32_t SYSCFG_CFGR2_ADDR   = 0x4001001C;
  static constexpr uint32_t SYSCFG_CLL_BREAK_EN = 1u << 0;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Controls the behavior of several peripherals
   *
   * The goal here is to ensure the ADC/Timer/DMA peripherals keep running
   * during CPU halts or will detect faults appropriately to prevent damaging
   * the power stage of the ESC.
   */
  static void setSystemBehavior()
  {
    /*-------------------------------------------------------------------------
    Connect the CM4 lockup bit to the break input of TIM1. This should place
    the 3-phase inverter into a safe state on hard-faults.
    -------------------------------------------------------------------------*/
    *( ( uint32_t * )SYSCFG_CFGR2_ADDR ) |= SYSCFG_CLL_BREAK_EN;

    /*-------------------------------------------------------------------------
    Set the motor control timer to always execute. ADC/DMA are triggered by the
    timer and will continue indefinitely.
    -------------------------------------------------------------------------*/
    *( ( uint32_t * )DBGMCU_APB2FZR_ADDR ) &= ~DBGMCU_TIM1_STOP_EN;
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUpSystemDrivers()
  {
    /*-------------------------------------------------------------------------
    Power up high level system controls
    -------------------------------------------------------------------------*/
#if defined( EMBEDDED )
    setSystemBehavior();
#endif

    /*-------------------------------------------------------------------------
    Power up the hardware peripherals
    -------------------------------------------------------------------------*/
    Orbit::ADC::powerUp();
    Orbit::CAN::powerUp();
    Orbit::GPIO::powerUp();
    Orbit::I2C::powerUp();
#if defined( ORBIT_ESC_V1 )
    Orbit::SPI::powerUp();
#endif
    Orbit::TIMER::powerUp();
    Orbit::USART::powerUp();

    /*-------------------------------------------------------------------------
    Power up more complex system components
    -------------------------------------------------------------------------*/
    Orbit::Data::initialize();
#if defined( ORBIT_ESC_V1 )
    Orbit::LED::powerUp();
#endif
    Orbit::Monitor::initialize();
  }


  void startTasks()
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    On the simulator, we need a bit more time to allow tasks to fully register
    -------------------------------------------------------------------------*/
#if defined( SIMULATOR )
    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
#endif

    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_HWM ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_COM ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_DIO ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_CTL ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
  }

}    // namespace Orbit::Boot
