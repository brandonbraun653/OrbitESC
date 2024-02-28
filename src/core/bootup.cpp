/******************************************************************************
 *  File Name:
 *    bootup.cpp
 *
 *  Description:
 *    Device driver power up procedures
 *
 *  2022-2024 | Brandon Braun | brandonbraun653@protonmail.com
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
#include <src/control/foc_driver.hpp>
#include <src/control/subroutines/interface.hpp>
#include <src/core/bootup.hpp>
#include <src/core/com/com_scheduler.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/orbit_log_io.hpp>
#include <src/core/events/event_stream.hpp>
#include <src/core/hw/orbit_adc.hpp>
#include <src/core/hw/orbit_can.hpp>
#include <src/core/hw/orbit_gpio.hpp>
#include <src/core/hw/orbit_instrumentation.hpp>
#include <src/core/hw/orbit_led.hpp>
#include <src/core/hw/orbit_sdio.hpp>
#include <src/core/hw/orbit_spi.hpp>
#include <src/core/hw/orbit_timer.hpp>
#include <src/core/hw/orbit_usart.hpp>
#include <src/core/tasks.hpp>
#include <src/monitor/orbit_metrics.hpp>
#include <src/monitor/orbit_monitors.hpp>

#if defined( EMBEDDED )
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/inc/flash>
#include <Thor/lld/interface/inc/flash>
#include <Thor/lld/interface/inc/power>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_prv.hpp>
#endif /* EMBEDDED */


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
#if defined( EMBEDDED )
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
#endif
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUpSystemDrivers()
  {
    /*-------------------------------------------------------------------------
    Power up high level system controls
    -------------------------------------------------------------------------*/
    setSystemBehavior();

    /*-------------------------------------------------------------------------
    Power up the HW peripherals supporting the file system (ORDER MATTERS!)
    -------------------------------------------------------------------------*/
    Orbit::USART::powerUp();           // Serial debug port logging
    Orbit::SPI::powerUp();             // NOR bus driver
    Orbit::SDIO::powerUp();            // SD card driver
    Orbit::Data::initialize();         // Prepare system data memory
    Orbit::Data::printSystemInfo();    // Print the system info to the console

    /*-------------------------------------------------------------------------
    Power up the file logging system as early as possible to catch any errors
    -------------------------------------------------------------------------*/
    Log::initialize();
    Log::enable();

    /*-------------------------------------------------------------------------
    Power up the peripherals with re-configurable settings
    -------------------------------------------------------------------------*/
    Orbit::ADC::powerUp();
    Orbit::CAN::powerUp();
    Orbit::TIMER::powerUp();

    /*-------------------------------------------------------------------------
    Power up remaining system components
    -------------------------------------------------------------------------*/
    Orbit::GPIO::powerUp();
    Orbit::LED::powerUp();
    Orbit::Instrumentation::powerUp();
    Orbit::COM::Scheduler::initialize();
    Orbit::Monitor::initMetrics();
    Orbit::Event::initialize();
    Orbit::Control::FOC::initialize();
    Orbit::Control::Subroutine::initialize();
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

    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_IDLE ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_HWM ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_USB ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
    Chimera::delayMilliseconds( 500 );

    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_CDC ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
    Chimera::delayMilliseconds( 15 );

    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_COM ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_CTL ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
    Chimera::delayMilliseconds( 25 );

#if defined( SIMULATOR )
    RT_HARD_ASSERT( true == sendTaskMsg( Tasks::getTaskId( Tasks::TASK_SIM ), TSK_MSG_WAKEUP, TIMEOUT_BLOCK ) );
#endif
  }

}    // namespace Orbit::Boot


#if defined( EMBEDDED )
namespace Thor::LLD::RCC
{
  /**
   * @brief Project configuration of the clock tree.
   *
   * Uses the external osciallator as the PLL source and drives the system
   * clock at 180MHz. I'm essentially pushing the performance to the max for
   * this chip.
   */
  void configureProjectClocks()
  {
    constexpr size_t hseClkIn     = 24'000'000;
    constexpr size_t targetSysClk = 180'000'000;
    constexpr size_t targetUSBClk = 48'000'000;

    /*-------------------------------------------------------------------------
    Notify RCC module of external clocks
    -------------------------------------------------------------------------*/
    cacheExtOscFreq( Chimera::Clock::Bus::HSE, hseClkIn );

    /*-------------------------------------------------------------------------
    Set flash latency to a safe value for all possible clocks. This will slow
    down the configuration, but this is only performed once at startup.
    -------------------------------------------------------------------------*/
    FLASH::setLatency( 15 );

    /*-------------------------------------------------------------------------
    Not strictly necessary, but done because this config function uses the max
    system clock.
    -------------------------------------------------------------------------*/
    PWR::setOverdriveMode( true );

    /*-------------------------------------------------------------------------
    Configure the system clocks to max performance
    -------------------------------------------------------------------------*/
    ClockTreeInit clkCfg;
    clkCfg.clear();

    /* Select which clocks to turn on  */
    clkCfg.enabled.hsi          = true;    // Needed for transfer of clock source
    clkCfg.enabled.hse          = true;    // Needed for PLL source
    clkCfg.enabled.lsi          = true;    // Allows IWDG use
    clkCfg.enabled.pll_core_clk = true;    // Will drive sys off PLL
    clkCfg.enabled.pll_sai_p    = true;    // USB 48 MHz clock

    /* Select clock mux routing */
    clkCfg.mux.pll   = Chimera::Clock::Bus::HSE;
    clkCfg.mux.sys   = Chimera::Clock::Bus::PLLP;
    clkCfg.mux.usb48 = Chimera::Clock::Bus::PLLSAI_P;
    clkCfg.mux.sdio  = Chimera::Clock::Bus::CK48;

    /* Divisors from the system clock */
    clkCfg.prescaler.ahb  = 1;
    clkCfg.prescaler.apb1 = 4;
    clkCfg.prescaler.apb2 = 2;

    /* Core PLL configuration settings */
    clkCfg.PLLCore.M = 24;
    clkCfg.PLLCore.N = 360;
    clkCfg.PLLCore.P = 2;
    clkCfg.PLLCore.Q = 8;    // Doesn't matter, not used
    clkCfg.PLLCore.R = 7;    // Doesn't matter, not used

    /* SAI PLL configuration settings */
    clkCfg.PLLSAI.M = 16;
    clkCfg.PLLSAI.N = 128;
    clkCfg.PLLSAI.P = 4;
    clkCfg.PLLSAI.Q = 2;    // Doesn't matter, not used

    RT_HARD_ASSERT( configureClockTree( clkCfg ) );

    /*-------------------------------------------------------------------------
    Make sure the rest of the system knows about the new clock frequency.
    -------------------------------------------------------------------------*/
    const size_t sys_clk = getSystemClock();
    RT_HARD_ASSERT( sys_clk == targetSysClk );

    CortexM4::Clock::updateCoreClockCache( sys_clk );

    /*-------------------------------------------------------------------------
    Trim the flash latency back to a performant range now that the high speed
    clock has been configured.
    -------------------------------------------------------------------------*/
    FLASH::setLatency( FLASH::LATENCY_AUTO_DETECT );

    /*-------------------------------------------------------------------------
    Verify the user's target clocks have been achieved
    -------------------------------------------------------------------------*/
    auto rcc = getCoreClockCtrl();
    RT_HARD_ASSERT( targetUSBClk == rcc->getClockFrequency( Chimera::Clock::Bus::PLLSAI_P ) );
  }
}    // namespace Thor::LLD::RCC
#endif /* EMBEDDED */
