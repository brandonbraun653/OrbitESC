/******************************************************************************
 *  File Name:
 *    segger_stubs.cpp
 *
 *  Description:
 *    Stub functions for the Segger System View drivers
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#include <src/config/orbit_esc_cfg.hpp>

/*-----------------------------------------------------------------------------
Includes (Order Dependent)
-----------------------------------------------------------------------------*/
#if defined( SEGGER_SYS_VIEW )
#include "SEGGER_SYSVIEW.h"    // Must be first
#include "FreeRTOSConfig.h"    // Must be second
#include <Chimera/system>
#include <src/monitor/debug/segger_modules.hpp>

/*-----------------------------------------------------------------------------
Literal Constants
-----------------------------------------------------------------------------*/
#ifndef SYSVIEW_APP_NAME
#define SYSVIEW_APP_NAME "OrbitESC"
#endif

#ifndef SYSVIEW_DEVICE_NAME
#define SYSVIEW_DEVICE_NAME "Cortex-M4"
#endif

#ifndef SYSVIEW_TIMESTAMP_FREQ
#define SYSVIEW_TIMESTAMP_FREQ ( configCPU_CLOCK_HZ )
#endif

#ifndef SYSVIEW_CPU_FREQ
#define SYSVIEW_CPU_FREQ configCPU_CLOCK_HZ
#endif

#ifndef SYSVIEW_RAM_BASE
#define SYSVIEW_RAM_BASE ( 0x20000000 )
#endif


// Register definitions common to all Cortex-M4 devices
#define DEMCR ( ( volatile uint32_t * )( 0xE000EDFCuL ) )       // Debug Exception and Monitor Control Register
#define DWT_CTRL ( ( volatile uint32_t * )( 0xE0001000uL ) )    // DWT Control Register

/*-------------------------------------------------------------------------------
Disable SysView events. Cortex-M4 + JLink can only handle about 5k events/sec
before overflows start occurring. The mask disables enough items to get the event
rate to ~2.5k/s while gathering data to rebuild the task switching structure. This
should leave enough room for user profiling/debugging in the future.

Reasoning for Disabled:
SYSVIEW_EVTMASK_ALL_INTERRUPTS    | Project uses a large number of ISRs
SYSVIEW_EVTMASK_SYSTIME_CYCLES    | Irrelevant to build system timing info
SYSVIEW_EVTMASK_SYSTIME_US        | Irrelevant to build system timing info
SYSVIEW_EVTMASK_TIMER_ENTER       | Irrelevant to build system timing info
SYSVIEW_EVTMASK_TIMER_EXIT        | Irrelevant to build system timing info
SYSVIEW_EVTMASK_TASK_STOP_EXEC    | SysView can re-build task sequences without this
SYSVIEW_EVTMASK_TASK_START_READY  | SysView can re-build task sequences without this
SYSVIEW_EVTMASK_TASK_STOP_READY   | SysView can re-build task sequences without this
-------------------------------------------------------------------------------*/
#define SEGGER_DISABLE_MASK                                                                                                  \
  ( SYSVIEW_EVTMASK_SYSTIME_CYCLES | SYSVIEW_EVTMASK_SYSTIME_US | SYSVIEW_EVTMASK_TIMER_ENTER | SYSVIEW_EVTMASK_TIMER_EXIT | \
    SYSVIEW_EVTMASK_TASK_STOP_EXEC | SYSVIEW_EVTMASK_TASK_START_READY | SYSVIEW_EVTMASK_TASK_STOP_READY )

/**
 * @brief Disables interrupt & task scheduler & background events.
 *
 * These two event sources comprise the majority of traffic on the system. We still
 * want to be responsive to other events (like SYSVIEW_EVTMASK_INIT), so we leave
 * those enabled.
 */
#define SEGGER_CFG_MINIMAL_MASK ( SYSVIEW_EVTMASK_ALL_INTERRUPTS | SYSVIEW_EVTMASK_ALL_TASKS | SYSVIEW_EVTMASK_IDLE )

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
extern const SEGGER_SYSVIEW_OS_API SYSVIEW_X_OS_TraceAPI;

/*-----------------------------------------------------------------------------
Static Data
-----------------------------------------------------------------------------*/
static Chimera::System::InterruptMask s_isr_mask;

/*-----------------------------------------------------------------------------
Public Functions
-----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief Callback function for sending system description information
   * @return void
   */
  static void _cbSendSystemDesc( void )
  {
    SEGGER_SYSVIEW_SendSysDesc( "N=" SYSVIEW_APP_NAME ",O=FreeRTOS,D=STM32F446RE,C=" SYSVIEW_DEVICE_NAME );
    SEGGER_SYSVIEW_SendSysDesc( "I#15=SysTick" );
  }


  void SEGGER_SYSVIEW_Conf( void )
  {
    constexpr auto TRACEENA_BIT  = ( 1u << 24 );    // Trace enable bit
    constexpr auto NOCYCCNT_BIT  = ( 1u << 25 );    // Cycle counter support bit
    constexpr auto CYCCNTENA_BIT = ( 1u << 0 );     // Cycle counter enable bit

    /*-------------------------------------------------------------------------
    Enable trace and debug blocks. If it's already enabled, this does nothing.
    -------------------------------------------------------------------------*/
    if( ( *DEMCR & TRACEENA_BIT ) == 0 )
    {
      *DEMCR |= TRACEENA_BIT;
    }

    /*-------------------------------------------------------------------------
    Enable the cycle counter if it's supported and not already enabled.
    -------------------------------------------------------------------------*/
    if( ( ( *DWT_CTRL & NOCYCCNT_BIT ) == 0 ) && ( ( *DWT_CTRL & CYCCNTENA_BIT ) == 0 ) )
    {
      *DWT_CTRL |= CYCCNTENA_BIT;
    }

    /*-------------------------------------------------------------------------
    Initialize the system view module
    -------------------------------------------------------------------------*/
    SEGGER_SYSVIEW_Init( SYSVIEW_TIMESTAMP_FREQ, SYSVIEW_CPU_FREQ, &SYSVIEW_X_OS_TraceAPI, _cbSendSystemDesc );
    SEGGER_SYSVIEW_SetRAMBase( SYSVIEW_RAM_BASE );
    SEGGER_SYSVIEW_DisableEvents( SEGGER_DISABLE_MASK );

    /*-------------------------------------------------------------------------
    Register all project modules
    -------------------------------------------------------------------------*/
    Orbit::Monitor::Segger::initialize();
    Orbit::Monitor::Segger::registerModules();

    for( uint8_t x = 0; x < Orbit::Monitor::Segger::SeggerModuleID::NumModules; x++ )
    {
      SEGGER_SYSVIEW_SendModule( x );
    }

    SEGGER_SYSVIEW_SendNumModules();
    SEGGER_SYSVIEW_SendModuleDescription();
  }


  void SEGGER_RTT_Lock_Prj()
  {
    s_isr_mask = Chimera::System::disableInterrupts();
  }

  void SEGGER_RTT_Unlock_Prj()
  {
    Chimera::System::enableInterrupts( s_isr_mask );
  }

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  /* SEGGER_SYS_VIEW */
