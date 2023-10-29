/******************************************************************************
 *  File Name:
 *    segger_modules.cpp
 *
 *  Description:
 *    Segger module declarations
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <Chimera/utility>
#include <src/monitor/debug/segger_modules_intf.h>
#include <src/monitor/debug/segger_modules.hpp>

namespace Orbit::Monitor::Segger
{
  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/

  SEGGER_SYSVIEW_MODULE OSM[ SeggerModuleID::NumModules ];


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void initialize( void )
  {
    OSM[ SeggerModuleID::TUSB_ID ] = { .sModule          = "M=TinyUSB, T=tusb, S='TinyUSB'",
                                       .NumEvents        = OrbitMonitor_TUSB_NumEvents,
                                       .EventOffset      = 0,
                                       .pfSendModuleDesc = nullptr,
                                       .pNext            = nullptr };
  }


  void registerModules( void )
  {
    for( size_t x = 0; x < SeggerModuleID::NumModules; x++ )
    {
      SEGGER_SYSVIEW_RegisterModule( &OSM[ x ] );
    }
  }
}    // namespace Orbit::Monitor::Segger

/*-----------------------------------------------------------------------------
Segger Modules Interface
-----------------------------------------------------------------------------*/
using namespace Orbit::Monitor::Segger;

extern "C" void OrbitMonitorRecordEvent_TUSB( const OrbitMonitor_TUSB_Events event )
{
  SEGGER_SYSVIEW_RecordVoid( event + OSM[ SeggerModuleID::TUSB_ID ].EventOffset );
}
