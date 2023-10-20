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

/*-----------------------------------------------------------------------------
Static Functions
-----------------------------------------------------------------------------*/
static void send_desc_TUSB();


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
    OSM[ SeggerModuleID::TUSB_ID ] = { .sModule          = "TinyUSB",
                                       .NumEvents        = OrbitMonitor_TUSB_NumEvents,
                                       .EventOffset      = 0,
                                       .pfSendModuleDesc = send_desc_TUSB,
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


static void send_desc_TUSB()
{
  static const char *const desc_strings[ OrbitMonitor_TUSB_NumEvents ] = {
    "0 Init",
  };

  SEGGER_SYSVIEW_RecordModuleDescription( &OSM[ SeggerModuleID::TUSB_ID ], "T=tusb, S='TinyUSB'" );
  for( size_t x = 0; x < ARRAY_COUNT( desc_strings ); x++ )
  {
    SEGGER_SYSVIEW_RecordModuleDescription( &OSM[ SeggerModuleID::TUSB_ID ], desc_strings[ x ] );
  }
}


extern "C" void OrbitMonitorRecordEvent_TUSB( const OrbitMonitor_TUSB_Events event )
{
  SEGGER_SYSVIEW_RecordVoid( event + OSM[ SeggerModuleID::TUSB_ID ].EventOffset );
}
