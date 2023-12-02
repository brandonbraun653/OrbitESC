/******************************************************************************
 *  File Name:
 *    system.cpp
 *
 *  Description:
 *    High level system control implementation
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/system>
#include <src/core/system.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/data/volatile/orbit_parameter.hpp>
#include <src/core/hw/orbit_led.hpp>

namespace Orbit::System
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void doSafeShutdown()
  {
    LOG_DEBUG( "Performing safe shutdown of the system" );

    /*-------------------------------------------------------------------------
    Wait for the DIO thread to sync the disk cache
    -------------------------------------------------------------------------*/
    LOG_DEBUG( "Waiting for cache to sync to disk" );
    while ( !Data::Param::synchronized() )
    {
      Chimera::delayMilliseconds( 100 );
    }

    /*-------------------------------------------------------------------------
    Reset the system
    -------------------------------------------------------------------------*/
    LOG_INFO( "Reseting the system" );
    Chimera::System::softwareReset( false );
  }


  void addFaultEvent( const Fault f, const std::string_view &msg )
  {
    // TODO
    LED::setChannel( LED::Channel::FAULT );
  }


  FaultLogEntry peekFaultLog()
  {
    // TODO
    return {};
  }


  void popFaultLog()
  {
    // TODO
  }

}    // namespace Orbit::System
