/******************************************************************************
 *  File Name:
 *    core_ctrl_router.cpp
 *
 *  Description:
 *    Handler for core system control messages
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <src/core/events/routers/router_types.hpp>
#include <src/core/system.hpp>
#include <src/core/com/com_app_tx.hpp>


namespace Orbit::Event
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  CoreControlRouter::CoreControlRouter() : message_router( ROUTER_CORE_CONTROL )
  {
  }


  void CoreControlRouter::on_receive( const SystemReset &msg )
  {
    Chimera::delayMilliseconds( 50 );
    Orbit::System::setMode( Orbit::System::getMode() );
  }


  void CoreControlRouter::on_receive( const StreamPhaseCurrents &msg )
  {
    using namespace Orbit::COM;
    enableStream( STREAM_ID_PHASE_CURRENTS, msg.enable );
  }

  void CoreControlRouter::on_receive_unknown( const etl::imessage &msg )
  {
    /* Do nothing */
  }
}  // namespace Orbit::Event
