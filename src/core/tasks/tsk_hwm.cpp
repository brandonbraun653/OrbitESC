/******************************************************************************
 *  File Name:
 *    tsk_hwm.cpp
 *
 *  Description:
 *    Hardware manager task
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/thread>
#include <Chimera/can>
#include <src/core/tasks.hpp>


namespace Orbit::Tasks::HWM
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void HWMThread( void *arg )
  {
    /*-------------------------------------------------------------------------
    Wait for the start signal
    -------------------------------------------------------------------------*/
    waitInit();

    /*-------------------------------------------------------------------------
    Do something?
    -------------------------------------------------------------------------*/
    auto can = Chimera::CAN::getDriver( Chimera::CAN::Channel::CAN0 );
    RT_HARD_ASSERT( can );

    Chimera::CAN::BasicFrame msg;
    msg.clear();
    msg.id         = 0x33;
    msg.idMode     = Chimera::CAN::IdType::STANDARD;
    msg.frameType  = Chimera::CAN::FrameType::DATA;
    msg.dataLength = 5;
    msg.data[ 0 ]  = 0x44;
    msg.data[ 1 ]  = 0x55;
    msg.data[ 2 ]  = 0x66;
    msg.data[ 3 ]  = 0x77;
    msg.data[ 4 ]  = 0x88;

    while( 1 )
    {
      Chimera::delayMilliseconds( 500 );
      can->send( msg );
    }
  }
}  // namespace Orbit::Tasks::HWM
