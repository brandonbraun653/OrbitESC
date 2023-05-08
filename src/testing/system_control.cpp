/******************************************************************************
 *  File Name:
 *    system_control.cpp
 *
 *  Description:
 *    System control implementation for testing purposes
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/testing/system_control.hpp>
#include <src/core/hw/orbit_motor.hpp>

namespace Orbit::Testing::SystemControl
{
  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool handleMessage( const Serial::Message::SysCtrl &msg )
  {
    if( msg.payload.has_motorCmd )
    {
      switch( msg.payload.motorCmd )
      {
        case MotorCtrlCmd_EMERGENCY_STOP:
          Motor::emergencyStop();
          return true;

        default:
          // Do nothing
          break;
      }
    }

    /*-------------------------------------------------------------------------
    Default fall-through case
    -------------------------------------------------------------------------*/
    return false;
  }
}    // namespace Orbit::Testing::SystemControl
