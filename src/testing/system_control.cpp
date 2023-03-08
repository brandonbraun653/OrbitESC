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

namespace Orbit::Testing::SystemControl
{
  bool handleMessage( const Serial::Message::SysCtrl &msg )
  {
    return true;
  }
}    // namespace Orbit::Testing::SystemControl
