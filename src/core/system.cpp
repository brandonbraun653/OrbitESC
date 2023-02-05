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
#include <Chimera/system>
#include <src/core/system.hpp>

namespace Orbit::System
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void doSafeShutdown()
  {
    // TODO: Add any other system shutdown tasks here
    Chimera::System::softwareReset( false );
  }
}    // namespace Orbit::System
