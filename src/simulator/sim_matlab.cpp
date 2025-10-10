/******************************************************************************
 *  File Name:
 *    sim_matlab.cpp
 *
 *  Description:
 *    Matlab simulator callback implementations
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/simulator/sim_matlab.hpp>
#include <src/control/foc_driver.hpp>

namespace Orbit::Sim::Matlab
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Motor simulation data from Matlab simulation
   */
  struct MotorData
  {
    float va; /**< Phase A voltage in Volts */
    float vb; /**< Phase B voltage in Volts */
    float vc; /**< Phase C voltage in Volts */
    float ia; /**< Phase A current in Amps */
    float ib; /**< Phase B current in Amps */
    float ic; /**< Phase C current in Amps */
  };

  /**
   * @brief System state control from Matlab simulation
   *
   * These are transitions/events/references that normally come from the
   * flight computer, representing input from a human via some interface
   * like a joystick or a button.
   */
  struct ControlData
  {
    bool  armed;         /**< System should be armed */
    bool  engaged;       /**< System should be engaged */
    float speed_ref_rpm; /**< Speed reference in rpm */

    ControlData()
    {
      armed         = false;
      engaged       = false;
      speed_ref_rpm = 0.0f;
    }
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void motorSimulationCallback( Orbit::Sim::TCP::Server &server, const void *data, size_t size )
  {
    if( !server.isClientConnected() )
    {
      return;
    }

    // accept simulation data, inject into motor model
  }

  void escControlCallback( Orbit::Sim::TCP::Server &server, const void *data, size_t size )
  {
    static ControlData prev_cmd = {};

    if( !server.isClientConnected() )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Parse the received data
    -------------------------------------------------------------------------*/
    ControlData new_cmd;
    if( size != sizeof( ControlData ) )
    {
      return;
    }

    memcpy( &new_cmd, data, sizeof( ControlData ) );

    /*-------------------------------------------------------------------------
    Activate motor controller
    -------------------------------------------------------------------------*/
    if( new_cmd.armed && !prev_cmd.armed )
    {
      Control::FOC::sendSystemEvent( Control::EventId::ARM );
    }
    if( new_cmd.engaged && !prev_cmd.engaged )
    {
      Control::FOC::sendSystemEvent( Control::EventId::ENGAGE );
    }

    prev_cmd = new_cmd;
  }
}    // namespace Orbit::Sim::Matlab
