/******************************************************************************
 *  File Name:
 *    can_message.cpp
 *
 *  Description:
 *    Implementation details for CAN messages
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <src/config/bsp/board_map.hpp>
#include <src/control/foc_driver.hpp>
#include <src/core/com/can_message.hpp>
#include <src/core/hw/orbit_can.hpp>
#include <src/core/runtime/can_runtime.hpp>

namespace Orbit::CAN::Message
{
  /*---------------------------------------------------------------------------
  System Tick Update
  ---------------------------------------------------------------------------*/
  void SystemTick::update()
  {
    payload.tick       = static_cast<uint32_t>( Chimera::millis() );
    payload.hdr.nodeId = thisNode();
    this->send( CANDriver );
  }


  /*---------------------------------------------------------------------------
  Power Supply Voltage Update
  ---------------------------------------------------------------------------*/
  void PowerSupplyVoltage::update()
  {
    Orbit::Control::ADCData adc;
    Orbit::Control::FOCDriver.lastADCData( adc );

    payload.vdd        = static_cast<uint32_t>( adc.supplyVoltage * 1e6f );
    payload.hdr.nodeId = thisNode();
    this->send( CANDriver );
  }
}    // namespace Orbit::CAN::Message
