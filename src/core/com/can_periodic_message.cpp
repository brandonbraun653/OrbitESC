/******************************************************************************
 *  File Name:
 *    can_periodic_message.cpp
 *
 *  Description:
 *    Implementation details for periodic CAN messages
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <src/config/bsp/board_map.hpp>
#include <src/control/foc_driver.hpp>
#include <src/control/foc_math.hpp>
#include <src/core/com/can_message.hpp>
#include <src/core/com/can_periodic_message.hpp>
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
    payload.hdr.nodeId = EnumValue( thisNode() );
    this->send( CANDriver );
  }

  /*---------------------------------------------------------------------------
  System Mode Update
  ---------------------------------------------------------------------------*/
  void SystemMode::update()
  {
    payload.mode       = Orbit::Control::FOCDriver.currentMode();
    payload.hdr.nodeId = EnumValue( thisNode() );
    this->send( CANDriver );
  }

  /*---------------------------------------------------------------------------
  Power Supply Voltage Update
  ---------------------------------------------------------------------------*/
  void PowerSupplyVoltage::update()
  {
    Orbit::Control::ADCSensorBuffer buffer;
    Orbit::Control::FOCDriver.lastSensorData( buffer );

    payload.vdd        = static_cast<uint16_t>( buffer[ Control::ADC_CH_MOTOR_SUPPLY_VOLTAGE ].converted * 1e3f );
    payload.timestamp  = buffer[ Control::ADC_CH_MOTOR_SUPPLY_VOLTAGE ].sampleTimeUs;
    payload.hdr.nodeId = EnumValue( thisNode() );
    this->send( CANDriver );
  }

  /*---------------------------------------------------------------------------
  Phase A Current Update
  ---------------------------------------------------------------------------*/
  void PhaseACurrent::update()
  {
    Orbit::Control::ADCSensorBuffer buffer;
    Orbit::Control::FOCDriver.lastSensorData( buffer );

    payload.current    = static_cast<uint16_t>( buffer[ Control::ADC_CH_MOTOR_PHASE_A_CURRENT ].converted * 1e3f );
    payload.timestamp  = buffer[ Control::ADC_CH_MOTOR_PHASE_A_CURRENT ].sampleTimeUs;
    payload.hdr.nodeId = EnumValue( thisNode() );
    this->send( CANDriver );
  }

  /*---------------------------------------------------------------------------
  Phase B Current Update
  ---------------------------------------------------------------------------*/
  void PhaseBCurrent::update()
  {
    Orbit::Control::ADCSensorBuffer buffer;
    Orbit::Control::FOCDriver.lastSensorData( buffer );

    payload.current    = static_cast<uint16_t>( buffer[ Control::ADC_CH_MOTOR_PHASE_B_CURRENT ].converted * 1e3f );
    payload.timestamp  = buffer[ Control::ADC_CH_MOTOR_PHASE_B_CURRENT ].sampleTimeUs;
    payload.hdr.nodeId = EnumValue( thisNode() );
    this->send( CANDriver );
  }

  /*---------------------------------------------------------------------------
  Motor Speed Update
  ---------------------------------------------------------------------------*/
  void MotorSpeed::update()
  {
    const float speed_in_rad = Orbit::Control::FOCDriver.dbgGetState().motorController.velEstRad;

    payload.speed      = static_cast<uint16_t>( RAD_TO_RPM( speed_in_rad ) );
    payload.tick       = Chimera::millis();
    payload.hdr.nodeId = EnumValue( thisNode() );
    this->send( CANDriver );
  }

  /*---------------------------------------------------------------------------
  Speed Reference Update
  ---------------------------------------------------------------------------*/
  void SpeedReference::update()
  {
    const float speed_in_rad = Orbit::Control::FOCDriver.dbgGetState().motorController.run.speedRefRad;

    payload.speed      = static_cast<uint16_t>( RAD_TO_RPM( speed_in_rad ) );
    payload.tick       = Chimera::millis();
    payload.hdr.nodeId = EnumValue( thisNode() );
    this->send( CANDriver );
  }

}    // namespace Orbit::CAN::Message
