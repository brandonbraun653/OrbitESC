/********************************************************************************
 *  File Name:
 *    board_map.hpp
 *
 *  Description:
 *    Maps system hardware resources for the OrbitESC PCB
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef ORBIT_ESC_BOARD_MAP_HPP
#define ORBIT_ESC_BOARD_MAP_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <Chimera/exti>
#include <Chimera/gpio>
#include <Chimera/i2c>
#include <Chimera/serial>


namespace Orbit::IO
{
  namespace GPIO
  {
    /*-------------------------------------------------------------------------
    Status LED
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinHeartbeat  = 6;
    static constexpr Chimera::GPIO::Port portHeartbeat = Chimera::GPIO::Port::PORTA;

    /*-------------------------------------------------------------------------
    User Button
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinButton  = 5;
    static constexpr Chimera::GPIO::Port portButton = Chimera::GPIO::Port::PORTB;
  }    // namespace GPIO

  namespace Analog
  {
    static constexpr Chimera::GPIO::PinInit CommonAnalogCfg = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                                .drive     = Chimera::GPIO::Drive::ANALOG,
                                                                .pin       = 0,
                                                                .port      = Chimera::GPIO::Port::UNKNOWN_PORT,
                                                                .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                                .state     = Chimera::GPIO::State::HIGH,
                                                                .threaded  = true,
                                                                .validity  = false };

    /*-------------------------------------------------------------------------
    Phase A
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinPhaseA  = 0;
    static constexpr Chimera::GPIO::Port   portPhaseA = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcPhaseA  = Chimera::ADC::Channel::ADC_CH_5;

    /*-------------------------------------------------------------------------
    Phase B
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinPhaseB  = 1;
    static constexpr Chimera::GPIO::Port   portPhaseB = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcPhaseB  = Chimera::ADC::Channel::ADC_CH_6;

    /*-----------------------------------------------------------------------------
    Phase C
    -----------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinPhaseC  = 4;
    static constexpr Chimera::GPIO::Port   portPhaseC = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcPhaseC  = Chimera::ADC::Channel::ADC_CH_9;

    /*-------------------------------------------------------------------------
    Three Phase Center Tap
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinCenterTap  = 5;
    static constexpr Chimera::GPIO::Port   portCenterTap = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcCenterTap  = Chimera::ADC::Channel::ADC_CH_10;

  }    // namespace Analog


  namespace CAN
  {

    /*-----------------------------------------------------------------------------
    GPIO
    -----------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinTX  = 12;
    static constexpr Chimera::GPIO::Port portTX = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  pinRX  = 11;
    static constexpr Chimera::GPIO::Port portRX = Chimera::GPIO::Port::PORTA;
  }    // namespace CAN


  namespace DBG
  {
    /*-------------------------------------------------------------------------
    Serial
    -------------------------------------------------------------------------*/
    static constexpr Chimera::Serial::Channel serialChannel = Chimera::Serial::Channel::SERIAL2;

    /*-------------------------------------------------------------------------
    GPIO
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  rxPin  = 3;
    static constexpr Chimera::GPIO::Port rxPort = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  txPin  = 2;
    static constexpr Chimera::GPIO::Port txPort = Chimera::GPIO::Port::PORTA;

    /*-------------------------------------------------------------------------
    Configuration Data
    -------------------------------------------------------------------------*/
    extern const Chimera::Serial::Config comConfig;
    extern const Chimera::GPIO::PinInit  txPinInit;
    extern const Chimera::GPIO::PinInit  rxPinInit;
  }    // namespace DBG


  namespace I2C
  {
    /*-------------------------------------------------------------------------
    GPIO
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  sdaPin  = 7;
    static constexpr Chimera::GPIO::Port sdaPort = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::Pin  sclPin  = 6;
    static constexpr Chimera::GPIO::Port sclPort = Chimera::GPIO::Port::PORTB;

    /*-------------------------------------------------------------------------
    I2C
    -------------------------------------------------------------------------*/
    static constexpr Chimera::I2C::Channel   channel   = Chimera::I2C::Channel::I2C1;
    static constexpr Chimera::I2C::Frequency frequency = Chimera::I2C::Frequency::F400KHZ;
  }    // namespace I2C


  namespace Timer
  {
    /*-------------------------------------------------------------------------
    Timer 1 Channel 1
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinT1Ch1   = 8;
    static constexpr Chimera::GPIO::Port portT1Ch1  = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  pinT1Ch1N  = 7;
    static constexpr Chimera::GPIO::Port portT1Ch1N = Chimera::GPIO::Port::PORTA;

    /*-------------------------------------------------------------------------
    Timer 1 Channel 2
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinT1Ch2   = 9;
    static constexpr Chimera::GPIO::Port portT1Ch2  = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  pinT1Ch2N  = 0;
    static constexpr Chimera::GPIO::Port portT1Ch2N = Chimera::GPIO::Port::PORTB;

    /*-------------------------------------------------------------------------
    Timer 1 Channel 3
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinT1Ch3   = 10;
    static constexpr Chimera::GPIO::Port portT1Ch3  = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  pinT1Ch3N  = 1;
    static constexpr Chimera::GPIO::Port portT1Ch3N = Chimera::GPIO::Port::PORTB;
  }    // namespace Timer

}    // namespace Orbit::IO

#endif /* !ORBIT_ESC_BOARD_MAP_HPP */
