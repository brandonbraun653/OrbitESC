/********************************************************************************
 *  File Name:
 *    board_map_v1.hpp
 *
 *  Description:
 *    Maps system hardware resources for the OrbitESC PCB Version 2.0
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef ORBIT_ESC_BOARD_MAP_V2_HPP
#define ORBIT_ESC_BOARD_MAP_V2_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <Chimera/can>
#include <Chimera/exti>
#include <Chimera/gpio>
#include <Chimera/i2c>
#include <Chimera/serial>
#include <Chimera/spi>

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
#define ORBIT_ESC_V3

/*-----------------------------------------------------------------------------
Configuration
-----------------------------------------------------------------------------*/
namespace Orbit::IO
{
  namespace Analog
  {
    static constexpr Chimera::ADC::Peripheral peripheral      = Chimera::ADC::Peripheral::ADC_0;
    static constexpr Chimera::GPIO::PinInit   CommonAnalogCfg = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                                  .drive     = Chimera::GPIO::Drive::ANALOG,
                                                                  .pin       = 0,
                                                                  .port      = Chimera::GPIO::Port::UNKNOWN_PORT,
                                                                  .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                                  .state     = Chimera::GPIO::State::HIGH,
                                                                  .threaded  = true,
                                                                  .validity  = false };

    /*-------------------------------------------------------------------------
    Supply Voltage
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinVSupply  = 5;
    static constexpr Chimera::GPIO::Port   portVSupply = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcVSupply  = Chimera::ADC::Channel::ADC_CH_10;

    /*-------------------------------------------------------------------------
    Phase A Current
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinPhaseA  = 4;
    static constexpr Chimera::GPIO::Port   portPhaseA = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcPhaseA  = Chimera::ADC::Channel::ADC_CH_9;

    /*-------------------------------------------------------------------------
    Phase B Current
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinPhaseB  = 1;
    static constexpr Chimera::GPIO::Port   portPhaseB = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcPhaseB  = Chimera::ADC::Channel::ADC_CH_6;

    /*-------------------------------------------------------------------------
    Phase C Current
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinPhaseC  = 0;
    static constexpr Chimera::GPIO::Port   portPhaseC = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcPhaseC  = Chimera::ADC::Channel::ADC_CH_5;
  }    // namespace Analog

  namespace Digital
  {
    /*-------------------------------------------------------------------------
    Debug Output Pins
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin     dbg1Pin     = 4;
    static constexpr Chimera::GPIO::Port    dbg1Port    = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::GPIO::PinInit dbg1PinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                            .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                                                            .pin       = dbg1Pin,
                                                            .port      = dbg1Port,
                                                            .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                            .state     = Chimera::GPIO::State::LOW,
                                                            .threaded  = false,
                                                            .validity  = true };

    static constexpr Chimera::GPIO::Pin     dbg2Pin     = 5;
    static constexpr Chimera::GPIO::Port    dbg2Port    = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::GPIO::PinInit dbg2PinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                            .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                                                            .pin       = dbg2Pin,
                                                            .port      = dbg2Port,
                                                            .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                            .state     = Chimera::GPIO::State::LOW,
                                                            .threaded  = false,
                                                            .validity  = true };
    /*-------------------------------------------------------------------------
    LED Pins
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin     ledArmedPin     = 6;
    static constexpr Chimera::GPIO::Port    ledArmedPort    = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::PinInit ledArmedPinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                                .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                                                                .pin       = ledArmedPin,
                                                                .port      = ledArmedPort,
                                                                .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                                .state     = Chimera::GPIO::State::HIGH,
                                                                .threaded  = false,
                                                                .validity  = true }; 
                                                                                    
    static constexpr Chimera::GPIO::Pin     ledHeartbeatPin     = 7;
    static constexpr Chimera::GPIO::Port    ledHeartbeatPort    = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::PinInit ledHeartbeatPinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                                    .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                                                                    .pin       = ledHeartbeatPin,
                                                                    .port      = ledHeartbeatPort,
                                                                    .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                                    .state     = Chimera::GPIO::State::HIGH,
                                                                    .threaded  = false,
                                                                    .validity  = true };  
                                                                                    
    static constexpr Chimera::GPIO::Pin     ledCANActivePin     = 8;
    static constexpr Chimera::GPIO::Port    ledCANActivePort    = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::PinInit ledCANActivePinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                                    .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                                                                    .pin       = ledCANActivePin,
                                                                    .port      = ledCANActivePort,
                                                                    .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                                    .state     = Chimera::GPIO::State::HIGH,
                                                                    .threaded  = false,
                                                                    .validity  = true }; 
                                                                                    
    static constexpr Chimera::GPIO::Pin     ledUSBActivePin     = 9;
    static constexpr Chimera::GPIO::Port    ledUSBActivePort    = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::PinInit ledUSBActivePinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                                    .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                                                                    .pin       = ledUSBActivePin,
                                                                    .port      = ledUSBActivePort,
                                                                    .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                                    .state     = Chimera::GPIO::State::HIGH,
                                                                    .threaded  = false,
                                                                    .validity  = true };  
                                                                                    
    static constexpr Chimera::GPIO::Pin     ledFaultPin     = 13;
    static constexpr Chimera::GPIO::Port    ledFaultPort    = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::GPIO::PinInit ledFaultPinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                                .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                                                                .pin       = ledFaultPin,
                                                                .port      = ledFaultPort,
                                                                .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                                .state     = Chimera::GPIO::State::HIGH,
                                                                .threaded  = false,
                                                                .validity  = true };  
                                                                                            
                                                                                           

  }    // namespace Digital

  namespace CAN
  {
    static constexpr Chimera::CAN::Channel channel = Chimera::CAN::Channel::CAN0;

    /*-----------------------------------------------------------------------------
    GPIO
    -----------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinTX  = 12;
    static constexpr Chimera::GPIO::Port portTX = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  pinRX  = 11;
    static constexpr Chimera::GPIO::Port portRX = Chimera::GPIO::Port::PORTA;
  }    // namespace CAN

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
    static constexpr Chimera::I2C::Frequency frequency = Chimera::I2C::Frequency::F100KHZ;
  }    // namespace I2C

  namespace SPI
  {
    /*-------------------------------------------------------------------------
    GPIO
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  sckPin    = 3;
    static constexpr Chimera::GPIO::Port sckPort   = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::Pin  misoPin   = 4;
    static constexpr Chimera::GPIO::Port misoPort  = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::Pin  mosiPin   = 5;
    static constexpr Chimera::GPIO::Port mosiPort  = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::Pin  norCSPin  = 15;
    static constexpr Chimera::GPIO::Port norCSPort = Chimera::GPIO::Port::PORTA;

    /*-------------------------------------------------------------------------
    SPI
    -------------------------------------------------------------------------*/
    static constexpr Chimera::SPI::Channel spiChannel = Chimera::SPI::Channel::SPI1;

    static constexpr Chimera::SPI::HardwareInit spiHwInit = { .bitOrder    = Chimera::SPI::BitOrder::MSB_FIRST,
                                                              .controlMode = Chimera::SPI::ControlMode::MASTER,
                                                              .clockFreq   = 40'000'000,
                                                              .clockMode   = Chimera::SPI::ClockMode::MODE0,
                                                              .dataSize    = Chimera::SPI::DataSize::SZ_8BIT,
                                                              .hwChannel   = Chimera::SPI::Channel::SPI3,
                                                              .txfrMode    = Chimera::SPI::TransferMode::BLOCKING,
                                                              .validity    = true };

    static constexpr Chimera::GPIO::PinInit sckPinInit = { .alternate = Chimera::GPIO::Alternate::SPI1_SCK,
                                                           .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                           .pin       = sckPin,
                                                           .port      = sckPort,
                                                           .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                           .state     = Chimera::GPIO::State::LOW,
                                                           .threaded  = false,
                                                           .validity  = true };

    static constexpr Chimera::GPIO::PinInit misoPinInit = { .alternate = Chimera::GPIO::Alternate::SPI1_MISO,
                                                            .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                            .pin       = misoPin,
                                                            .port      = misoPort,
                                                            .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                            .state     = Chimera::GPIO::State::LOW,
                                                            .threaded  = false,
                                                            .validity  = true };

    static constexpr Chimera::GPIO::PinInit mosiPinInit = { .alternate = Chimera::GPIO::Alternate::SPI1_MOSI,
                                                            .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                            .pin       = mosiPin,
                                                            .port      = mosiPort,
                                                            .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                            .state     = Chimera::GPIO::State::LOW,
                                                            .threaded  = false,
                                                            .validity  = true };

    static constexpr Chimera::GPIO::PinInit norCSPinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                             .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                                                             .pin       = norCSPin,
                                                             .port      = norCSPort,
                                                             .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                             .state     = Chimera::GPIO::State::HIGH,
                                                             .threaded  = false,
                                                             .validity  = true };
  }    // namespace SPI

  namespace Timer
  {
    /*-------------------------------------------------------------------------
    Timer 1 Channel 1
    V2 Mapping: Phase C
    V3 Mapping: ???
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinT1Ch1   = 8;
    static constexpr Chimera::GPIO::Port portT1Ch1  = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  pinT1Ch1N  = 7;
    static constexpr Chimera::GPIO::Port portT1Ch1N = Chimera::GPIO::Port::PORTA;

    /*-------------------------------------------------------------------------
    Timer 1 Channel 2
    V2 Mapping: Phase B
    V3 Mapping: ???
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinT1Ch2   = 9;
    static constexpr Chimera::GPIO::Port portT1Ch2  = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  pinT1Ch2N  = 0;
    static constexpr Chimera::GPIO::Port portT1Ch2N = Chimera::GPIO::Port::PORTB;

    /*-------------------------------------------------------------------------
    Timer 1 Channel 3
    V2 Mapping: Phase A
    V3 Mapping: ???
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinT1Ch3   = 10;
    static constexpr Chimera::GPIO::Port portT1Ch3  = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  pinT1Ch3N  = 1;
    static constexpr Chimera::GPIO::Port portT1Ch3N = Chimera::GPIO::Port::PORTB;
  }    // namespace Timer

  namespace USART
  {
    /*-------------------------------------------------------------------------
    Serial
    -------------------------------------------------------------------------*/
    static constexpr Chimera::Serial::Channel serialChannel = Chimera::Serial::Channel::SERIAL6;
    static constexpr size_t                   baudRate      = 2000000;

    /*-------------------------------------------------------------------------
    GPIO
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  rxPin     = 7;
    static constexpr Chimera::GPIO::Port rxPort    = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::GPIO::Pin  txPin     = 6;
    static constexpr Chimera::GPIO::Port txPort    = Chimera::GPIO::Port::PORTC;

    /*-------------------------------------------------------------------------
    Configuration Data
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::PinInit txPinInit = { .alternate = Chimera::GPIO::Alternate::USART6_TX,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = txPin,
                                                          .port      = txPort,
                                                          .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                          .state     = Chimera::GPIO::State::HIGH,
                                                          .threaded  = false,
                                                          .validity  = true };

    static constexpr Chimera::GPIO::PinInit rxPinInit = { .alternate = Chimera::GPIO::Alternate::USART6_RX,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = rxPin,
                                                          .port      = rxPort,
                                                          .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                          .state     = Chimera::GPIO::State::HIGH,
                                                          .threaded  = false,
                                                          .validity  = true };
  }    // namespace USART

}    // namespace Orbit::IO

#endif /* !ORBIT_ESC_BOARD_MAP_V2_HPP */
