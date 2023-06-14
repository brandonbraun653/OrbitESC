/********************************************************************************
 *  File Name:
 *    board_map_v1.hpp
 *
 *  Description:
 *    Maps system hardware resources for the OrbitESC PCB Version 1.0
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef ORBIT_ESC_BOARD_MAP_V1_HPP
#define ORBIT_ESC_BOARD_MAP_V1_HPP

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
#define ORBIT_ESC_V1

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
    static constexpr Chimera::GPIO::Pin    pinVSupply  = 0;
    static constexpr Chimera::GPIO::Port   portVSupply = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcVSupply  = Chimera::ADC::Channel::ADC_CH_5;

    /*-------------------------------------------------------------------------
    Phase A Current
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinPhaseA  = 4;
    static constexpr Chimera::GPIO::Port   portPhaseA = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcPhaseA  = Chimera::ADC::Channel::ADC_CH_9;

    /*-------------------------------------------------------------------------
    Phase B Current
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinPhaseB  = 5;
    static constexpr Chimera::GPIO::Port   portPhaseB = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcPhaseB  = Chimera::ADC::Channel::ADC_CH_10;

    /*-------------------------------------------------------------------------
    Phase C Current
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinPhaseC  = 1;
    static constexpr Chimera::GPIO::Port   portPhaseC = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcPhaseC  = Chimera::ADC::Channel::ADC_CH_6;
  }    // namespace Analog

  namespace Digital
  {
    /*-------------------------------------------------------------------------
    Emergency Stop Input
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin     eStopPin     = 3;
    static constexpr Chimera::GPIO::Port    eStopPort    = Chimera::GPIO::Port::PORTH;
    static constexpr Chimera::GPIO::PinInit eStopPinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                             .drive     = Chimera::GPIO::Drive::INPUT,
                                                             .pin       = eStopPin,
                                                             .port      = eStopPort,
                                                             .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                             .state     = Chimera::GPIO::State::LOW,
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
    static constexpr Chimera::GPIO::Pin  ledCSPin  = 15;
    static constexpr Chimera::GPIO::Port ledCSPort = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::GPIO::Pin  norCSPin  = 15;
    static constexpr Chimera::GPIO::Port norCSPort = Chimera::GPIO::Port::PORTA;

    /*-------------------------------------------------------------------------
    SPI
    -------------------------------------------------------------------------*/
    static constexpr Chimera::SPI::Channel spiChannel = Chimera::SPI::Channel::SPI3;

    static constexpr Chimera::SPI::HardwareInit spiHwInit = { .bitOrder    = Chimera::SPI::BitOrder::MSB_FIRST,
                                                              .controlMode = Chimera::SPI::ControlMode::MASTER,
                                                              .clockFreq   = 40'000'000,
                                                              .clockMode   = Chimera::SPI::ClockMode::MODE0,
                                                              .dataSize    = Chimera::SPI::DataSize::SZ_8BIT,
                                                              .hwChannel   = Chimera::SPI::Channel::SPI3,
                                                              .txfrMode    = Chimera::SPI::TransferMode::BLOCKING,
                                                              .validity    = true };

    static constexpr Chimera::GPIO::PinInit sckPinInit = { .alternate = Chimera::GPIO::Alternate::SPI3_SCK,
                                                           .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                           .pin       = sckPin,
                                                           .port      = sckPort,
                                                           .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                           .state     = Chimera::GPIO::State::LOW,
                                                           .threaded  = false,
                                                           .validity  = true };

    static constexpr Chimera::GPIO::PinInit misoPinInit = { .alternate = Chimera::GPIO::Alternate::SPI3_MISO,
                                                            .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                            .pin       = misoPin,
                                                            .port      = misoPort,
                                                            .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                            .state     = Chimera::GPIO::State::LOW,
                                                            .threaded  = false,
                                                            .validity  = true };

    static constexpr Chimera::GPIO::PinInit mosiPinInit = { .alternate = Chimera::GPIO::Alternate::SPI3_MOSI,
                                                            .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                            .pin       = mosiPin,
                                                            .port      = mosiPort,
                                                            .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                            .state     = Chimera::GPIO::State::LOW,
                                                            .threaded  = false,
                                                            .validity  = true };

    static constexpr Chimera::GPIO::PinInit ledCSPinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                             .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                                                             .pin       = ledCSPin,
                                                             .port      = ledCSPort,
                                                             .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                             .state     = Chimera::GPIO::State::HIGH,
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

  namespace USART
  {
    /*-------------------------------------------------------------------------
    Serial
    -------------------------------------------------------------------------*/
    static constexpr Chimera::Serial::Channel serialChannel = Chimera::Serial::Channel::SERIAL2;

    /*-------------------------------------------------------------------------
    GPIO
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  rxPin     = 3;
    static constexpr Chimera::GPIO::Port rxPort    = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  txPin     = 2;
    static constexpr Chimera::GPIO::Port txPort    = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  resetPin  = 14;
    static constexpr Chimera::GPIO::Port resetPort = Chimera::GPIO::Port::PORTC;

    /*-------------------------------------------------------------------------
    Configuration Data
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::PinInit txPinInit = { .alternate = Chimera::GPIO::Alternate::USART2_TX,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = txPin,
                                                          .port      = txPort,
                                                          .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                          .state     = Chimera::GPIO::State::HIGH,
                                                          .threaded  = false,
                                                          .validity  = true };

    static constexpr Chimera::GPIO::PinInit rxPinInit = { .alternate = Chimera::GPIO::Alternate::USART2_RX,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = rxPin,
                                                          .port      = rxPort,
                                                          .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                          .state     = Chimera::GPIO::State::HIGH,
                                                          .threaded  = false,
                                                          .validity  = true };

    static constexpr Chimera::GPIO::PinInit resetPinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                             .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                                                             .pin       = resetPin,
                                                             .port      = resetPort,
                                                             .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                             .state     = Chimera::GPIO::State::HIGH,
                                                             .threaded  = false,
                                                             .validity  = true };
  }    // namespace USART

}    // namespace Orbit::IO

#endif /* !ORBIT_ESC_BOARD_MAP_V1_HPP */
