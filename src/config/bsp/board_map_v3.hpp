/********************************************************************************
 *  File Name:
 *    board_map_v3.hpp
 *
 *  Description:
 *    Maps system hardware resources for the OrbitESC PCB Version 3.0
 *
 *  2023 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef ORBIT_ESC_BOARD_MAP_V3_HPP
#define ORBIT_ESC_BOARD_MAP_V3_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/adc>
#include <Chimera/can>
#include <Chimera/exti>
#include <Chimera/gpio>
#include <Chimera/i2c>
#include <Chimera/sdio>
#include <Chimera/serial>
#include <Chimera/spi>
#include <Chimera/timer>

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
    /*-------------------------------------------------------------------------
    High Level Parameters
    -------------------------------------------------------------------------*/
    static constexpr Chimera::ADC::Peripheral MotorADC        = Chimera::ADC::Peripheral::ADC_0;
    static constexpr Chimera::ADC::Peripheral InstrADC        = Chimera::ADC::Peripheral::ADC_1;
    static constexpr Chimera::GPIO::PinInit   CommonAnalogCfg = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                                  .drive     = Chimera::GPIO::Drive::ANALOG,
                                                                  .pin       = 0,
                                                                  .port      = Chimera::GPIO::Port::UNKNOWN_PORT,
                                                                  .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                                  .state     = Chimera::GPIO::State::HIGH,
                                                                  .threaded  = false,
                                                                  .validity  = false };

    /*-------------------------------------------------------------------------
    Phase A Current/Voltage
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinIPhaseA  = 0;
    static constexpr Chimera::GPIO::Port   portIPhaseA = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcIPhaseA  = Chimera::ADC::Channel::ADC_CH_0;

    static constexpr Chimera::GPIO::Pin    pinVPhaseA  = 1;
    static constexpr Chimera::GPIO::Port   portVPhaseA = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcVPhaseA  = Chimera::ADC::Channel::ADC_CH_1;

    /*-------------------------------------------------------------------------
    Phase B Current/Voltage
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinIPhaseB  = 2;
    static constexpr Chimera::GPIO::Port   portIPhaseB = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcIPhaseB  = Chimera::ADC::Channel::ADC_CH_2;

    static constexpr Chimera::GPIO::Pin    pinVPhaseB  = 5;
    static constexpr Chimera::GPIO::Port   portVPhaseB = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcVPhaseB  = Chimera::ADC::Channel::ADC_CH_5;

    /*-------------------------------------------------------------------------
    Phase C Current/Voltage
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinIPhaseC  = 4;
    static constexpr Chimera::GPIO::Port   portIPhaseC = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcIPhaseC  = Chimera::ADC::Channel::ADC_CH_4;

    static constexpr Chimera::GPIO::Pin    pinVPhaseC  = 3;
    static constexpr Chimera::GPIO::Port   portVPhaseC = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::ADC::Channel adcVPhaseC  = Chimera::ADC::Channel::ADC_CH_3;

    /*-------------------------------------------------------------------------
    Supply Voltage
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinVSupply  = 0;
    static constexpr Chimera::GPIO::Port   portVSupply = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::ADC::Channel adcVSupply  = Chimera::ADC::Channel::ADC_CH_10;

    /*-------------------------------------------------------------------------
    MCU Voltage
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinVMCU  = 1;
    static constexpr Chimera::GPIO::Port   portVMCU = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::ADC::Channel adcVMCU  = Chimera::ADC::Channel::ADC_CH_11;

    /*-------------------------------------------------------------------------
    Temperature Sensor Voltage
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinVTemp  = 2;
    static constexpr Chimera::GPIO::Port   portVTemp = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::ADC::Channel adcVTemp  = Chimera::ADC::Channel::ADC_CH_12;

    /*-------------------------------------------------------------------------
    Current Sense Reference Voltage
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin    pinVISenseRef  = 3;
    static constexpr Chimera::GPIO::Port   portVISenseRef = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::ADC::Channel adcVISenseRef  = Chimera::ADC::Channel::ADC_CH_13;

    /*-------------------------------------------------------------------------
    Timer Event Injection
    -------------------------------------------------------------------------*/
    static constexpr size_t MotorExternalEventChannel  = 14; /**< EXTSEL Regular Channel, TIM8_TRGO */
    static constexpr size_t SensorExternalEventChannel = 8;  /**< EXTSEL Regular Channel, TIM3_TRGO */

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

  namespace SDIO
  {
    /*-------------------------------------------------------------------------
    SDIO
    -------------------------------------------------------------------------*/
    static constexpr Chimera::SDIO::Channel  Channel    = Chimera::SDIO::Channel::SDIO1;
    static constexpr Chimera::SDIO::BusWidth BusWidth   = Chimera::SDIO::BusWidth::BUS_WIDTH_1BIT;
    static constexpr uint32_t                ClockSpeed = 4'000'000;
    static constexpr uint32_t                BlockSize  = 512;

    /*-------------------------------------------------------------------------
    GPIO
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin     d0Pin     = 8;
    static constexpr Chimera::GPIO::Port    d0Port    = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::GPIO::PinInit d0PinInit = { .alternate = Chimera::GPIO::Alternate::SDIO_D0,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = d0Pin,
                                                          .port      = d0Port,
                                                          .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                          .state     = Chimera::GPIO::State::LOW,
                                                          .threaded  = false,
                                                          .validity  = true };

    static constexpr Chimera::GPIO::Pin     d1Pin     = 9;
    static constexpr Chimera::GPIO::Port    d1Port    = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::GPIO::PinInit d1PinInit = { .alternate = Chimera::GPIO::Alternate::SDIO_D1,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = d1Pin,
                                                          .port      = d1Port,
                                                          .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                          .state     = Chimera::GPIO::State::LOW,
                                                          .threaded  = false,
                                                          .validity  = true };

    static constexpr Chimera::GPIO::Pin     d2Pin     = 10;
    static constexpr Chimera::GPIO::Port    d2Port    = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::GPIO::PinInit d2PinInit = { .alternate = Chimera::GPIO::Alternate::SDIO_D2,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = d2Pin,
                                                          .port      = d2Port,
                                                          .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                          .state     = Chimera::GPIO::State::LOW,
                                                          .threaded  = false,
                                                          .validity  = true };

    static constexpr Chimera::GPIO::Pin     d3Pin     = 11;
    static constexpr Chimera::GPIO::Port    d3Port    = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::GPIO::PinInit d3PinInit = { .alternate = Chimera::GPIO::Alternate::SDIO_D3,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = d3Pin,
                                                          .port      = d3Port,
                                                          .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                          .state     = Chimera::GPIO::State::LOW,
                                                          .threaded  = false,
                                                          .validity  = true };

    static constexpr Chimera::GPIO::Pin     clkPin     = 12;
    static constexpr Chimera::GPIO::Port    clkPort    = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::GPIO::PinInit clkPinInit = { .alternate = Chimera::GPIO::Alternate::SDIO_CK,
                                                           .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                           .pin       = clkPin,
                                                           .port      = clkPort,
                                                           .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                           .state     = Chimera::GPIO::State::LOW,
                                                           .threaded  = false,
                                                           .validity  = true };

    static constexpr Chimera::GPIO::Pin     cmdPin     = 2;
    static constexpr Chimera::GPIO::Port    cmdPort    = Chimera::GPIO::Port::PORTD;
    static constexpr Chimera::GPIO::PinInit cmdPinInit = { .alternate = Chimera::GPIO::Alternate::SDIO_CMD,
                                                           .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                           .pin       = cmdPin,
                                                           .port      = cmdPort,
                                                           .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                           .state     = Chimera::GPIO::State::LOW,
                                                           .threaded  = false,
                                                           .validity  = true };

    static constexpr Chimera::GPIO::Pin     cdPin     = 10;
    static constexpr Chimera::GPIO::Port    cdPort    = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::PinInit cdPinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                          .drive     = Chimera::GPIO::Drive::INPUT,
                                                          .pin       = cdPin,
                                                          .port      = cdPort,
                                                          .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                          .state     = Chimera::GPIO::State::LOW,
                                                          .threaded  = false,
                                                          .validity  = true };
  }    // namespace SDIO

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
                                                              .hwChannel   = spiChannel,
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
    High Level Parameters
    -------------------------------------------------------------------------*/
    static constexpr Chimera::Timer::Instance MotorDrive    = Chimera::Timer::Instance::TIMER1;
    static constexpr Chimera::Timer::Instance MotorSense    = Chimera::Timer::Instance::TIMER8;
    static constexpr Chimera::Timer::Instance SpeedControl  = Chimera::Timer::Instance::TIMER2;
    static constexpr Chimera::Timer::Instance SensorCapture = Chimera::Timer::Instance::TIMER3;

    /*-------------------------------------------------------------------------
    Timer 1 Channel 1: Phase A
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinT1Ch1   = 8;
    static constexpr Chimera::GPIO::Port portT1Ch1  = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  pinT1Ch1N  = 7;
    static constexpr Chimera::GPIO::Port portT1Ch1N = Chimera::GPIO::Port::PORTA;

    /*-------------------------------------------------------------------------
    Timer 1 Channel 2: Phase B
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin  pinT1Ch2   = 9;
    static constexpr Chimera::GPIO::Port portT1Ch2  = Chimera::GPIO::Port::PORTA;
    static constexpr Chimera::GPIO::Pin  pinT1Ch2N  = 0;
    static constexpr Chimera::GPIO::Port portT1Ch2N = Chimera::GPIO::Port::PORTB;

    /*-------------------------------------------------------------------------
    Timer 1 Channel 3: Phase C
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
    static constexpr Chimera::GPIO::Pin  rxPin  = 7;
    static constexpr Chimera::GPIO::Port rxPort = Chimera::GPIO::Port::PORTC;
    static constexpr Chimera::GPIO::Pin  txPin  = 6;
    static constexpr Chimera::GPIO::Port txPort = Chimera::GPIO::Port::PORTC;

    /*-------------------------------------------------------------------------
    Configuration Data
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::PinInit txPinInit = { .alternate = Chimera::GPIO::Alternate::USART6_TX,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = txPin,
                                                          .port      = txPort,
                                                          .pull      = Chimera::GPIO::Pull::PULL_UP,
                                                          .state     = Chimera::GPIO::State::HIGH,
                                                          .threaded  = false,
                                                          .validity  = true };

    static constexpr Chimera::GPIO::PinInit rxPinInit = { .alternate = Chimera::GPIO::Alternate::USART6_RX,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = rxPin,
                                                          .port      = rxPort,
                                                          .pull      = Chimera::GPIO::Pull::PULL_UP,
                                                          .state     = Chimera::GPIO::State::HIGH,
                                                          .threaded  = false,
                                                          .validity  = true };
  }    // namespace USART

  namespace USB
  {
    /*-------------------------------------------------------------------------
    GPIO
    -------------------------------------------------------------------------*/
    static constexpr Chimera::GPIO::Pin     dmPin     = 14;
    static constexpr Chimera::GPIO::Port    dmPort    = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::PinInit dmPinInit = { .alternate = Chimera::GPIO::Alternate::OTG_HS_DM,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = dmPin,
                                                          .port      = dmPort,
                                                          .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                          .state     = Chimera::GPIO::State::LOW,
                                                          .threaded  = false,
                                                          .validity  = true };

    static constexpr Chimera::GPIO::Pin     dpPin     = 15;
    static constexpr Chimera::GPIO::Port    dpPort    = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::PinInit dpPinInit = { .alternate = Chimera::GPIO::Alternate::OTG_HS_DP,
                                                          .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                                          .pin       = dpPin,
                                                          .port      = dpPort,
                                                          .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                          .state     = Chimera::GPIO::State::LOW,
                                                          .threaded  = false,
                                                          .validity  = true };

    static constexpr Chimera::GPIO::Pin     enumPin     = 12;
    static constexpr Chimera::GPIO::Port    enumPort    = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::PinInit enumPinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                            .drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,
                                                            .pin       = enumPin,
                                                            .port      = enumPort,
                                                            .pull      = Chimera::GPIO::Pull::NO_PULL,
                                                            .state     = Chimera::GPIO::State::LOW,
                                                            .threaded  = false,
                                                            .validity  = true };

    static constexpr Chimera::GPIO::Pin     vbusPin     = 13;
    static constexpr Chimera::GPIO::Port    vbusPort    = Chimera::GPIO::Port::PORTB;
    static constexpr Chimera::GPIO::PinInit vbusPinInit = { .alternate = Chimera::GPIO::Alternate::NONE,
                                                            .drive     = Chimera::GPIO::Drive::INPUT,
                                                            .pin       = vbusPin,
                                                            .port      = vbusPort,
                                                            .pull      = Chimera::GPIO::Pull::PULL_DN,
                                                            .state     = Chimera::GPIO::State::LOW,
                                                            .threaded  = false,
                                                            .validity  = true };
  }    // namespace USB
}    // namespace Orbit::IO

#endif /* !ORBIT_ESC_BOARD_MAP_V3_HPP */
