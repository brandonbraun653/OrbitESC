/********************************************************************************
 *  File Name:
 *    board_map.cpp
 *
 *  Description:
 *    Board config definitions
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Project Includes */
#include <src/config/bsp/board_map.hpp>

namespace DC::IO
{
  /*-------------------------------------------------------------------------------
  Bluetooth
  -------------------------------------------------------------------------------*/
  namespace Bluetooth
  {
    const Chimera::Serial::Config comConfig = { .baud     = 115200,
                                                .width    = Chimera::Serial::CharWid::CW_8BIT,
                                                .parity   = Chimera::Serial::Parity::PAR_NONE,
                                                .stopBits = Chimera::Serial::StopBits::SBITS_ONE,
                                                .flow     = Chimera::Serial::FlowControl::FCTRL_NONE };

    const Chimera::GPIO::PinInit txPinInit = { .alternate = Chimera::GPIO::Alternate::USART6_TX,
                                               .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                               .pin       = txPin,
                                               .port      = txPort,
                                               .pull      = Chimera::GPIO::Pull::NO_PULL,
                                               .state     = Chimera::GPIO::State::HIGH,
                                               .threaded  = false,
                                               .validity  = true };

    const Chimera::GPIO::PinInit rxPinInit = { .alternate = Chimera::GPIO::Alternate::USART6_RX,
                                               .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                               .pin       = rxPin,
                                               .port      = rxPort,
                                               .pull      = Chimera::GPIO::Pull::NO_PULL,
                                               .state     = Chimera::GPIO::State::HIGH,
                                               .threaded  = false,
                                               .validity  = true };
  }    // namespace DBG

  /*-------------------------------------------------------------------------------
  Debug Port
  -------------------------------------------------------------------------------*/
  namespace DBG
  {
    const Chimera::Serial::Config comConfig = { .baud     = 115200,
                                                .width    = Chimera::Serial::CharWid::CW_8BIT,
                                                .parity   = Chimera::Serial::Parity::PAR_NONE,
                                                .stopBits = Chimera::Serial::StopBits::SBITS_ONE,
                                                .flow     = Chimera::Serial::FlowControl::FCTRL_NONE };

    const Chimera::GPIO::PinInit txPinInit = { .alternate = Chimera::GPIO::Alternate::USART3_TX,
                                               .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                               .pin       = txPin,
                                               .port      = txPort,
                                               .pull      = Chimera::GPIO::Pull::NO_PULL,
                                               .state     = Chimera::GPIO::State::HIGH,
                                               .threaded  = false,
                                               .validity  = true };

    const Chimera::GPIO::PinInit rxPinInit = { .alternate = Chimera::GPIO::Alternate::USART3_RX,
                                               .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                               .pin       = rxPin,
                                               .port      = rxPort,
                                               .pull      = Chimera::GPIO::Pull::NO_PULL,
                                               .state     = Chimera::GPIO::State::HIGH,
                                               .threaded  = false,
                                               .validity  = true };
  }    // namespace DBG


  /*-------------------------------------------------------------------------------
  USB
  -------------------------------------------------------------------------------*/
  namespace USB
  {
    const Chimera::GPIO::PinInit dPPinInit = { .alternate = Chimera::GPIO::Alternate::OTG_FS_DP,
                                               .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                               .pin       = dPPin,
                                               .port      = dPPort,
                                               .pull      = Chimera::GPIO::Pull::PULL_UP,
                                               .state     = Chimera::GPIO::State::HIGH,
                                               .threaded  = false,
                                               .validity  = true };

    const Chimera::GPIO::PinInit dMPinInit = { .alternate = Chimera::GPIO::Alternate::OTG_FS_DM,
                                               .drive     = Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,
                                               .pin       = dMPin,
                                               .port      = dMPort,
                                               .pull      = Chimera::GPIO::Pull::NO_PULL,
                                               .state     = Chimera::GPIO::State::LOW,
                                               .threaded  = false,
                                               .validity  = true };

    const Chimera::GPIO::PinInit idPinInit = { .alternate = Chimera::GPIO::Alternate::OTG_FS_ID,
                                               .drive     = Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN,
                                               .pin       = idPin,
                                               .port      = idPort,
                                               .pull      = Chimera::GPIO::Pull::PULL_UP,
                                               .state     = Chimera::GPIO::State::HIGH,
                                               .threaded  = false,
                                               .validity  = true };
  }
}    // namespace DC::IO
