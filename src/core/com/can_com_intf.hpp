/******************************************************************************
 *  File Name:
 *    can_com_intf.hpp
 *
 *  Description:
 *    Interface class for declaring CAN messages for OrbitESC
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_CAN_COM_INTF_HPP
#define ORBIT_ESC_CAN_COM_INTF_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstdlib>

namespace Orbit::CAN
{
  /*---------------------------------------------------------------------------
  Template for Declaring a New Message
  ---------------------------------------------------------------------------*/
  template<typename PAYLOAD, uint8_t ENUM, uint8_t ID, uint32_t PERIOD = 0>
  class MessageDefinition
  {
  public:
    using PayloadType = PAYLOAD;

    static constexpr uint8_t  SWEnum = ENUM;
    static constexpr uint8_t  CanID = ID;
    static constexpr uint32_t Period = PERIOD;

    static constexpr bool is_periodic()
    {
      return Period != 0;
    }
  };
}  // namespace Orbit::CAN

#endif  /* !ORBIT_ESC_CAN_COM_INTF_HPP */
