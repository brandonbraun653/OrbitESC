/******************************************************************************
 *  File Name:
 *    orbit_parameter_validators.cpp
 *
 *  Description:
 *    Validator function implementations
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/data/volatile/orbit_parameter_validators.hpp>
#include <src/core/hw/orbit_can.hpp>

namespace Orbit::Data::Param
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool ValidateParamId_PARAM_CAN_NODE_ID( const Node &node, const void *data, const size_t size )
  {
    if ( ( size == node.maxSize ) && ( node.type == ParamType_UINT8 ) )
    {
      auto value = *static_cast<const CAN::NodeId *>( data );
      return ( value >= CAN::NodeId::FIRST_ASSIGNABLE ) && ( value <= CAN::NodeId::LAST_ASSIGNABLE );
    }
    else
    {
      return false;
    }
  }
}    // namespace Orbit::Data::Param
