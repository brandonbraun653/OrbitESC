/******************************************************************************
 *  File Name:
 *    orbit_parameter_validators.hpp
 *
 *  Description:
 *    Processing utilities to validate parameter data
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_DATA_VALIDATORS_HPP
#define ORBIT_DATA_VALIDATORS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <src/core/data/volatile/orbit_parameter_type.hpp>


namespace Orbit::Data::Param
{
  /*-------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------*/
  bool ValidateParamId_PARAM_CAN_NODE_ID( const Node &node, const void *data, const size_t size );

}    // namespace Orbit::Data::Param

#endif /* !ORBIT_DATA_VALIDATORS_HPP */
