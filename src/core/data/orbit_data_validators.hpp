/******************************************************************************
 *  File Name:
 *    orbit_data_utils.hpp
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
#include <src/core/com/serial/serial_interface.pb.h>


namespace Orbit::Data
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  struct ParameterNode;

  /*-------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------*/
  using VerifierFunc = bool ( * )( const ParameterNode &node, const void *data, const size_t size );

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ParameterNode
  {
    ParamId       id;        /**< Software enumeration tied to parameter */
    ParamType     type;      /**< Type of data stored in the parameter */
    const char   *key;       /**< String to use in JSON data storage */
    void         *address;   /**< Fixed location in memory where data lives */
    size_t        maxSize;   /**< Max possible size of the data */
    VerifierFunc  validator; /**< Function to validate the data before writing */
  };

  /*-------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------*/
  bool ValidateParamId_PARAM_CAN_NODE_ID( const ParameterNode &node, const void *data, const size_t size );

}  // namespace Orbit::Data

#endif  /* !ORBIT_DATA_VALIDATORS_HPP */
