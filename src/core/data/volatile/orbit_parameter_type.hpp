/******************************************************************************
 *  File Name:
 *    orbit_parameter_type.hpp
 *
 *  Description:
 *    Common types used for parameters
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_PARAMETER_TYPE_HPP
#define ORBIT_ESC_PARAMETER_TYPE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/serial/serial_interface.pb.h>


namespace Orbit::Data::Param
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  struct Node;

  /*-------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------*/
  using VerifierFunc = bool ( * )( const Node &node, const void *data, const size_t size );

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct Node
  {
    ParamId      id;        /**< Software enumeration tied to parameter */
    ParamType    type;      /**< Type of data stored in the parameter */
    const char  *key;       /**< String to use in database lookup*/
    void        *address;   /**< Fixed location in memory where data lives */
    uint16_t     maxSize;   /**< Max possible size of the data */
    VerifierFunc validator; /**< Function to validate the data before writing */
  };
}    // namespace Orbit::Data::Param

#endif /* !ORBIT_ESC_PARAMETER_TYPE_HPP */
