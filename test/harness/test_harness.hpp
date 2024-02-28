/******************************************************************************
 *  File Name:
 *    test_harness.hpp
 *
 *  Description:
 *    Test harness for the OrbitESC firmware
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_TEST_HARNESS_HPP
#define ORBIT_ESC_TEST_HARNESS_HPP

namespace Orbit::Testing
{
  /*-----------------------------------------------------------------------------
  Public Functions
  -----------------------------------------------------------------------------*/

  /**
   * @brief Entry point for the on-device testing engine
   *
   * Expects to be called from the main() function in the main application.
   *
   * @return int
   */
  int EmbeddedTestThread();
}  // namespace Orbit::Testing

#endif  /* !ORBIT_ESC_TEST_HARNESS_HPP */
