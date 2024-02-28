/******************************************************************************
 *  File Name:
 *    test_harness_cfg.hpp
 *
 *  Description:
 *    Configuration options for the test harness
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_TEST_HARNESS_CONFIG_HPP
#define ORBIT_TEST_HARNESS_CONFIG_HPP


/*-----------------------------------------------------------------------------
Set the stack size of the test runner thread
-----------------------------------------------------------------------------*/
#ifndef THOR_TEST_RUNNER_STACK_SIZE
#define THOR_TEST_RUNNER_STACK_SIZE ( 10 * 1024u )
#endif

#endif  /* !ORBIT_TEST_HARNESS_CONFIG_HPP */
