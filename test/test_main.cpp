/******************************************************************************
 *  File Name:
 *    test_main.cpp
 *
 *  Description:
 *    Main entry point for GoogleTest-based unit tests
 *
 *  2025 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#include <gtest/gtest.h>

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
