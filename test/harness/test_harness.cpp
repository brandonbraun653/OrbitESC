/******************************************************************************
 *  File Name:
 *    test_harness.cpp
 *
 *  Description:
 *    Test harness for the OrbitESC firmware
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/thread>
#include <test/harness/test_harness.hpp>
#include <test/harness/test_harness_cfg.hpp>

/* Include this last to prevent compiling issues */
#include <CppUTest/Utest.h>
#include <CppUTest/CommandLineTestRunner.h>


namespace Orbit::Testing
{
  /*---------------------------------------------------------------------------
  Default implementation of the test harness driver interface
  ---------------------------------------------------------------------------*/
  namespace Driver
  {
    Chimera::Status_t __attribute__( ( weak ) ) setUp()
    {
      return Chimera::Status::OK;
    }


    Chimera::Status_t __attribute__( ( weak ) ) tearDown()
    {
      return Chimera::Status::OK;
    }


    const char *__attribute__( ( weak ) ) getTestName()
    {
      return "Unknown Test";
    }

  }    // namespace Driver


  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static uint32_t s_test_thread_stack[ STACK_BYTES( THOR_TEST_RUNNER_STACK_SIZE ) ];

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  static void cpputest_crash_method()
  {
    RT_HARD_ASSERT( false );
  }


  /**
   * @brief Core thread that handles execution of the test suite
   *
   * @param argument  Unused
   * @return void
   */
  static void test_thread( void *argument )
  {
    /*-------------------------------------------------------------------------
    Configure the output style:
      - Colorized
      - Verbose
    -------------------------------------------------------------------------*/
    const char *av_override[] = { "-c", "-v" };

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    // char fmt_buffer[ 128 ];

    /*-------------------------------------------------------------------------
    Perform project specific setup
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( Driver::setUp() == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    Execute the tests
    -------------------------------------------------------------------------*/
    // auto serial = Chimera::Serial::getDriver( THOR_TEST_SERIAL_CHANNEL );
    // RT_HARD_ASSERT( serial );

    // memset( fmt_buffer, 0, sizeof( fmt_buffer ) );
    // snprintf( fmt_buffer, sizeof( fmt_buffer ), "\rStarting test: %s\r\n", Driver::getTestName() );
    // serial->write( fmt_buffer, strlen( fmt_buffer ), Chimera::Thread::TIMEOUT_BLOCK );

    UtestShell::getCurrent()->setCrashMethod( cpputest_crash_method );
    UtestShell::setCrashOnFail();
    int rcode = CommandLineTestRunner::RunAllTests( ARRAY_COUNT( av_override ), av_override );

    // memset( fmt_buffer, 0, sizeof( fmt_buffer ) );
    // snprintf( fmt_buffer, sizeof( fmt_buffer ), "Test exit with code: %d\r\n", rcode );
    // serial->write( fmt_buffer, strlen( fmt_buffer ), Chimera::Thread::TIMEOUT_BLOCK );

    /*-------------------------------------------------------------------------
    Perform project specific teardown
    -------------------------------------------------------------------------*/
    RT_HARD_ASSERT( Driver::tearDown() == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    Idle away to infinity
    -------------------------------------------------------------------------*/
    while ( 1 )
    {
      Chimera::delayMilliseconds( 25 );
    }
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  int EmbeddedTestThread()
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Initialize the hardware drivers
    -------------------------------------------------------------------------*/
    ChimeraInit();

    /*-------------------------------------------------------------------------
    Allocate the test thread
    -------------------------------------------------------------------------*/
    Task       userThread;
    TaskConfig cfg;

    cfg.name                                  = "Tester";
    cfg.arg                                   = nullptr;
    cfg.function                              = test_thread;
    cfg.priority                              = Priority::MINIMUM;
    cfg.stackWords                            = STACK_BYTES( sizeof( s_test_thread_stack ) );
    cfg.type                                  = TaskInitType::STATIC;
    cfg.specialization.staticTask.stackBuffer = s_test_thread_stack;
    cfg.specialization.staticTask.stackSize   = sizeof( s_test_thread_stack );

    userThread.create( cfg );
    userThread.start();

    /*-------------------------------------------------------------------------
    Run the test suite
    -------------------------------------------------------------------------*/
    startScheduler();
    return 0;
  }
}    // namespace Orbit::Testing
