/******************************************************************************
 *  File Name:
 *    com_stream.cpp
 *
 *  Description:
 *    Transport agnostic interface for publishing data to a remote host
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/core/com/com_stream.hpp>
#include <src/core/com/com_scheduler.hpp>


namespace Orbit::COM::Stream
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct InternalStreamState
  {
    StreamState userState; /**< Public state of the stream */
    pb_size_t   fieldTag;  /**< Encoding identifier for the 'one-of' field of StreamRequestMessage */
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  // Need to add a queue of chunks for building messages in the stream data callback

  /*---------------------------------------------------------------------------
  Static Function Declarations
  ---------------------------------------------------------------------------*/

  static void cmn_update_stream_data_callback( Scheduler::Task *task );
  static void flush_stream_data();

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void powerUp()
  {
    // Initialize the stream data queue

    // Register the common stream data callback. This needs to run fast enough
    // to flush the highest rate streams, but it shouldn't consume too much
    // CPU time. I guess that's going to be a bit of a balance.
  }


  const StreamState &getState( const SystemDataId id )
  {

  }


  bool processRequest( const StreamRequestMessage &request )
  {
    // This function is low speed. Can take the time to do a look up.

    // Enable/disable a stream
    // Update the rate of a stream
    // If a new stream, bind the common callback for data updates
    // Potentially update the scheduler task rate for the stream flushing.
  }


  bool enqueue( const SystemDataId id, const StreamData &&data )
  {

    // Possibly flush from this call instead of a full copy

    flush_stream_data();
  }


  bool enqueueFromISR( const SystemDataId id, const StreamData &&data )
  {
    // Use this as the primary enqueue interface, maybe.

    // Update the flags for the ID being sent. We know at this point if the data stream has
    // met the deadline for the next transaction time.

    // Possibly flush from this call
  }

  /*---------------------------------------------------------------------------
  Static Function Definitions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Shared handler for processing stream data update events
   *
   * @param task Pointer to the task entry being processed
   */
  static void cmn_update_stream_data_callback( Scheduler::Task *task )
  {
    // This function's purpose is to flush accumulated data only. When called,
    // package as much data from the queue as possible into a message and send it.
    // This might require seeing how much space is in the USB queue? Might want to
    // track a variable for rate control kinda like TCP to ensure the system doesn't
    // become incredibly unresponsive.

    // Should the queue be a ring buffer?

    // Use this method to flush data as a fallback if the USB queue was full

    flush_stream_data();
  }


  static void flush_stream_data()
  {

  }
}    // namespace Orbit::COM::Stream
