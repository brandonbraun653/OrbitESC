/******************************************************************************
 *  File Name:
 *    orbit_sdio.cpp
 *
 *  Description:
 *    OrbitESC SDIO driver implementation
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/exti>
#include <Chimera/sdio>
#include <Chimera/thread>
#include <src/config/bsp/board_map.hpp>
#include <src/core/hw/orbit_sdio.hpp>
#include <src/core/tasks.hpp>

namespace Orbit::SDIO
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/

  /**
   * @brief How long to wait between card detect interrupts before
   *        considering the card state to have changed.
   */
  static constexpr size_t CARD_DEBOUNCE_MS = 100;

  /**
   * @brief How long to wait after a card is inserted before attempting to mount it.
   */
  static constexpr size_t CARD_MOUNT_DELAY_MS = 250;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/

  /**
   * @brief Logical state of card slot presence
   */
  enum class Status : uint8_t
  {
    CARD_INSERTED,
    CARD_REMOVED,
    CARD_NUM_OPTIONS
  };


  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

  static Status                      s_prev_status;     /**< Current card status */
  static Status                      s_card_action;     /**< Action to take on the card */
  static size_t                      s_last_event_time; /**< Last time an ISR event occurred */
  static size_t                      s_scheduled_mount; /**< Time to mount the card */
  static Chimera::Function::vGeneric s_on_insert_cb;    /**< Callback for card insertion */
  static Chimera::Function::vGeneric s_on_remove_cb;    /**< Callback for card removal */


  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Detects the presence of a card in the slot
   * @return Status
   */
  static Status cardStatus()
  {
    using namespace Chimera::GPIO;

    auto pin = getDriver( IO::SDIO::cdPort, IO::SDIO::cdPin );
    RT_DBG_ASSERT( pin != nullptr );

    State io_level;
    pin->getState( io_level );

    if ( io_level == State::LOW )
    {
      return Status::CARD_INSERTED;
    }
    else
    {
      return Status::CARD_REMOVED;
    }
  }


  /**
   * @brief Callback for when the card detect pin changes state.
   *
   * It's possible to get multiple interrupts for a single card event, so
   * debounce the signal here.
   *
   * @return void
   */
  static void onCardDetectISR( void *unused )
  {
    using namespace Chimera::Thread;
    using namespace Orbit::Tasks;

    const size_t event_time = Chimera::millis();
    const Status new_status = cardStatus();

    if ( ( event_time - s_last_event_time ) >= CARD_DEBOUNCE_MS )
    {
      s_last_event_time = event_time;

      if ( new_status == Status::CARD_INSERTED )
      {
        s_prev_status     = new_status;
        s_scheduled_mount = event_time + CARD_MOUNT_DELAY_MS;
      }
      else if ( new_status == Status::CARD_REMOVED )
      {
        s_prev_status = new_status;
      }
    }
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void powerUp()
  {
    /*-------------------------------------------------------------------------
    Initialize the SDIO peripheral
    -------------------------------------------------------------------------*/
    Chimera::SDIO::HWConfig cfg;

    cfg.clear();
    cfg.channel    = IO::SDIO::Channel;
    cfg.clockSpeed = IO::SDIO::ClockSpeed;
    cfg.blockSize  = IO::SDIO::BlockSize;
    cfg.width      = IO::SDIO::BusWidth;
    cfg.clkPin     = IO::SDIO::clkPinInit;
    cfg.cmdPin     = IO::SDIO::cmdPinInit;
    cfg.dxPin[ 0 ] = IO::SDIO::d0PinInit;
    cfg.dxPin[ 1 ] = IO::SDIO::d1PinInit;
    cfg.dxPin[ 2 ] = IO::SDIO::d2PinInit;
    cfg.dxPin[ 3 ] = IO::SDIO::d3PinInit;

    auto pSDIO = Chimera::SDIO::getDriver( cfg.channel );
    RT_HARD_ASSERT( pSDIO != nullptr );
    RT_HARD_ASSERT( pSDIO->open( cfg ) == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    Initialize SD card slot detection pin
    -------------------------------------------------------------------------*/
    auto pin = Chimera::GPIO::getDriver( IO::SDIO::cdPort, IO::SDIO::cdPin );
    RT_HARD_ASSERT( pin != nullptr );
    RT_HARD_ASSERT( pin->init( IO::SDIO::cdPinInit ) == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    Configure interrupts on the card detect pin
    -------------------------------------------------------------------------*/
    auto cb = Chimera::Function::vGeneric::create<onCardDetectISR>();
    RT_HARD_ASSERT( pin->attachInterrupt( cb, Chimera::EXTI::EdgeTrigger::BOTH_EDGE ) == Chimera::Status::OK );

    /*-------------------------------------------------------------------------
    Initialize module data
    -------------------------------------------------------------------------*/
    s_prev_status     = cardStatus();
    s_last_event_time = Chimera::millis();
    s_scheduled_mount = 0xFFFFFFFF;
    s_card_action     = Status::CARD_NUM_OPTIONS;
    s_on_insert_cb    = {};
    s_on_remove_cb    = {};
  }


  void handleCardStatusChange()
  {
    /*-------------------------------------------------------------------------
    Nothing to do if the card status hasn't changed
    -------------------------------------------------------------------------*/
    if ( s_card_action == Status::CARD_NUM_OPTIONS )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Check if it's time to mount/unmount the card
    -------------------------------------------------------------------------*/
    if ( ( s_card_action == Status::CARD_INSERTED ) && ( Chimera::millis() >= s_scheduled_mount ) )
    {
      s_card_action = Status::CARD_NUM_OPTIONS;

      if ( s_on_insert_cb )
      {
        s_on_insert_cb( nullptr );
      }
    }
    else if ( s_card_action == Status::CARD_REMOVED )
    {
      s_card_action = Status::CARD_NUM_OPTIONS;

      if ( s_on_remove_cb )
      {
        s_on_remove_cb( nullptr );
      }
    }
  }


  void onCardInsert( Chimera::Function::vGeneric &func )
  {
    s_on_insert_cb = func;
  }


  void onCardRemove( Chimera::Function::vGeneric &func )
  {
    s_on_remove_cb = func;
  }


  bool isCardPresent()
  {
    return cardStatus() == Status::CARD_INSERTED;
  }

}    // namespace Orbit::SDIO
