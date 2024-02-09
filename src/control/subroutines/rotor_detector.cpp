/******************************************************************************
 *  File Name:
 *    rotor_detector.cpp
 *
 *  Description:
 *    Subroutine for detecting the static position of the rotor
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <src/control/subroutines/rotor_detector.hpp>
#include <src/core/hw/orbit_motor_drive.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>

namespace Orbit::Control::Subroutine
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr float DRIVE_DUTY_CYCLE = 0.03f;

  /*---------------------------------------------------------------------------
  RotorDetector Implementation
  ---------------------------------------------------------------------------*/

  RotorDetector::RotorDetector()
  {
    mMeasurements.fill( {} );
    id         = Routine::ALIGNMENT_DETECTION;
    name       = "Rotor Alignment Detector";
    mState     = State::UNINITIALIZED;
    mStartTime = 0;
    mIdx       = 0;
  }


  RotorDetector::~RotorDetector()
  {
  }


  void RotorDetector::initialize()
  {
    using namespace Chimera::Timer::Inverter;

    LOG_INFO( "Initialized %s", this->name.c_str() );

    mTimer = Motor::Drive::getDriver();
    mTimer->enableOutput();

    /*-------------------------------------------------------------------------
    Configure the 6 commutation sampling sequences
    -------------------------------------------------------------------------*/
    mMeasurements[ 0 ].hiSide     = SwitchIO::SWITCH_2_HI;
    mMeasurements[ 0 ].loSide     = SwitchIO::SWITCH_3_LO;
    mMeasurements[ 0 ].accCurrent = 0.0f;

    mMeasurements[ 1 ].hiSide     = SwitchIO::SWITCH_3_HI;
    mMeasurements[ 1 ].loSide     = SwitchIO::SWITCH_2_LO;
    mMeasurements[ 1 ].accCurrent = 0.0f;

    mMeasurements[ 2 ].hiSide     = SwitchIO::SWITCH_1_HI;
    mMeasurements[ 2 ].loSide     = SwitchIO::SWITCH_2_LO;
    mMeasurements[ 2 ].accCurrent = 0.0f;

    mMeasurements[ 3 ].hiSide     = SwitchIO::SWITCH_2_HI;
    mMeasurements[ 3 ].loSide     = SwitchIO::SWITCH_1_LO;
    mMeasurements[ 3 ].accCurrent = 0.0f;

    mMeasurements[ 4 ].hiSide     = SwitchIO::SWITCH_3_HI;
    mMeasurements[ 4 ].loSide     = SwitchIO::SWITCH_1_LO;
    mMeasurements[ 4 ].accCurrent = 0.0f;

    mMeasurements[ 5 ].hiSide     = SwitchIO::SWITCH_1_HI;
    mMeasurements[ 5 ].loSide     = SwitchIO::SWITCH_3_LO;
    mMeasurements[ 5 ].accCurrent = 0.0f;

    mState = State::INITIALIZED;
  }


  void RotorDetector::start()
  {
    LOG_INFO( "Running %s", this->name.c_str() );
    mState = State::RUNNING;


    mIdx = 0;
    mTimer->energizeWinding( mMeasurements[ mIdx ].hiSide, mMeasurements[ mIdx ].loSide, DRIVE_DUTY_CYCLE );
    mStartTime    = Chimera::millis();
    mSampleActive = true;
  }


  void RotorDetector::stop()
  {
    LOG_INFO( "Stopped %s", this->name.c_str() );
    mState = State::STOPPED;
    if( mTimer )
    {
      mTimer->disableOutput();
    }
  }


  void RotorDetector::destroy()
  {
    mState = State::UNINITIALIZED;
  }


  void RotorDetector::process()
  {
    using namespace Chimera::Timer::Inverter;
    using namespace Orbit::Motor::Sense;

    /*-------------------------------------------------------------------------
    Read the latest current measurement
    -------------------------------------------------------------------------*/
    volatile const SenseData &sense_data = getSenseData();

    /*-------------------------------------------------------------------------
    Accumulate current readings if we're actively sampling
    -------------------------------------------------------------------------*/
    if( mSampleActive )
    {
      switch( mMeasurements[ mIdx ].loSide )
      {
        case SwitchIO::SWITCH_1_LO:
          mMeasurements[ mIdx ].accCurrent += sense_data.channel[ CHANNEL_PHASE_A_CURRENT ];
          break;

        case SwitchIO::SWITCH_2_LO:
          mMeasurements[ mIdx ].accCurrent += sense_data.channel[ CHANNEL_PHASE_B_CURRENT ];
          break;

        case SwitchIO::SWITCH_3_LO:
          mMeasurements[ mIdx ].accCurrent += sense_data.channel[ CHANNEL_PHASE_C_CURRENT ];
          break;

        default:
          break;
      }

      if( mMeasurements[ mIdx ].accCurrent > 25.0f )
      {
        mMeasurements[ mIdx ].accTime = Chimera::millis() - mStartTime;
        // mTimer->shortLowSideWindings();
        mTimer->energizeWinding( mMeasurements[ mIdx ].hiSide, mMeasurements[ mIdx ].loSide, 0.0f );
        mSampleActive = false;
        mStartTime    = Chimera::millis();
      }
    }

    /*-------------------------------------------------------------------------
    Windings fully discharged, move to next measurement
    -------------------------------------------------------------------------*/
    // TODO: Could probably do this in a more efficient way, maybe by using the actual measurements
    if( !mSampleActive && ( ( Chimera::millis() - mStartTime ) > 100 ) )
    {
      mIdx++;
      mTimer->energizeWinding( mMeasurements[ mIdx ].hiSide, mMeasurements[ mIdx ].loSide, DRIVE_DUTY_CYCLE );
      mStartTime    = Chimera::millis();
      mSampleActive = true;
    }

    /*-------------------------------------------------------------------------
    Check if all measurements are complete
    -------------------------------------------------------------------------*/
    if( mIdx >= mMeasurements.size() )
    {
      mState = State::STOPPED;
      mTimer->disableOutput();
      LOG_INFO( "Rotor Detector Results" );
      size_t lowest_idx = 0;
      size_t lowest_val = 0xFFFFFFFF;
      for( size_t idx = 0; idx < mMeasurements.size(); idx++ )
      {
        LOG_INFO( "  Measurement %d: %d", idx, mMeasurements[ idx ].accTime );
        if( mMeasurements[ idx ].accTime < lowest_val )
        {
          lowest_idx = idx;
          lowest_val = mMeasurements[ idx ].accTime;
        }
      }

      LOG_INFO( "  Lowest: %d", lowest_idx );
    }
  }


  State RotorDetector::state()
  {
    return mState;
  }

}    // namespace Orbit::Control::Subroutine
