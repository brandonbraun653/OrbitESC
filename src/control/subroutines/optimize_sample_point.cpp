/******************************************************************************
 *  File Name:
 *    optimize_sample_point.cpp
 *
 *  Description:
 *    Implements the ADC sample time optimization routine
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/timer>
#include <src/control/subroutines/optimize_sample_point.hpp>
#include <src/core/hw/orbit_motor_drive.hpp>
#include <src/core/hw/orbit_motor_sense.hpp>


namespace Orbit::Control::Subroutine
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class SampleState : uint8_t
  {
    PWM_REDUCTION,
    FIND_LOWER_OFFSET,
    FIND_UPPER_OFFSET,
    COMPLETE
  };

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  // Offset max taken from motor timer ARR register. Assumes sense/motor timer have same clock and are synchronized (which they are).
  static constexpr uint32_t SENSE_OFFSET_MAX = 3000;
  static constexpr uint32_t SENSE_OFFSET_INC = 10;
  static constexpr uint32_t SENSE_ADJUST_MAX = ( 2u * ( SENSE_OFFSET_MAX / SENSE_OFFSET_INC ) );
  static constexpr float    SENSE_CURRENT    = 0.1f;

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::Timer::Inverter::Driver *s_motor_timer;
  static Chimera::Timer::Trigger::Slave   *s_sense_timer;

  static SampleState s_state;
  static uint32_t    s_cycle_count;
  static float       s_duty_cycle;
  static float       s_duty_final;
  static uint32_t    s_sense_offset;
  static uint32_t    s_offset_lower;
  static uint32_t    s_offset_upper;
  static bool        s_prev_current_active;


  /*---------------------------------------------------------------------------
  Class Implementation
  ---------------------------------------------------------------------------*/

  SampleTimeOptimizer::SampleTimeOptimizer()
  {
    id     = Routine::ADC_SAMPLE_POINT_OPTIMIZER;
    name   = "ADC Sample Point Optimizer";
    mState = RunState::UNINITIALIZED;
  }


  SampleTimeOptimizer::~SampleTimeOptimizer()
  {
  }


  void SampleTimeOptimizer::initialize()
  {
    using namespace Chimera::Timer::Inverter;

    LOG_INFO( "Initialized %s", this->name.c_str() );

    s_state        = SampleState::PWM_REDUCTION;
    s_duty_cycle   = 0.60f;
    s_duty_final   = s_duty_cycle;
    s_offset_lower = SENSE_OFFSET_MAX;

    s_motor_timer = Motor::Drive::getDriver();
    s_sense_timer = Motor::Sense::getTimer();

    s_motor_timer->enableOutput();

    mState = RunState::INITIALIZED;
  }


  void SampleTimeOptimizer::start()
  {
    LOG_INFO( "Running %s", this->name.c_str() );
    mState = RunState::RUNNING;
  }


  void SampleTimeOptimizer::stop()
  {
    LOG_INFO( "Stopped %s", this->name.c_str() );
    mState = RunState::STOPPED;
  }


  void SampleTimeOptimizer::destroy()
  {
    mState = RunState::UNINITIALIZED;
  }


  void SampleTimeOptimizer::process()
  {
    using namespace Chimera::Timer::Inverter;
    using namespace Orbit::Motor::Sense;

    /*-------------------------------------------------------------------------
    Read the latest current measurement
    -------------------------------------------------------------------------*/
    volatile const SenseData &sense_data = getSenseData();
    const float               current    = sense_data.channel[ CHANNEL_PHASE_C_CURRENT ];

    /*-------------------------------------------------------------------------
    RunState machine to find the optimal sample point. This sweeps the ADC sample
    time across the PWM output to find the best point to sample the current.
    Eventually this will find the lowest duty cycle that can be used in the
    motor control loop. This is just for debugging and experimentation.
    -------------------------------------------------------------------------*/
    switch( s_state )
    {
      case SampleState::PWM_REDUCTION:
        s_duty_final = s_duty_cycle;
        s_duty_cycle -= s_duty_cycle > 0.2f ? 0.1f : 0.01f;
        s_motor_timer->energizeWinding( SwitchIO::SWITCH_2_HI, SwitchIO::SWITCH_3_LO, s_duty_cycle );

        if( s_duty_cycle >= 0.01f )
        {
          s_state               = SampleState::FIND_LOWER_OFFSET;
          s_cycle_count         = 0;
          s_prev_current_active = false;
          s_sense_offset        = SENSE_OFFSET_MAX;
          s_sense_timer->setEventOffset( s_sense_offset );
        }
        else
        {
          s_state = SampleState::COMPLETE;
        }
        break;

      case SampleState::FIND_LOWER_OFFSET:
        if( current < SENSE_CURRENT && !s_prev_current_active )
        {
          s_sense_offset = ( s_sense_offset - SENSE_OFFSET_INC ) % SENSE_OFFSET_MAX;
          s_sense_timer->setEventOffset( s_sense_offset );
        }
        else if( current >= SENSE_CURRENT )
        {
          s_prev_current_active = true;
          s_offset_lower        = s_sense_offset;
          s_sense_offset        = ( s_sense_offset - SENSE_OFFSET_INC ) % SENSE_OFFSET_MAX;
          s_sense_timer->setEventOffset( s_sense_offset );
        }
        else if( current < SENSE_CURRENT && s_prev_current_active )
        {
          s_state               = SampleState::FIND_UPPER_OFFSET;
          s_cycle_count         = 0;
          s_sense_offset        = 0;
          s_prev_current_active = false;
          s_sense_timer->setEventOffset( s_sense_offset );
        }
        break;

      case SampleState::FIND_UPPER_OFFSET:
        if( current < SENSE_CURRENT && !s_prev_current_active )
        {
          s_sense_offset = ( s_sense_offset + SENSE_OFFSET_INC ) % SENSE_OFFSET_MAX;
          s_sense_timer->setEventOffset( s_sense_offset );
        }
        else if( current >= SENSE_CURRENT )
        {
          s_prev_current_active = true;
          s_offset_upper        = s_sense_offset;
          s_sense_offset        = ( s_sense_offset + SENSE_OFFSET_INC ) % SENSE_OFFSET_MAX;
          s_sense_timer->setEventOffset( s_sense_offset );
        }
        else if( current < SENSE_CURRENT && s_prev_current_active )
        {
          LOG_INFO( "Duty: %f, Lower: %d, Upper: %d", s_duty_cycle, s_offset_lower, s_offset_upper );
          s_state       = SampleState::PWM_REDUCTION;
          s_cycle_count = 0;
        }
        break;

      case SampleState::COMPLETE:
        s_motor_timer->disableOutput();
        LOG_INFO( "Min Duty: %f, Offset Lower: %d, Offset Upper: %d", s_duty_final, s_offset_lower, s_offset_upper );
        this->stop();
        break;
    }

    /*-----------------------------------------------------------------------------
    Prevent accidental infinite loops. Yes this is lazy. No I don't care. This is
    some very quick and dirty code.
    -----------------------------------------------------------------------------*/
    s_cycle_count++;
    if( s_cycle_count >= SENSE_ADJUST_MAX )
    {
      s_state = SampleState::COMPLETE;
    }
  }


  RunState SampleTimeOptimizer::state()
  {
    return mState;
  }

}    // namespace Orbit::Control::Subroutine
