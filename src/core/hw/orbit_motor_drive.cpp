/******************************************************************************
 *  File Name:
 *    orbit_motor_drive.cpp
 *
 *  Description:
 *    Hardware driver interface for controlling the motor drive power stage.
 *    This is a thin veneer over the Chimera timer driver, mainly to manage the
 *    lifetime of the HW.
 *
 *  2023-2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/timer>
#include <src/config/bsp/board_map.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/hw/orbit_adc.hpp>
#include <src/core/hw/orbit_motor.hpp>
#include <src/core/hw/orbit_motor_drive.hpp>


namespace Orbit::Motor::Drive
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::Timer::Inverter::Driver s_motor_drive_timer;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  Chimera::Timer::Inverter::Driver *getDriver()
  {
    return &s_motor_drive_timer;
  }


  void initialize()
  {
    /*-------------------------------------------------------------------------
    Configure the Advanced Timer for center-aligned 3-phase PWM
    -------------------------------------------------------------------------*/
    Chimera::Timer::Inverter::DriverConfig pwm_cfg;

    pwm_cfg.clear();
    pwm_cfg.coreCfg.instance    = Orbit::IO::Timer::MotorDrive;
    pwm_cfg.coreCfg.clockSource = Chimera::Clock::Bus::SYSCLK;
    pwm_cfg.coreCfg.baseFreq    = TIMER_BASE_FREQ;
    pwm_cfg.coreCfg.tolerance   = 1.0f;
    pwm_cfg.breakIOLevel        = Chimera::GPIO::State::LOW;
    pwm_cfg.pwmFrequency        = Orbit::Data::SysControl.statorPWMFreq;
    pwm_cfg.deadTimeNs          = 250.0f;
    pwm_cfg.adcSampleTimeNs     = 1800.0f;
    pwm_cfg.settleTimeNs        = 2000.0f;

    RT_HARD_ASSERT( Chimera::Status::OK == s_motor_drive_timer.init( pwm_cfg ) );
  }


  void reset()
  {
    // TODO: Don't call this unless you have a way to restart the power up sequence.
    // s_motor_drive_timer.reset();
  }


  void emergencyStop()
  {
    s_motor_drive_timer.emergencyBreak();
  }


  void enableOutput()
  {
    s_motor_drive_timer.enableOutput();
  }


  void disableOutput()
  {
    s_motor_drive_timer.disableOutput();
  }

}    // namespace Orbit::Motor
