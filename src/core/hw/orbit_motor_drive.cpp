/******************************************************************************
 *  File Name:
 *    orbit_motor_drive.cpp
 *
 *  Description:
 *    Hardware driver for controlling the motor power stage
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/timer>
#include <src/config/bsp/board_map.hpp>
#include <src/core/data/orbit_data.hpp>
#include <src/core/hw/orbit_motor.hpp>


namespace Orbit::Motor
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Chimera::Timer::Inverter::Driver s_motor_drive_timer; /**< Motor drive timer */

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void powerUpDrive()
  {
    /*-------------------------------------------------------------------------
    Configure the Advanced Timer for center-aligned 3-phase PWM
    -------------------------------------------------------------------------*/
    Chimera::Timer::Inverter::DriverConfig pwm_cfg;

    pwm_cfg.clear();
    pwm_cfg.coreCfg.instance    = Orbit::IO::Timer::MotorDrive;
    pwm_cfg.coreCfg.clockSource = Chimera::Clock::Bus::SYSCLK;
    pwm_cfg.coreCfg.baseFreq    = 30'000'000.0f;
    pwm_cfg.coreCfg.tolerance   = 1.0f;
    pwm_cfg.adcPeripheral       = Orbit::IO::Analog::MotorPeripheral;
    pwm_cfg.breakIOLevel        = Chimera::GPIO::State::LOW;
    pwm_cfg.deadTimeNs          = 250.0f;
    pwm_cfg.pwmFrequency        = Orbit::Data::SysControl.statorPWMFreq;

    RT_HARD_ASSERT( Chimera::Status::OK == s_motor_drive_timer.init( pwm_cfg ) );
  }


  void emergencyStop()
  {
    s_motor_drive_timer.emergencyBreak();
  }


  void enableDriveOutput()
  {
    s_motor_drive_timer.enableOutput();
  }


  void disableDriveOutput()
  {
    s_motor_drive_timer.disableOutput();
  }


  void setDrivePhaseWidth( const uint32_t a, const uint32_t b, const uint32_t c )
  {
    s_motor_drive_timer.setPhaseDutyCycle( a, b, c );
  }

}    // namespace Orbit::Motor
