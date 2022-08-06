/******************************************************************************
 *  File Name:
 *    foc_driver.hpp
 *
 *  Description:
 *    Field Oriented Control (FOC) Driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef ORBIT_ESC_FOC_CONTROL_HPP
#define ORBIT_ESC_FOC_CONTROL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <src/control/foc_data.hpp>
#include <src/control/modes/sys_mode_armed.hpp>
#include <src/control/modes/sys_mode_base.hpp>
#include <src/control/modes/sys_mode_fault.hpp>
#include <src/control/modes/sys_mode_idle.hpp>
#include <src/control/modes/sys_mode_park.hpp>
#include <src/control/modes/sys_mode_ramp.hpp>
#include <src/control/modes/sys_mode_run.hpp>


namespace Orbit::Control
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class FOC;

  /*---------------------------------------------------------------------------
  Public Data
  ---------------------------------------------------------------------------*/
  extern FOC FOCDriver;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct FOCConfig
  {
    Chimera::ADC::Peripheral                    adcSource; /**< Which ADC peripheral to use */
    std::array<ADCTxfrFunc, ADC_CH_NUM_OPTIONS> txfrFuncs; /**< Conversion functions for each ADC channel */

    void clear()
    {
      adcSource = Chimera::ADC::Peripheral::UNKNOWN;
      txfrFuncs.fill( nullptr );
    }
  };


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Brush-less dc motor control library implemented with FOC
   *
   */
  class FOC
  {
  public:
    FOC();
    ~FOC();

    /**
     * @brief Power up the FOC library
     *
     * @param cfg           Configuration data for the FOC library
     * @param motorParams   Motor parameters for the motor being controlled
     * @return int
     */
    int initialize( const FOCConfig &cfg, const MotorParameters &motorParams );

    /**
     * @brief Move from an idle state to prepared for running
     *
     * @return int
     */
    int arm();

    /**
     * @brief Revert to SW idle state and disable all motor control outputs
     *
     * @return int
     */
    int disarm();

    /**
     * @brief Start closed loop control of motor speed
     * @note Requires the motor to be armed
     *
     * @return int
     */
    int engage();

    /**
     * @brief Stop controlling motor speed and revert to arm mode
     * @note Requires controller to be engaged
     *
     * @return int
     */
    int disengage();

    /**
     * @brief Set a new speed reference for the motor
     * @note Requires arm or engage mode
     *
     * Assigns a new set-point for the motor control system. Only takes physical effect on
     * the motor once the system is engaged.
     *
     * @param ref        New speed reference in rpm
     * @return int
     */
    int setSpeedRef( const float ref );

    /**
     * @brief Instructs an emergency halt of the motor
     *
     * @return int
     */
    int emergencyStop();

    /**
     * @brief Gets the last data collected from the ADC
     *
     * @param data  The data to fill with the last ADC data
     */
    void lastSensorData( ADCSensorBuffer &data );

    /**
     * @brief Gets a view of the internal state of the FOC driver
     *
     * @return const SuperState&
     */
    const SuperState &dbgGetState() const;

  protected:
    /**
     * @brief Interrupt handler for the ADC
     *
     * @param isr   Data from the ADC interrupt
     */
    void dma_isr_current_controller( const Chimera::ADC::InterruptDetail &isr );

    /**
     * @brief Interrupt handler for a periodic timer to do the speed control loop
     */
    void timer_isr_speed_controller();

    /*-------------------------------------------------------------------------
    State Machine Variables
    -------------------------------------------------------------------------*/
    FSMMotorControl    mFSM;        /**< Root controller instance */
    State::Idle        mIdleState;  /**< Idle state controller */
    State::Armed       mArmedState; /**< Armed state controller */
    State::Fault       mFaultState; /**< Fault state controller */
    State::EngagedPark mParkState;  /**< Park state controller */
    State::EngagedRamp mRampState;  /**< Ramp state controller */
    State::EngagedRun  mRunState;   /**< Run state controller */

    /* Track the available instances */
    std::array<etl::ifsm_state *, StateId::NUM_STATES> mFSMStateArray;

  private:
    Chimera::ADC::Driver_rPtr        mADCDriver;
    Chimera::Timer::Inverter::Driver mTimerDriver;
    Chimera::Timer::Trigger::Master  mSpeedCtrlTrigger;

    SuperState mState;
    FOCConfig  mConfig;

    /**
     * @brief Calculates back-EMF estimates along the D and Q axes
     *
     * @param dt  The time in seconds since the last call to this function
     */
    void stepEMFObserver( const float dt );
  };
}    // namespace Orbit::Control

#endif /* !ORBIT_ESC_FOC_CONTROL_HPP */
