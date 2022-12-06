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
#include <etl/fsm.h>
#include <etl/message.h>
#include <src/control/foc_data.hpp>
#include <src/control/modes/sys_mode_base.hpp>


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
  class FOC : public etl::fsm
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
     * @brief Calibrate system measurements
     */
    void calibrate();

    /**
     * @brief Executes the high level controller
     * @note Expects to be run periodically
     */
    void run();

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

    /**
     * @brief Gets the current system mode of the FOC driver
     *
     * @return ModeId_t
     */
    ModeId_t currentMode() const;

    /**
     * @brief Common handler for logging unexpected messages in the current state
     *
     * @param msg   The event that was received
     */
    void logUnhandledMessage( const etl::imessage &msg );


    /*-------------------------------------------------------------------------
    Public Data:
      While not ideal, this allows the state machine to implement all the
      transition details.
    -------------------------------------------------------------------------*/
    SuperState                       mState;            /**< Entire FOC subsystem state */
    FOCConfig                        mConfig;           /**< Configuration data for the FOC library */
    Chimera::ADC::Driver_rPtr        mADCDriver;        /**< ADC Hardware Driver */
    Chimera::Timer::Inverter::Driver mTimerDriver;      /**< Motor drive timer */
    Chimera::Timer::Trigger::Master  mSpeedCtrlTrigger; /**< Trigger for the speed control loop */

  protected:
    /**
     * @brief Interrupt handler for the ADC
     *
     * @param isr   Data from the ADC interrupt
     */
    void adcISRTxfrComplete( const Chimera::ADC::InterruptDetail &isr );

    /**
     * @brief Interrupt handler for a periodic timer to do the speed control loop
     */
    void timer_isr_speed_controller();

    /**
     * @brief Interrupt context handler to control rotor ramp from park to run
     */
    void isr_rotor_ramp_controller();


  private:
    bool                                                    mInitialized;   /**< Driver initialized state */
    std::array<etl::ifsm_state *, ModeId::NUM_STATES>       mFSMStateArray; /**< Storage for the FSM state controllers */
    std::array<void ( FOC::* )( void ), ModeId::NUM_STATES> mRunFuncArray;  /**< Lookup for periodic state behavior */


    void stepEMFObserver( const float dt );
    void stepIControl( const float dt );
    void stepEstimator( const float dt );

    void onFault();
    void onArmed();
    void onPark();
    void onRamp();
    void onRun();
  };
}    // namespace Orbit::Control

#endif /* !ORBIT_ESC_FOC_CONTROL_HPP */
