#include <gtest/gtest.h>

#include "model/ParameterModel.h"
#include "model/ParameterRegistry.h"
#include "protocol/ParameterCodec.h"

using namespace orbit::model;

TEST( ParameterRegistry, FirmwareTableIsComplete )
{
  const auto &all = registry::all();
  EXPECT_EQ( all.size(), 36 );

  int information     = 0;
  int system          = 0;
  int motorControl    = 0;
  int motorAttributes = 0;
  for( const auto &descriptor : all )
  {
    switch( descriptor.group )
    {
      case ParameterGroup::Information:
        information++;
        break;
      case ParameterGroup::SystemBehavior:
        system++;
        break;
      case ParameterGroup::MotorControl:
        motorControl++;
        break;
      case ParameterGroup::MotorAttributes:
        motorAttributes++;
        break;
    }
  }

  EXPECT_EQ( information, 7 );
  EXPECT_EQ( system, 3 );
  EXPECT_EQ( motorControl, 18 );
  EXPECT_EQ( motorAttributes, 8 );
}

TEST( ParameterRegistry, FirmwareOnlyParametersExist )
{
  EXPECT_NE( registry::find( ParamId_PARAM_PWM_DRIVE_MAX_DUTY ), nullptr );
  EXPECT_NE( registry::find( ParamId_PARAM_MIN_ARM_VOLTAGE ), nullptr );
  EXPECT_NE( registry::find( ParamId_PARAM_MAX_ARM_VOLTAGE ), nullptr );
}

TEST( ParameterRegistry, ReadOnlyInformation )
{
  const auto *bootCount = registry::find( ParamId_PARAM_BOOT_COUNT );
  ASSERT_NE( bootCount, nullptr );
  EXPECT_TRUE( bootCount->readOnly );

  const auto *serial = registry::find( ParamId_PARAM_SERIAL_NUMBER );
  ASSERT_NE( serial, nullptr );
  EXPECT_FALSE( serial->readOnly );
}

TEST( ParameterRegistry, ModelTracksDirtyValues )
{
  ParameterModel model( registry::forGroup( ParameterGroup::MotorControl ) );
  const int id = ParamId_PARAM_STATOR_PWM_FREQ;

  EXPECT_TRUE( model.setValueForParamId( id, 12000.0, true ) );
  EXPECT_EQ( model.dirtyCount(), 1 );
  EXPECT_TRUE( model.setValueForParamId( id, 15000.0, true ) );
  EXPECT_EQ( model.dirtyCount(), 1 );

  model.setClean( id );
  EXPECT_EQ( model.dirtyCount(), 0 );
}

TEST( ParameterRegistry, ReadOnlyModelRejectsWrites )
{
  ParameterModel model( registry::forGroup( ParameterGroup::Information ) );
  EXPECT_FALSE( model.setValueForParamId( ParamId_PARAM_BOOT_COUNT, 1, true ) );
}

TEST( ParameterRegistry, StringLimitIsExposed )
{
  const auto *description = registry::find( ParamId_PARAM_DESCRIPTION );
  ASSERT_NE( description, nullptr );
  EXPECT_EQ( description->maxStringLength, 32 );
}
