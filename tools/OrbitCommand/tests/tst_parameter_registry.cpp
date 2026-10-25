#include <QtTest>

#include "model/ParameterModel.h"
#include "model/ParameterRegistry.h"
#include "protocol/ParameterCodec.h"

using namespace orbit::model;

class TestParameterRegistry : public QObject
{
  Q_OBJECT

private slots:
  void firmwareTableIsComplete()
  {
    const auto &all = registry::all();
    QCOMPARE( all.size(), 36 );

    int information = 0;
    int system = 0;
    int motorControl = 0;
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

    QCOMPARE( information, 7 );
    QCOMPARE( system, 3 );
    QCOMPARE( motorControl, 18 );
    QCOMPARE( motorAttributes, 8 );
  }

  void firmwareOnlyParametersExist()
  {
    QVERIFY( registry::find( ParamId_PARAM_PWM_DRIVE_MAX_DUTY ) != nullptr );
    QVERIFY( registry::find( ParamId_PARAM_MIN_ARM_VOLTAGE ) != nullptr );
    QVERIFY( registry::find( ParamId_PARAM_MAX_ARM_VOLTAGE ) != nullptr );
  }

  void readOnlyInformation()
  {
    const auto *bootCount = registry::find( ParamId_PARAM_BOOT_COUNT );
    QVERIFY( bootCount );
    QVERIFY( bootCount->readOnly );

    const auto *serial = registry::find( ParamId_PARAM_SERIAL_NUMBER );
    QVERIFY( serial );
    QVERIFY( !serial->readOnly );
  }

  void modelTracksDirtyValues()
  {
    ParameterModel model( registry::forGroup( ParameterGroup::MotorControl ) );
    const int id = ParamId_PARAM_STATOR_PWM_FREQ;

    QVERIFY( model.setValueForParamId( id, 12000.0, true ) );
    QCOMPARE( model.dirtyCount(), 1 );
    QVERIFY( model.setValueForParamId( id, 15000.0, true ) );
    QCOMPARE( model.dirtyCount(), 1 );

    model.setClean( id );
    QCOMPARE( model.dirtyCount(), 0 );
  }

  void readOnlyModelRejectsWrites()
  {
    ParameterModel model( registry::forGroup( ParameterGroup::Information ) );
    QVERIFY( !model.setValueForParamId( ParamId_PARAM_BOOT_COUNT, 1, true ) );
  }

  void stringLimitIsExposed()
  {
    const auto *description = registry::find( ParamId_PARAM_DESCRIPTION );
    QVERIFY( description );
    QCOMPARE( description->maxStringLength, 32 );
  }
};

QTEST_GUILESS_MAIN( TestParameterRegistry )
#include "tst_parameter_registry.moc"
