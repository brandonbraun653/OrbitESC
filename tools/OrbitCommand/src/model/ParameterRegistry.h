#pragma once

#include <QString>
#include <QVector>

#include "system_config.pb.h"

namespace orbit::model
{
  enum class ParameterGroup
  {
    Information,
    SystemBehavior,
    MotorControl,
    MotorAttributes
  };

  struct ParameterDescriptor
  {
    int id = -1;
    QString key;
    QString label;
    ParameterGroup group = ParameterGroup::Information;
    ParamType type = ParamType_UNKNOWN;
    bool readOnly = false;
    QString unit;
    int decimals = 0;
    double minValue = 0.0;
    double maxValue = 0.0;
    int maxStringLength = 32;
  };

  namespace registry
  {
    const QVector<ParameterDescriptor> &all();
    QVector<ParameterDescriptor> forGroup( ParameterGroup group );
    QString groupName( ParameterGroup group );
    const ParameterDescriptor *find( int paramId );
    bool isValidParamId( int paramId );
  }    // namespace registry
}    // namespace orbit::model
