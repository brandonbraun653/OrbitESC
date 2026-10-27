#include <gtest/gtest.h>

#include <QtGlobal>

#include "model/TelemetryBuffer.h"

using namespace orbit::model;

TEST( TelemetryBuffer, WindowContainsSamples )
{
  TelemetryBuffer buffer;
  buffer.append( 1000000, 1.0f, 2.0f, 3.0f );
  buffer.append( 1010000, 4.0f, 5.0f, 6.0f );

  const QVariantMap data = buffer.windowData( 10.0 );
  EXPECT_EQ( data.value( "count" ).toInt(), 2 );
  EXPECT_EQ( data.value( "times" ).toList().size(), 2 );
  EXPECT_EQ( data.value( "ia" ).toList().size(), 2 );
  EXPECT_DOUBLE_EQ( data.value( "min" ).toDouble(), 1.0 );
  EXPECT_DOUBLE_EQ( data.value( "max" ).toDouble(), 6.0 );
}

TEST( TelemetryBuffer, RmsAndPeak )
{
  TelemetryBuffer buffer;
  buffer.append( 1000000, 3.0f, 4.0f, 0.0f );

  EXPECT_DOUBLE_EQ( buffer.peakCurrent(), 4.0 );
  EXPECT_LT( qAbs( buffer.rmsCurrent() - 2.8867513 ), 0.0001 );
}

TEST( TelemetryBuffer, TimestampResetClearsBuffer )
{
  TelemetryBuffer buffer;
  buffer.append( 5000000, 1.0f, 1.0f, 1.0f );
  buffer.append( 100, 2.0f, 2.0f, 2.0f );
  EXPECT_EQ( buffer.count(), 1 );
}

TEST( TelemetryBuffer, SampleRateIsComputed )
{
  TelemetryBuffer buffer;
  buffer.append( 0, 0.0f, 0.0f, 0.0f );
  buffer.append( 10000, 0.0f, 0.0f, 0.0f );
  buffer.append( 20000, 0.0f, 0.0f, 0.0f );
  EXPECT_LT( qAbs( buffer.sampleRate() - 100.0 ), 0.001 );
}

TEST( TelemetryBuffer, ClearResetsValues )
{
  TelemetryBuffer buffer;
  buffer.append( 1000000, 1.0f, 1.0f, 1.0f );
  buffer.clear();
  EXPECT_EQ( buffer.count(), 0 );
  EXPECT_DOUBLE_EQ( buffer.rmsCurrent(), -1.0 );
  EXPECT_DOUBLE_EQ( buffer.peakCurrent(), -1.0 );
}
