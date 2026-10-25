#include <QtTest>

#include "model/TelemetryBuffer.h"

using namespace orbit::model;

class TestTelemetryBuffer : public QObject
{
  Q_OBJECT

private slots:
  void windowContainsSamples()
  {
    TelemetryBuffer buffer;
    buffer.append( 1000000, 1.0f, 2.0f, 3.0f );
    buffer.append( 1010000, 4.0f, 5.0f, 6.0f );

    const QVariantMap data = buffer.windowData( 10.0 );
    QCOMPARE( data.value( "count" ).toInt(), 2 );
    QCOMPARE( data.value( "times" ).toList().size(), 2 );
    QCOMPARE( data.value( "ia" ).toList().size(), 2 );
    QCOMPARE( data.value( "min" ).toDouble(), 1.0 );
    QCOMPARE( data.value( "max" ).toDouble(), 6.0 );
  }

  void rmsAndPeak()
  {
    TelemetryBuffer buffer;
    buffer.append( 1000000, 3.0f, 4.0f, 0.0f );

    QCOMPARE( buffer.peakCurrent(), 4.0 );
    QVERIFY( qAbs( buffer.rmsCurrent() - 2.8867513 ) < 0.0001 );
  }

  void timestampResetClearsBuffer()
  {
    TelemetryBuffer buffer;
    buffer.append( 5000000, 1.0f, 1.0f, 1.0f );
    buffer.append( 100, 2.0f, 2.0f, 2.0f );
    QCOMPARE( buffer.count(), 1 );
  }

  void sampleRateIsComputed()
  {
    TelemetryBuffer buffer;
    buffer.append( 0, 0.0f, 0.0f, 0.0f );
    buffer.append( 10000, 0.0f, 0.0f, 0.0f );
    buffer.append( 20000, 0.0f, 0.0f, 0.0f );
    QVERIFY( qAbs( buffer.sampleRate() - 100.0 ) < 0.001 );
  }

  void clearResetsValues()
  {
    TelemetryBuffer buffer;
    buffer.append( 1000000, 1.0f, 1.0f, 1.0f );
    buffer.clear();
    QCOMPARE( buffer.count(), 0 );
    QCOMPARE( buffer.rmsCurrent(), -1.0 );
    QCOMPARE( buffer.peakCurrent(), -1.0 );
  }
};

QTEST_GUILESS_MAIN( TestTelemetryBuffer )
#include "tst_telemetry.moc"
