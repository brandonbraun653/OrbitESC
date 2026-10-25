#include <QtTest>

#include "protocol/FrameCodec.h"

using namespace orbit::protocol;

class TestFrameCodec : public QObject
{
  Q_OBJECT

private slots:
  void knownVector()
  {
    const QByteArray payload = QByteArray::fromHex( "11220033" );
    QCOMPARE( encodeFrame( payload ), QByteArray::fromHex( "031122023300" ) );
  }

  void roundTrip()
  {
    FrameAccumulator accumulator;
    const QByteArray payload = QByteArray::fromHex( "001122003344" );
    accumulator.append( encodeFrame( payload ) );

    const ParsedFrames frames = accumulator.takeFrames();
    QCOMPARE( frames.payloads.size(), 1 );
    QCOMPARE( frames.payloads.first(), payload );
    QCOMPARE( frames.invalidFrames, 0 );
  }

  void partialAndMultipleFrames()
  {
    FrameAccumulator accumulator;
    const QByteArray first = encodeFrame( QByteArray( "first" ) );
    const QByteArray second = encodeFrame( QByteArray( "second" ) );

    accumulator.append( first.left( 2 ) );
    QCOMPARE( accumulator.takeFrames().payloads.size(), 0 );

    accumulator.append( first.mid( 2 ) + second );
    const ParsedFrames frames = accumulator.takeFrames();
    QCOMPARE( frames.payloads.size(), 2 );
    QCOMPARE( frames.payloads.at( 0 ), QByteArray( "first" ) );
    QCOMPARE( frames.payloads.at( 1 ), QByteArray( "second" ) );
  }

  void emptyFramesAreIgnored()
  {
    FrameAccumulator accumulator;
    accumulator.append( QByteArray( "\0\0\0", 3 ) );
    const ParsedFrames frames = accumulator.takeFrames();
    QCOMPARE( frames.payloads.size(), 0 );
    QCOMPARE( frames.invalidFrames, 0 );
  }

  void invalidFrameIsReported()
  {
    FrameAccumulator accumulator;
    accumulator.append( QByteArray::fromHex( "0501" ) + QByteArray( 1, '\0' ) );
    const ParsedFrames frames = accumulator.takeFrames();
    QCOMPARE( frames.payloads.size(), 0 );
    QCOMPARE( frames.invalidFrames, 1 );
  }

  void overflowIsReported()
  {
    FrameAccumulator accumulator;
    accumulator.append( QByteArray( MaxBufferedBytes + 1, '\x11' ) );
    const ParsedFrames frames = accumulator.takeFrames();
    QVERIFY( frames.overflowed );
  }

  void oversizedPayloadIsRejected()
  {
    QVERIFY( encodeFrame( QByteArray( MaxPayloadSize + 1, '\x11' ) ).isEmpty() );
  }
};

QTEST_GUILESS_MAIN( TestFrameCodec )
#include "tst_frame_codec.moc"
