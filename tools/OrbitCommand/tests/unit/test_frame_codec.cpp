#include <gtest/gtest.h>

#include "protocol/FrameCodec.h"

using namespace orbit::protocol;

TEST( FrameCodec, KnownVector )
{
  const QByteArray payload = QByteArray::fromHex( "11220033" );
  EXPECT_EQ( encodeFrame( payload ), QByteArray::fromHex( "031122023300" ) );
}

TEST( FrameCodec, RoundTrip )
{
  FrameAccumulator accumulator;
  const QByteArray payload = QByteArray::fromHex( "001122003344" );
  accumulator.append( encodeFrame( payload ) );

  const ParsedFrames frames = accumulator.takeFrames();
  ASSERT_EQ( frames.payloads.size(), 1 );
  EXPECT_EQ( frames.payloads.first(), payload );
  EXPECT_EQ( frames.invalidFrames, 0 );
}

TEST( FrameCodec, PartialAndMultipleFrames )
{
  FrameAccumulator accumulator;
  const QByteArray first  = encodeFrame( QByteArray( "first" ) );
  const QByteArray second = encodeFrame( QByteArray( "second" ) );

  accumulator.append( first.left( 2 ) );
  EXPECT_EQ( accumulator.takeFrames().payloads.size(), 0 );

  accumulator.append( first.mid( 2 ) + second );
  const ParsedFrames frames = accumulator.takeFrames();
  ASSERT_EQ( frames.payloads.size(), 2 );
  EXPECT_EQ( frames.payloads.at( 0 ), QByteArray( "first" ) );
  EXPECT_EQ( frames.payloads.at( 1 ), QByteArray( "second" ) );
}

TEST( FrameCodec, EmptyFramesAreIgnored )
{
  FrameAccumulator accumulator;
  accumulator.append( QByteArray( "\0\0\0", 3 ) );
  const ParsedFrames frames = accumulator.takeFrames();
  EXPECT_EQ( frames.payloads.size(), 0 );
  EXPECT_EQ( frames.invalidFrames, 0 );
}

TEST( FrameCodec, InvalidFrameIsReported )
{
  FrameAccumulator accumulator;
  accumulator.append( QByteArray::fromHex( "0501" ) + QByteArray( 1, '\0' ) );
  const ParsedFrames frames = accumulator.takeFrames();
  EXPECT_EQ( frames.payloads.size(), 0 );
  EXPECT_EQ( frames.invalidFrames, 1 );
}

TEST( FrameCodec, OverflowIsReported )
{
  FrameAccumulator accumulator;
  accumulator.append( QByteArray( MaxBufferedBytes + 1, '\x11' ) );
  const ParsedFrames frames = accumulator.takeFrames();
  EXPECT_TRUE( frames.overflowed );
}

TEST( FrameCodec, OversizedPayloadIsRejected )
{
  EXPECT_TRUE( encodeFrame( QByteArray( MaxPayloadSize + 1, '\x11' ) ).isEmpty() );
}
