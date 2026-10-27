#pragma once

#include <QList>
#include <QTimer>

#include <functional>
#include <optional>

#include "device/Transport.h"
#include "protocol/FrameCodec.h"
#include "protocol/MessageCodec.h"

namespace orbit::test
{
  class FakeTransport : public orbit::device::Transport
  {
  public:
    std::function<void( const QByteArray & )> onWrite;
    QList<QByteArray> writes;
    int openCount  = 0;
    int closeCount = 0;
    bool openResult = true;

    bool open( const orbit::device::ConnectionTarget &target ) override
    {
      Q_UNUSED( target )
      openCount++;
      QTimer::singleShot( 0, this, [this]() { emit opened(); } );
      return openResult;
    }

    void close() override
    {
      closeCount++;
      QTimer::singleShot( 0, this, [this]() { emit closed(); } );
    }

    void write( const QByteArray &data ) override
    {
      writes.append( data );
      if( onWrite )
      {
        const QByteArray copy = data;
        QTimer::singleShot( 0, this, [this, copy]() { onWrite( copy ); } );
      }
    }

    bool isOpen() const override
    {
      return true;
    }

    void inject( const QByteArray &payload )
    {
      emit dataReceived( orbit::protocol::encodeFrame( payload ) );
    }

    void injectRaw( const QByteArray &data )
    {
      emit dataReceived( data );
    }

    void injectFrame( const QByteArray &payload )
    {
      inject( payload );
    }

    std::optional<orbit::protocol::HeaderView> lastHeader() const
    {
      if( writes.isEmpty() )
      {
        return std::nullopt;
      }

      orbit::protocol::FrameAccumulator accumulator;
      accumulator.append( writes.last() );
      const auto frames = accumulator.takeFrames();
      return frames.payloads.isEmpty() ? std::nullopt : orbit::protocol::peekHeader( frames.payloads.first() );
    }
  };
}    // namespace orbit::test
