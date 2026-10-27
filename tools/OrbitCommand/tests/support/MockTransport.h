#pragma once

#include <gmock/gmock.h>

#include "device/Transport.h"

namespace orbit::test
{
  class MockTransport : public orbit::device::Transport
  {
  public:
    MOCK_METHOD( bool, open, ( const orbit::device::ConnectionTarget &target ), ( override ) );
    MOCK_METHOD( void, close, (), ( override ) );
    MOCK_METHOD( void, write, ( const QByteArray &data ), ( override ) );
    MOCK_METHOD( bool, isOpen, (), ( const, override ) );

    void emitOpened()
    {
      emit opened();
    }

    void emitClosed()
    {
      emit closed();
    }

    void emitData( const QByteArray &data )
    {
      emit dataReceived( data );
    }

    void emitError( const QString &message )
    {
      emit errorOccurred( message );
    }
  };
}    // namespace orbit::test
