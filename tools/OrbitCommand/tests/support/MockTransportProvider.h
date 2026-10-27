#pragma once

#include <gmock/gmock.h>

#include "device/TransportProvider.h"

namespace orbit::test
{
  class MockTransportProvider : public orbit::device::TransportProvider
  {
  public:
    MOCK_METHOD( QString, kind, (), ( const, override ) );
    MOCK_METHOD( QList<orbit::device::ConnectionTarget>, discover, (), ( const, override ) );
    MOCK_METHOD( orbit::device::Transport *, create,
                 ( const orbit::device::ConnectionTarget &target, QObject *parent ), ( const, override ) );
  };
}    // namespace orbit::test
