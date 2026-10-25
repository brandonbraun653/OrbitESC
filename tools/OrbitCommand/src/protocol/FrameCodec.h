#pragma once

#include <QByteArray>
#include <QList>

namespace orbit::protocol
{
  inline constexpr int MaxPayloadSize = 512;
  inline constexpr int MaxBufferedBytes = 65536;

  QByteArray encodeFrame( const QByteArray &payload );

  struct ParsedFrames
  {
    QList<QByteArray> payloads;
    int invalidFrames = 0;
    bool overflowed   = false;
  };

  class FrameAccumulator
  {
  public:
    void append( const QByteArray &bytes );
    ParsedFrames takeFrames();
    void clear();

  private:
    QByteArray m_buffer;
    bool m_overflowed = false;
  };
}    // namespace orbit::protocol
