#include "protocol/ConsoleAssembler.h"

#include <QDateTime>

namespace orbit::protocol
{
  namespace
  {
    constexpr qint64 StaleBufferMs = 30000;
  }

  std::optional<QString> ConsoleAssembler::add( const ConsoleMessage &message )
  {
    if( message.total_frames == 0 || message.this_frame == 0 || message.this_frame > message.total_frames )
    {
      return std::nullopt;
    }

    Buffer &buffer = m_buffers[message.header.uuid];
    if( buffer.totalFrames != static_cast<int>( message.total_frames ) || buffer.frames.size() != buffer.totalFrames )
    {
      buffer.frames.resize( message.total_frames );
      buffer.received.fill( false, message.total_frames );
      buffer.receivedCount = 0;
      buffer.totalFrames   = message.total_frames;
      buffer.startedAt     = QDateTime::currentMSecsSinceEpoch();
    }

    const int index = static_cast<int>( message.this_frame ) - 1;
    if( !buffer.received.at( index ) )
    {
      buffer.frames[index] = QString::fromUtf8( reinterpret_cast<const char *>( message.data.bytes ),
                                                static_cast<int>( message.data.size ) );
      buffer.received[index] = true;
      buffer.receivedCount++;
    }

    if( buffer.receivedCount < buffer.totalFrames )
    {
      return std::nullopt;
    }

    const QString line = buffer.frames.join( QString{} );
    m_buffers.remove( message.header.uuid );
    return line;
  }

  QStringList ConsoleAssembler::expire( qint64 nowMs )
  {
    QStringList expired;
    for( auto it = m_buffers.begin(); it != m_buffers.end(); )
    {
      if( nowMs - it.value().startedAt > StaleBufferMs )
      {
        expired.append( QStringLiteral( "Dropped incomplete console message" ) );
        it = m_buffers.erase( it );
      }
      else
      {
        ++it;
      }
    }
    return expired;
  }
}    // namespace orbit::protocol
