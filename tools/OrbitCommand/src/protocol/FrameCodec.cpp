#include "protocol/FrameCodec.h"

#include <cobs.h>

namespace orbit::protocol
{
  QByteArray encodeFrame( const QByteArray &payload )
  {
    if( payload.size() > MaxPayloadSize )
    {
      return {};
    }

    QByteArray output;
    output.resize( static_cast<int>( COBS_ENCODE_DST_BUF_LEN_MAX( static_cast<size_t>( payload.size() ) ) ) );

    const cobs_encode_result result = cobs_encode( output.data(), static_cast<size_t>( output.size() ),
                                                   payload.constData(), static_cast<size_t>( payload.size() ) );
    if( result.status != COBS_ENCODE_OK )
    {
      return {};
    }

    output.resize( static_cast<int>( result.out_len ) );
    output.append( '\0' );
    return output;
  }

  void FrameAccumulator::append( const QByteArray &bytes )
  {
    if( bytes.isEmpty() )
    {
      return;
    }

    m_buffer.append( bytes );
    if( m_buffer.size() > MaxBufferedBytes )
    {
      m_buffer.clear();
      m_overflowed = true;
    }
  }

  ParsedFrames FrameAccumulator::takeFrames()
  {
    ParsedFrames result;
    result.overflowed = m_overflowed;
    m_overflowed      = false;

    while( true )
    {
      const int terminator = m_buffer.indexOf( '\0' );
      if( terminator < 0 )
      {
        break;
      }

      const QByteArray frame = m_buffer.left( terminator );
      m_buffer.remove( 0, terminator + 1 );

      if( frame.isEmpty() )
      {
        continue;
      }

      QByteArray decoded;
      decoded.resize( static_cast<int>( COBS_DECODE_DST_BUF_LEN_MAX( static_cast<size_t>( frame.size() ) ) ) );

      const cobs_decode_result decoded_result = cobs_decode( decoded.data(), static_cast<size_t>( decoded.size() ),
                                                             frame.constData(), static_cast<size_t>( frame.size() ) );
      if( decoded_result.status != COBS_DECODE_OK || decoded_result.out_len > static_cast<size_t>( MaxPayloadSize ) )
      {
        result.invalidFrames++;
        continue;
      }

      decoded.resize( static_cast<int>( decoded_result.out_len ) );
      result.payloads.append( decoded );
    }

    return result;
  }

  void FrameAccumulator::clear()
  {
    m_buffer.clear();
    m_overflowed = false;
  }
}    // namespace orbit::protocol
