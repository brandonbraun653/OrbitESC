#include "model/TelemetryBuffer.h"

#include <QtMath>

namespace orbit::model
{
  TelemetryBuffer::TelemetryBuffer( QObject *parent ) : QObject( parent )
  {
    m_samples.reserve( MaxSamples );
  }

  void TelemetryBuffer::append( quint32 timestampUs, float ia, float ib, float ic )
  {
    const qint64 timestamp = static_cast<qint64>( timestampUs );
    if( m_latestTimestampUs > 0 && timestamp + 1000000 < m_latestTimestampUs )
    {
      clear();
    }

    m_latestTimestampUs = timestamp;
    m_samples.append( Sample{ timestamp, ia, ib, ic } );
    if( m_samples.size() > MaxSamples )
    {
      m_samples.remove( 0, m_samples.size() - MaxSamples );
    }

    emit updated();
  }

  void TelemetryBuffer::clear()
  {
    m_samples.clear();
    m_latestTimestampUs = 0;
    emit updated();
  }

  QVariantMap TelemetryBuffer::windowData( double windowSeconds ) const
  {
    QVariantMap result;
    result.insert( "count", m_samples.size() );
    result.insert( "rate", sampleRate() );

    if( m_samples.isEmpty() )
    {
      result.insert( "times", QVariantList{} );
      result.insert( "ia", QVariantList{} );
      result.insert( "ib", QVariantList{} );
      result.insert( "ic", QVariantList{} );
      result.insert( "min", -1.0 );
      result.insert( "max", 1.0 );
      return result;
    }

    const qint64 windowUs = static_cast<qint64>( qMax( 0.1, windowSeconds ) * 1000000.0 );
    const qint64 cutoff   = m_latestTimestampUs - windowUs;

    QVariantList times;
    QVariantList ia;
    QVariantList ib;
    QVariantList ic;
    double minValue = 0.0;
    double maxValue = 0.0;
    bool first = true;

    for( const Sample &sample : m_samples )
    {
      if( sample.timestampUs < cutoff )
      {
        continue;
      }

      const double timeSeconds = static_cast<double>( sample.timestampUs - m_latestTimestampUs ) / 1000000.0;
      times.append( timeSeconds );
      ia.append( sample.ia );
      ib.append( sample.ib );
      ic.append( sample.ic );

      const double values[] = { sample.ia, sample.ib, sample.ic };
      for( double value : values )
      {
        if( first )
        {
          minValue = maxValue = value;
          first = false;
        }
        else
        {
          minValue = qMin( minValue, value );
          maxValue = qMax( maxValue, value );
        }
      }
    }

    if( first )
    {
      minValue = -1.0;
      maxValue = 1.0;
    }
    else if( qFuzzyCompare( minValue, maxValue ) )
    {
      minValue -= 0.5;
      maxValue += 0.5;
    }

    result.insert( "times", times );
    result.insert( "ia", ia );
    result.insert( "ib", ib );
    result.insert( "ic", ic );
    result.insert( "min", minValue );
    result.insert( "max", maxValue );
    return result;
  }

  double TelemetryBuffer::rmsCurrent() const
  {
    if( m_samples.isEmpty() )
    {
      return -1.0;
    }

    const qint64 cutoff = m_latestTimestampUs - 1000000;
    double sum = 0.0;
    int count = 0;
    for( auto it = m_samples.crbegin(); it != m_samples.crend(); ++it )
    {
      if( it->timestampUs < cutoff )
      {
        break;
      }
      sum += static_cast<double>( it->ia ) * it->ia;
      sum += static_cast<double>( it->ib ) * it->ib;
      sum += static_cast<double>( it->ic ) * it->ic;
      ++count;
    }

    return count > 0 ? qSqrt( sum / ( 3.0 * count ) ) : -1.0;
  }

  double TelemetryBuffer::peakCurrent() const
  {
    if( m_samples.isEmpty() )
    {
      return -1.0;
    }

    const qint64 cutoff = m_latestTimestampUs - 1000000;
    double peak = 0.0;
    for( auto it = m_samples.crbegin(); it != m_samples.crend(); ++it )
    {
      if( it->timestampUs < cutoff )
      {
        break;
      }
      peak = qMax( peak, qAbs( static_cast<double>( it->ia ) ) );
      peak = qMax( peak, qAbs( static_cast<double>( it->ib ) ) );
      peak = qMax( peak, qAbs( static_cast<double>( it->ic ) ) );
    }
    return peak;
  }

  double TelemetryBuffer::sampleRate() const
  {
    if( m_samples.size() < 2 )
    {
      return 0.0;
    }

    const int count = qMin( 16, m_samples.size() );
    const qint64 delta = m_samples.last().timestampUs - m_samples.at( m_samples.size() - count ).timestampUs;
    return delta > 0 ? ( count - 1 ) * 1000000.0 / static_cast<double>( delta ) : 0.0;
  }
}    // namespace orbit::model
