#pragma once

#include <QObject>
#include <QVariantMap>
#include <QVector>

namespace orbit::model
{
  class TelemetryBuffer : public QObject
  {
    Q_OBJECT
    Q_PROPERTY( int count READ count NOTIFY updated )

  public:
    explicit TelemetryBuffer( QObject *parent = nullptr );

    int count() const
    {
      return m_samples.size();
    }

    void append( quint32 timestampUs, float ia, float ib, float ic );
    void clear();

    Q_INVOKABLE QVariantMap windowData( double windowSeconds ) const;
    Q_INVOKABLE double rmsCurrent() const;
    Q_INVOKABLE double peakCurrent() const;
    Q_INVOKABLE double sampleRate() const;

  signals:
    void updated();

  private:
    struct Sample
    {
      qint64 timestampUs = 0;
      float ia = 0.0f;
      float ib = 0.0f;
      float ic = 0.0f;
    };

    static constexpr int MaxSamples = 4096;
    QVector<Sample> m_samples;
    qint64 m_latestTimestampUs = 0;
  };
}    // namespace orbit::model
