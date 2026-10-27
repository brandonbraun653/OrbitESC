#pragma once

#include <QHash>
#include <QList>
#include <QTcpServer>
#include <QTimer>
#include <QVariant>

#include "protocol/FrameCodec.h"
#include "protocol/MessageCodec.h"
#include "protocol/ParameterCodec.h"

class QTcpSocket;

namespace orbit::test
{
  class FakeDeviceServer : public QTcpServer
  {
    Q_OBJECT
  public:
    explicit FakeDeviceServer( QObject *parent = nullptr );

    bool start( quint16 port = 0 );
    quint16 port() const;

    void setTicksEnabled( bool enabled );
    void setTickIntervalMs( int intervalMs );
    void setStatusIntervalMs( int intervalMs );
    void setStreamEnabled( bool enabled );
    void setStreamIntervalMs( int intervalMs );
    void setParameter( int paramId, ParamType type, const QVariant &value );
    void setMotorState( MotorCtrlState state );

    const QList<QByteArray> &receivedPayloads() const
    {
      return m_receivedPayloads;
    }

  private slots:
    void acceptConnection();
    void readFromClient();
    void sendTick();
    void sendStatus();
    void sendStreamSample();

  private:
    void processPayload( const QByteArray &payload );
    void sendPayload( const QByteArray &payload );
    void stopTimers();

    QTcpSocket *m_peer = nullptr;
    orbit::protocol::FrameAccumulator m_accumulator;
    QTimer m_tickTimer;
    QTimer m_statusTimer;
    QTimer m_streamTimer;
    QList<QByteArray> m_receivedPayloads;
    QHash<int, QVariant> m_parameterValues;
    QHash<int, ParamType> m_parameterTypes;
    MotorCtrlState m_motorState = MotorCtrlState_MOTOR_CTRL_STATE_IDLE;
    quint32 m_tick             = 0;
    quint32 m_sampleTimestamp  = 0;
    bool m_ticksEnabled        = true;
  };
}    // namespace orbit::test
