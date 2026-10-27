#pragma once

#include <QByteArray>
#include <QObject>
#include <QString>

#include "device/ConnectionTarget.h"

namespace orbit::device
{
  class Transport : public QObject
  {
    Q_OBJECT
  public:
    explicit Transport( QObject *parent = nullptr ) : QObject( parent )
    {
    }
    ~Transport() override = default;

    virtual bool open( const ConnectionTarget &target ) = 0;
    virtual void close() = 0;
    virtual void write( const QByteArray &data ) = 0;
    virtual bool isOpen() const = 0;

  signals:
    void opened();
    void closed();
    void dataReceived( const QByteArray &data );
    void errorOccurred( const QString &message );
  };
}    // namespace orbit::device
