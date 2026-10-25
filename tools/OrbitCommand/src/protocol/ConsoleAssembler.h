#pragma once

#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>
#include <optional>

#include "system_data.pb.h"

namespace orbit::protocol
{
  class ConsoleAssembler
  {
  public:
    std::optional<QString> add( const ConsoleMessage &message );
    QStringList expire( qint64 nowMs );

  private:
    struct Buffer
    {
      QVector<QString> frames;
      QVector<bool> received;
      int receivedCount = 0;
      int totalFrames   = 0;
      qint64 startedAt  = 0;
    };

    QHash<quint16, Buffer> m_buffers;
  };
}    // namespace orbit::protocol
