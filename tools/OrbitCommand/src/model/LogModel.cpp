#include "model/LogModel.h"

#include <QDateTime>

namespace orbit::model
{
  LogModel::LogModel( QObject *parent ) : QAbstractListModel( parent )
  {
  }

  int LogModel::rowCount( const QModelIndex &parent ) const
  {
    return parent.isValid() ? 0 : m_entries.size();
  }

  QVariant LogModel::data( const QModelIndex &index, int role ) const
  {
    if( !index.isValid() || index.row() < 0 || index.row() >= m_entries.size() )
    {
      return {};
    }

    const Entry &entry = m_entries.at( index.row() );
    switch( role )
    {
      case TimestampRole:
        return entry.timestamp;
      case SeverityRole:
        return static_cast<int>( entry.severity );
      case SourceRole:
        return entry.source;
      case TextRole:
        return entry.text;
      default:
        return {};
    }
  }

  QHash<int, QByteArray> LogModel::roleNames() const
  {
    return {
      { TimestampRole, "timestamp" },
      { SeverityRole, "severity" },
      { SourceRole, "source" },
      { TextRole, "text" },
    };
  }

  void LogModel::add( Severity severity, const QString &source, const QString &text )
  {
    const int insertAt = m_entries.size();
    beginInsertRows( QModelIndex(), insertAt, insertAt );
    m_entries.append( Entry{ QDateTime::currentDateTime().toString( "HH:mm:ss.zzz" ), severity, source, text } );
    endInsertRows();

    if( m_entries.size() > MaxEntries )
    {
      const int removeCount = m_entries.size() - MaxEntries;
      beginRemoveRows( QModelIndex(), 0, removeCount - 1 );
      m_entries.remove( 0, removeCount );
      endRemoveRows();
    }

    emit countChanged();
  }

  void LogModel::addApp( Severity severity, const QString &text )
  {
    add( severity, QStringLiteral( "APP" ), text );
  }

  void LogModel::addDevice( const QString &text )
  {
    add( Severity::Info, QStringLiteral( "ESC" ), text );
  }

  void LogModel::clear()
  {
    if( m_entries.isEmpty() )
    {
      return;
    }

    beginResetModel();
    m_entries.clear();
    endResetModel();
    emit countChanged();
  }
}    // namespace orbit::model
