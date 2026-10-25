#pragma once

#include <QAbstractListModel>
#include <QString>
#include <QVector>

namespace orbit::model
{
  class LogModel : public     QAbstractListModel
  {
    Q_OBJECT
    Q_PROPERTY( int count READ count NOTIFY countChanged )

  public:
    enum class Severity
    {
      Debug,
      Info,
      Warning,
      Error
    };
    Q_ENUM( Severity )

    enum Roles
    {
      TimestampRole = Qt::UserRole + 1,
      SeverityRole,
      SourceRole,
      TextRole
    };
    Q_ENUM( Roles )

    explicit LogModel( QObject *parent = nullptr );

    int count() const
    {
      return m_entries.size();
    }

    int rowCount( const QModelIndex &parent = QModelIndex() ) const override;
    QVariant data( const QModelIndex &index, int role ) const override;
    QHash<int, QByteArray> roleNames() const override;

    void add( Severity severity, const QString &source, const QString &text );
    void addApp( Severity severity, const QString &text );
    void addDevice( const QString &text );

    Q_INVOKABLE void clear();

  signals:
    void countChanged();

  private:
    struct Entry
    {
      QString timestamp;
      Severity severity = Severity::Info;
      QString source;
      QString text;
    };

    static constexpr int MaxEntries = 5000;
    QVector<Entry> m_entries;
  };
}    // namespace orbit::model
