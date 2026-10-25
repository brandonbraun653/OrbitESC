#pragma once

#include <QAbstractListModel>
#include <QVariant>
#include <QVector>

#include "model/ParameterRegistry.h"

namespace orbit::model
{
  class ParameterModel : public QAbstractListModel
  {
    Q_OBJECT
  public:
    enum Roles
    {
      ParamIdRole = Qt::UserRole + 1,
      KeyRole,
      LabelRole,
      TypeRole,
      ValueRole,
      UnitRole,
      ReadOnlyRole,
      DirtyRole,
      MinRole,
      MaxRole,
      DecimalsRole,
      MaxLengthRole
    };
    Q_ENUM( Roles )

    explicit ParameterModel( const QVector<ParameterDescriptor> &descriptors, QObject *parent = nullptr );

    int rowCount( const QModelIndex &parent = QModelIndex() ) const override;
    Q_INVOKABLE int count() const
    {
      return m_rows.size();
    }
    QVariant data( const QModelIndex &index, int role ) const override;
    bool setData( const QModelIndex &index, const QVariant &value, int role ) override;
    QHash<int, QByteArray> roleNames() const override;

    int rowForParamId( int paramId ) const;
    QVariant valueForParamId( int paramId ) const;
    bool setValueForParamId( int paramId, const QVariant &value, bool markDirty = true );
    bool setDeviceValue( int paramId, const QVariant &value );
    void setClean( int paramId );
    void setAllClean();

    QVector<int> allParamIds() const;
    QVector<int> dirtyParamIds() const;
    bool hasDirty() const;
    int dirtyCount() const;
    const QVector<ParameterDescriptor> &descriptors() const
    {
      return m_descriptors;
    }

  private:
    struct Row
    {
      ParameterDescriptor descriptor;
      QVariant value;
      bool dirty = false;
    };

    QVector<Row> m_rows;
    QVector<ParameterDescriptor> m_descriptors;
  };
}    // namespace orbit::model
