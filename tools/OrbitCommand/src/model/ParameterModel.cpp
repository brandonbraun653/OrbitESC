#include "model/ParameterModel.h"

#include "protocol/ParameterCodec.h"

namespace orbit::model
{
  ParameterModel::ParameterModel( const QVector<ParameterDescriptor> &descriptors, QObject *parent ) :
      QAbstractListModel( parent ), m_descriptors( descriptors )
  {
    m_rows.reserve( descriptors.size() );
    for( const auto &descriptor : descriptors )
    {
      Row row;
      row.descriptor = descriptor;
      switch( descriptor.type )
      {
        case ParamType_BOOL:
          row.value = false;
          break;
        case ParamType_STRING:
          row.value = QString{};
          break;
        case ParamType_BYTES:
          row.value = QByteArray{};
          break;
        case ParamType_FLOAT:
        case ParamType_DOUBLE:
          row.value = 0.0;
          break;
        default:
          row.value = 0;
          break;
      }
      m_rows.append( row );
    }
  }

  int ParameterModel::rowCount( const QModelIndex &parent ) const
  {
    return parent.isValid() ? 0 : m_rows.size();
  }

  QVariant ParameterModel::data( const QModelIndex &index, int role ) const
  {
    if( !index.isValid() || index.row() < 0 || index.row() >= m_rows.size() )
    {
      return {};
    }

    const Row &row = m_rows.at( index.row() );
    switch( role )
    {
      case ParamIdRole:
        return row.descriptor.id;
      case KeyRole:
        return row.descriptor.key;
      case LabelRole:
        return row.descriptor.label;
      case TypeRole:
        return static_cast<int>( row.descriptor.type );
      case ValueRole:
        return row.value;
      case UnitRole:
        return row.descriptor.unit;
      case ReadOnlyRole:
        return row.descriptor.readOnly;
      case DirtyRole:
        return row.dirty;
      case MinRole:
        return row.descriptor.minValue;
      case MaxRole:
        return row.descriptor.maxValue;
      case DecimalsRole:
        return row.descriptor.decimals;
      case MaxLengthRole:
        return row.descriptor.maxStringLength;
      default:
        return {};
    }
  }

  bool ParameterModel::setData( const QModelIndex &index, const QVariant &value, int role )
  {
    if( !index.isValid() || index.row() < 0 || index.row() >= m_rows.size() || role != ValueRole )
    {
      return false;
    }

    return setValueForParamId( m_rows.at( index.row() ).descriptor.id, value, true );
  }

  QHash<int, QByteArray> ParameterModel::roleNames() const
  {
    return {
      { ParamIdRole, "paramId" },
      { KeyRole, "key" },
      { LabelRole, "label" },
      { TypeRole, "type" },
      { ValueRole, "value" },
      { UnitRole, "unit" },
      { ReadOnlyRole, "readOnly" },
      { DirtyRole, "dirty" },
      { MinRole, "minimum" },
      { MaxRole, "maximum" },
      { DecimalsRole, "decimals" },
      { MaxLengthRole, "maxLength" },
    };
  }

  int ParameterModel::rowForParamId( int paramId ) const
  {
    for( int i = 0; i < m_rows.size(); ++i )
    {
      if( m_rows.at( i ).descriptor.id == paramId )
      {
        return i;
      }
    }
    return -1;
  }

  QVariant ParameterModel::valueForParamId( int paramId ) const
  {
    const int row = rowForParamId( paramId );
    return row < 0 ? QVariant{} : m_rows.at( row ).value;
  }

  bool ParameterModel::setValueForParamId( int paramId, const QVariant &value, bool markDirty )
  {
    const int row = rowForParamId( paramId );
    if( row < 0 )
    {
      return false;
    }

    Row &item = m_rows[row];
    if( item.descriptor.readOnly )
    {
      return false;
    }

    QVariant converted;
    switch( item.descriptor.type )
    {
      case ParamType_BOOL:
        converted = value.toBool();
        break;
      case ParamType_FLOAT:
      case ParamType_DOUBLE:
      {
        bool ok = false;
        double numeric = value.toDouble( &ok );
        if( !ok )
        {
          return false;
        }
        numeric   = qBound( item.descriptor.minValue, numeric, item.descriptor.maxValue );
        converted = numeric;
        break;
      }
      case ParamType_STRING:
        converted = value.toString().left( item.descriptor.maxStringLength );
        break;
      case ParamType_BYTES:
        converted = value.toByteArray().left( 64 );
        break;
      default:
      {
        bool ok = false;
        const qlonglong numeric = value.toLongLong( &ok );
        if( !ok )
        {
          return false;
        }
        converted = qBound( static_cast<qlonglong>( item.descriptor.minValue ), numeric,
                            static_cast<qlonglong>( item.descriptor.maxValue ) );
        break;
      }
    }

    if( item.value == converted && item.dirty == markDirty )
    {
      return true;
    }

    item.value = converted;
    item.dirty = markDirty;
    emit dataChanged( index( row ), index( row ), { ValueRole, DirtyRole } );
    return true;
  }

  bool ParameterModel::setDeviceValue( int paramId, const QVariant &value )
  {
    const int row = rowForParamId( paramId );
    if( row < 0 )
    {
      return false;
    }

    Row &item = m_rows[row];
    item.value  = value;
    item.dirty  = false;
    emit dataChanged( index( row ), index( row ), { ValueRole, DirtyRole } );
    return true;
  }

  void ParameterModel::setClean( int paramId )
  {
    const int row = rowForParamId( paramId );
    if( row < 0 || !m_rows.at( row ).dirty )
    {
      return;
    }

    m_rows[row].dirty = false;
    emit dataChanged( index( row ), index( row ), { DirtyRole } );
  }

  void ParameterModel::setAllClean()
  {
    for( int row = 0; row < m_rows.size(); ++row )
    {
      if( m_rows[row].dirty )
      {
        m_rows[row].dirty = false;
        emit dataChanged( index( row ), index( row ), { DirtyRole } );
      }
    }
  }

  QVector<int> ParameterModel::allParamIds() const
  {
    QVector<int> ids;
    ids.reserve( m_rows.size() );
    for( const auto &row : m_rows )
    {
      ids.append( row.descriptor.id );
    }
    return ids;
  }

  QVector<int> ParameterModel::dirtyParamIds() const
  {
    QVector<int> ids;
    for( const auto &row : m_rows )
    {
      if( row.dirty )
      {
        ids.append( row.descriptor.id );
      }
    }
    return ids;
  }

  bool ParameterModel::hasDirty() const
  {
    for( const auto &row : m_rows )
    {
      if( row.dirty )
      {
        return true;
      }
    }
    return false;
  }

  int ParameterModel::dirtyCount() const
  {
    int count = 0;
    for( const auto &row : m_rows )
    {
      if( row.dirty )
      {
        count++;
      }
    }
    return count;
  }
}    // namespace orbit::model
