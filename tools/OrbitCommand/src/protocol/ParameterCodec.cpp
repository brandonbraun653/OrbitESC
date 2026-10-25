#include "protocol/ParameterCodec.h"

#include <QtEndian>
#include <cstring>

namespace orbit::protocol
{
  namespace
  {
    template<typename T>
    QVariant decodeLittleEndian( const QByteArray &data )
    {
      if( data.size() != static_cast<int>( sizeof( T ) ) )
      {
        return {};
      }

      T value{};
      memcpy( &value, data.constData(), sizeof( T ) );
      return QVariant::fromValue( value );
    }

    template<typename T>
    QByteArray encodeLittleEndian( const QVariant &value )
    {
      bool converted = false;
      const qlonglong raw = value.toLongLong( &converted );
      if( !converted )
      {
        return {};
      }

      const T narrowed = static_cast<T>( raw );
      QByteArray output;
      output.resize( static_cast<int>( sizeof( T ) ) );
      memcpy( output.data(), &narrowed, sizeof( T ) );
      return output;
    }
  }    // namespace

  QString parameterTypeName( ParamType type )
  {
    switch( type )
    {
      case ParamType_BOOL:
        return "BOOL";
      case ParamType_UINT8:
        return "UINT8";
      case ParamType_UINT16:
        return "UINT16";
      case ParamType_UINT32:
        return "UINT32";
      case ParamType_FLOAT:
        return "FLOAT";
      case ParamType_DOUBLE:
        return "DOUBLE";
      case ParamType_BYTES:
        return "BYTES";
      case ParamType_STRING:
        return "STRING";
      case ParamType_UNKNOWN:
      default:
        return "UNKNOWN";
    }
  }

  bool isSupportedParameterType( ParamType type )
  {
    switch( type )
    {
      case ParamType_BOOL:
      case ParamType_UINT8:
      case ParamType_UINT16:
      case ParamType_UINT32:
      case ParamType_FLOAT:
      case ParamType_DOUBLE:
      case ParamType_STRING:
      case ParamType_BYTES:
        return true;
      default:
        return false;
    }
  }

  QVariant decodeParameterValue( ParamType type, const QByteArray &data, bool *ok )
  {
    bool success = false;
    QVariant result;

    switch( type )
    {
      case ParamType_BOOL:
        if( data.size() == 1 )
        {
          result  = data.at( 0 ) != 0;
          success = true;
        }
        break;
      case ParamType_UINT8:
        result  = decodeLittleEndian<quint8>( data );
        success = result.isValid();
        break;
      case ParamType_UINT16:
        result  = decodeLittleEndian<quint16>( data );
        success = result.isValid();
        break;
      case ParamType_UINT32:
        result  = decodeLittleEndian<quint32>( data );
        success = result.isValid();
        break;
      case ParamType_FLOAT:
      {
        if( data.size() == 4 )
        {
          quint32 bits = qFromLittleEndian<quint32>( reinterpret_cast<const uchar *>( data.constData() ) );
          float value = 0.0f;
          memcpy( &value, &bits, sizeof( value ) );
          result  = value;
          success = true;
        }
        break;
      }
      case ParamType_DOUBLE:
      {
        if( data.size() == 8 )
        {
          quint64 bits = qFromLittleEndian<quint64>( reinterpret_cast<const uchar *>( data.constData() ) );
          double value = 0.0;
          memcpy( &value, &bits, sizeof( value ) );
          result  = value;
          success = true;
        }
        break;
      }
      case ParamType_STRING:
        result  = QString::fromUtf8( data.constData(), data.size() );
        success = true;
        break;
      case ParamType_BYTES:
        result  = data;
        success = true;
        break;
      default:
        break;
    }

    if( ok )
    {
      *ok = success;
    }
    return success ? result : QVariant{};
  }

  QByteArray encodeParameterValue( ParamType type, const QVariant &value, bool *ok )
  {
    bool success = false;
    QByteArray result;

    switch( type )
    {
      case ParamType_BOOL:
        result.append( static_cast<char>( value.toBool() ? 1 : 0 ) );
        success = true;
        break;
      case ParamType_UINT8:
        result  = encodeLittleEndian<quint8>( value );
        success = !result.isEmpty();
        break;
      case ParamType_UINT16:
        result  = encodeLittleEndian<quint16>( value );
        success = !result.isEmpty();
        break;
      case ParamType_UINT32:
        result  = encodeLittleEndian<quint32>( value );
        success = !result.isEmpty();
        break;
      case ParamType_FLOAT:
      {
        const float float_value = value.toFloat( &success );
        if( success )
        {
          quint32 bits = 0;
          memcpy( &bits, &float_value, sizeof( bits ) );
          result.resize( 4 );
          qToLittleEndian<quint32>( bits, reinterpret_cast<uchar *>( result.data() ) );
        }
        break;
      }
      case ParamType_DOUBLE:
      {
        const double double_value = value.toDouble( &success );
        if( success )
        {
          quint64 bits = 0;
          memcpy( &bits, &double_value, sizeof( bits ) );
          result.resize( 8 );
          qToLittleEndian<quint64>( bits, reinterpret_cast<uchar *>( result.data() ) );
        }
        break;
      }
      case ParamType_STRING:
        result  = value.toString().toUtf8();
        success = true;
        break;
      case ParamType_BYTES:
        result  = value.toByteArray();
        success = true;
        break;
      default:
        break;
    }

    if( ok )
    {
      *ok = success;
    }
    return success ? result : QByteArray{};
  }
}    // namespace orbit::protocol
