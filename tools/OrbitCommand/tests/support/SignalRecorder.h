#pragma once

#include <QList>
#include <QMetaObject>
#include <QObject>

#include <tuple>
#include <type_traits>

#include "support/Wait.h"

namespace orbit::test
{
  template<typename... Args>
  class SignalRecorder
  {
  public:
    template<typename Sender, typename Signal>
    SignalRecorder( Sender *sender, Signal signal ) :
        m_connection( QObject::connect( sender, signal, sender,
                                        [this]( Args... arguments )
                                        {
                                          m_records.append( std::tuple<Args...>( arguments... ) );
                                        } ) )
    {
    }

    ~SignalRecorder()
    {
      QObject::disconnect( m_connection );
    }

    SignalRecorder( const SignalRecorder & )            = delete;
    SignalRecorder &operator=( const SignalRecorder & ) = delete;

    qsizetype count() const
    {
      return m_records.size();
    }

    bool empty() const
    {
      return m_records.isEmpty();
    }

    const std::tuple<Args...> &at( qsizetype index ) const
    {
      return m_records.at( index );
    }

    const std::tuple<Args...> &first() const
    {
      return m_records.first();
    }

    const std::tuple<Args...> &last() const
    {
      return m_records.last();
    }

    void clear()
    {
      m_records.clear();
    }

    bool waitForCount( qsizetype minimum, int timeoutMs = 5000 )
    {
      return waitFor( [this, minimum]() { return m_records.size() >= minimum; }, timeoutMs );
    }

  private:
    QMetaObject::Connection m_connection;
    QList<std::tuple<Args...>> m_records;
  };

  template<typename... Args, typename Sender, typename Signal>
  SignalRecorder<Args...> recordSignal( Sender *sender, Signal signal )
  {
    return SignalRecorder<Args...>( sender, signal );
  }
}    // namespace orbit::test
