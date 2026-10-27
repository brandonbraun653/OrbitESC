#pragma once

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QThread>

#include <functional>

namespace orbit::test
{
  inline bool waitFor( const std::function<bool()> &predicate, int timeoutMs = 5000 )
  {
    QElapsedTimer timer;
    timer.start();
    while( !predicate() )
    {
      if( timer.elapsed() >= timeoutMs )
      {
        return false;
      }

      QCoreApplication::processEvents( QEventLoop::AllEvents, 10 );
      QThread::msleep( 1 );
    }
    return true;
  }

  inline void processEventsFor( int durationMs )
  {
    QElapsedTimer timer;
    timer.start();
    while( timer.elapsed() < durationMs )
    {
      QCoreApplication::processEvents( QEventLoop::AllEvents, 10 );
      QThread::msleep( 1 );
    }
  }
}    // namespace orbit::test
