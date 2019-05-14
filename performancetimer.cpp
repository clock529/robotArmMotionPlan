#include "performancetimer.h"


void  CALLBACK PeriodCycle(uint timerId,uint,DWORD_PTR user,DWORD_PTR,DWORD_PTR)
{
    PerformanceTimer *t=reinterpret_cast<PerformanceTimer *>(user);
    emit t->timeout();
}
PerformanceTimer::PerformanceTimer(QObject *parent) : QObject(parent)
{
    m_id=0;
}
PerformanceTimer::~PerformanceTimer()
{
    stop();
}
uint PerformanceTimer::start(uint timeInterval)
{
    m_id=timeSetEvent(timeInterval,1,PeriodCycle,(DWORD_PTR)this,TIME_CALLBACK_FUNCTION|TIME_PERIODIC|TIME_KILL_SYNCHRONOUS);
    return m_id;
}
void PerformanceTimer::stop()
{
    if(m_id)
       if(timeKillEvent(m_id) == TIMERR_NOERROR)
           m_id = 0;
}
