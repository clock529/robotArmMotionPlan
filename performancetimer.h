#ifndef PERFORMANCETIMER_H
#define PERFORMANCETIMER_H
#include <QObject>
#include <qt_windows.h>
#pragma comment(lib,"winmm.lib")
#include <mmsystem.h>

class PerformanceTimer : public QObject
{
    Q_OBJECT
public:
    explicit PerformanceTimer(QObject *parent = nullptr);
    ~PerformanceTimer();
signals:
    void timeout();
public slots:
    uint start(uint timeInterval);
    void stop();
    friend void  WINAPI CALLBACK PeriodCycle(uint,uint,DWORD_PTR,DWORD_PTR,DWORD_PTR);
    bool IsStop()
    {
        return m_id == 0;
    }
private:
    int m_interval;
    uint m_id;
};
#endif // PERFORMANCETIMER_H
