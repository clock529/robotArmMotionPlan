#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <stdio.h>  //读取配置文件和输出需要
#include <math.h>
#include "performancetimer.h"
#define PI 3.141592654
#include <qfile.h>
#include <iostream>
#include <qstring.h>
#include <qtextstream.h>
#include <QTime>
#include <ControlCAN.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void init();
    void quater2Euler(double q[4], double euler[3]);
    void Euler2quater(double q[4], double euler[3]);
    void quater2Tr(double end[3], double q[4], double TransMtrx[4][4]);
    void Tr2quater(double end[3], double q[4], double TransMtrx[4][4]);
    void euler2Tr(double end[3], double euler[3], double TransMtrx[4][4]);
    void Tr2euler(double end[3], double euler[3], double TransMtrx[4][4]);
    void forkine(double joint[6], double TransMtrx[4][4]);
    int invkine(double expjoint[6], double TransMtrx[4][4]);
    int safecheck(double joint[6]);
    int sgn(double a);
    double max(double a, double b);
    void showResult();
    void runJntPlan();
    void jntplan();
    void cartplan();
    void calcuCycle(double p1[3], double p2[3], double p3[3], double cycleV, double cycleResult[5]);
    void runLinePlan();
    void Lineplan();
    void Cycleplan();
    void planfit(double v1, double v2, double inits, double transtime, double runtime, double curtime, double vs[2]);
    double squareOfVecMag(double vec[3]);
    double multiplyOfVec(double vec1[3], double vec2[3]);
    void sendJnt(int JntIndex, double joint, double jointV);

    void on_initialize_clicked();
    void on_goToZero_clicked();
    void on_clearSysState_clicked();


    void on_theta1p_clicked();
    void on_theta1m_clicked();
    void on_theta2p_clicked();
    void on_theta2m_clicked();
    void on_theta3p_clicked();
    void on_theta3m_clicked();
    void on_theta4p_clicked();
    void on_theta4m_clicked();
    void on_theta5p_clicked();
    void on_theta5m_clicked();
    void on_theta6p_clicked();
    void on_theta6m_clicked();

    void on_cartXp_clicked();
    void on_cartXm_clicked();
    void on_cartYp_clicked();
    void on_cartYm_clicked();
    void on_cartZp_clicked();
    void on_cartZm_clicked();
    void on_cartPhip_clicked();
    void on_cartPhim_clicked();
    void on_cartThetap_clicked();
    void on_cartThetam_clicked();
    void on_cartPsip_clicked();
    void on_cartPsim_clicked();

    void on_record_clicked();
    void on_clear_clicked();
    void on_cartRun_clicked();
    double trajfit(double v1, double v2, double inits, double transitime, double linetime, double cycletime, double cp, double p1, double p2, double theta, double curtime);


    void on_lineRun_clicked();
    void on_cycleRun_clicked();

    void on_connect_clicked();
    void on_close_clicked();

    void on_enable_clicked();
    void on_stop_clicked();

    void sendData(unsigned ID, unsigned char data[8]);
    void recieve();
    void reset(int JntIndex);
    void gotoZero();
    void setRPDO3(int JntIndex);
    void setRPDO1(int JntIndex);
    void setTPDO3(int JntIndex);
    void setTPDO1(int JntIndex);
    void setBasicParams(int JntIndex);
    void guideStart(int JntIndex);
    void start(int JntIndex, unsigned char data[8]);

private:
    Ui::MainWindow *ui;
    /*****DH参数******/
    double d[6] = { 0.0 };    //连杆偏移量，单位：米
    double a[6] = { 0.0 };    // 连杆长度，单位：米
    double alpha[6] = { 0.0 }; //扭转角，单位：弧度
    double InitJnt[6] = { 0.0 }; //初始关节角，单位：弧度

    double MinJnt[6] = { 0.0 };      //关节运动范围下限
    double MaxJnt[6] = { 0.0 };      //关节运动范围上限
    double MaxJntVel[6] = { 0.0 };   //最大关节角速度，单位：rad/s

    /******辅助变量*****/
    double curJnt[6] = { 0.0 };  //当前机械臂关节角，单位：rad
    double curJntV[6] = { 0.0 }; //当前机械臂关节角速度，单位：rad/s
    double curend[3] = { 0.0 };  //当前机械臂末端位置，单位：米
    double curq[4] = { 0.0 };    //当前机械臂末端四元数
    double curEuler[3] = { 0.0 };    //当前机械臂末端欧拉角
    double curtime = 0.0;      //当前时间
    double totaltime = 0.0;    //总运行时间
    QString sysState;          //系统状态
    PerformanceTimer *mtr;
    QTime time;

    /******关节空间规划变量*******/
    double startJnt[6] = { 0.0 };
    double targetJnt[6] = { 0.0 };
    double JntPlanV[6] = { 0.0 }; //单位为rad/s
    double Jntruntime[6] = { 0.0 };   //各关节运行时间
    int jntStartIndex = 0;      //待规划的关节起始序号
    int jntEndIndex = 0;        //待规划的关节结束序号
    QFile JntFile;

    /*******笛卡尔空间规划变量********/
    double startend[3] = { 0.0 };
    double startq[4] = { 0.0 };
    double targetend[3] = { 0.0 };
    double targetq[4] = { 0.0 };
    double cartplanv[3] = { 0.0 };
    int curnum = 0;    //当前规划点数
    double recordtraj[20][9]; //记录的轨迹点信息
    double plantraj[20][22]; //轨迹点参数信息
    int recordnum = 0;    //记录的点数
    QFile CartFile;

    /*******圆弧轨迹规划变量********/
    double targetq1[4] = { 0.0 };
    double targetq2[4] = { 0.0 };
    double alphaq1 = 0.0;
    double alphaq2 = 0.0;
    double curtheta = 0.0;
    double intertheta = 0.0;
    double theta1 = 0.0;
    double theta2 = 0.0;
    double p01[3] = { 0.0 };
    double p03[3] = { 0.0 };
    double p0[3] = { 0.0 };

    /*******CANopen通信********/
    DWORD DeviceType = 4;
    DWORD DeviceInd = 0;
    DWORD CANInd;
    DWORD Reserved = 1;
    VCI_CAN_OBJ vco;
};

#endif // MAINWINDOW_H
