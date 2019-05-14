#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QTime>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    mtr = new PerformanceTimer(this);           //计时器ID
    JntFile.setFileName("C:/Users/qianyikeji/Documents/Robot/robArmMotionPlan/jointspace.txt");
    if (!(JntFile.open( QIODevice::ReadWrite|QIODevice::Text)))
    {
        ui->systermState->append("Error 01: Jointspace.txt open error!");
        return;
    }
    CartFile.setFileName("C:/Users/qianyikeji/Documents/Robot/robArmMotionPlan/cartspace.txt");
    if (!(CartFile.open( QIODevice::ReadWrite|QIODevice::Text)))
    {
        ui->systermState->append("Error 02: Cartspace.txt open error!");
        return;
    }
}

MainWindow::~MainWindow()
{
    delete ui;
    if(nullptr != mtr)
    {
        delete mtr;
        mtr = nullptr;
    }
    if(JntFile.isOpen())
        JntFile.close();
    if(CartFile.isOpen())
        CartFile.close();
}

/********************************
初始化函数，初始化当前关节角、角速度、末端位姿
输入：无
输出：无
*******************************/
void MainWindow::init()
{

    double data[7][6] = {{0.0}};
    double Tr[4][4] = { {0.0} };
    if(mtr != nullptr)
    {
        delete mtr;
        mtr = nullptr;
    }

    //从配置文件读取DH参数值
    QFile fp("C:/Users/qianyikeji/Documents/Robot/robArmMotionPlan/DHparameters.txt");

    if (!fp.open(fp.ReadOnly))
    {
        ui->systermState->append("Error 06: DHparameters.txt open error");
        return;
    }
    else
    {
        QTextStream ts(&fp);
        int i = 0, j = 0;
        while(i<7)
        {
            ts>>data[i][j];
            j++;
            if(j == 6)
            {
                i++;
                j = 0;
            }
        }
        fp.close();
    }

    for (int i = 0; i < 6; i++)
    {
        d[i] = data[0][i];
        a[i] = data[1][i];
        alpha[i] = data[2][i] * PI / 180.0;
        InitJnt[i] = data[3][i] * PI / 180.0;
        MinJnt[i] = data[4][i] * PI / 180.0;
        MaxJnt[i] = data[5][i] * PI / 180.0;
        MaxJntVel[i] = data[6][i] * PI / 180.0;

        curJnt[i] = InitJnt[i];
        curJntV[i] = 0;
    }
    forkine(curJnt, Tr);
    Tr2euler(curend, curEuler, Tr);
    Euler2quater(curq, curEuler);
    showResult();
    ui->systermState->append("Initialization sucess!");
}

/*******************************************************************************
将欧拉角ZYX转换为四元数
欧拉角：
Z: pitch(phi) [-PI, PI], Y: yaw(theta) [-PI/2, PI/2], X: roll(psi) [-PI, PI]
输入:欧拉角
输出:四元数
*******************************************************************************/
void MainWindow::Euler2quater(double q[4], double euler[3])
{
    q[0] = cos(0.5*euler[0])*cos(0.5*euler[1])*cos(0.5*euler[2]) + sin(0.5*euler[0])*sin(0.5*euler[1])*sin(0.5*euler[2]);
    q[1] = cos(0.5*euler[0])*cos(0.5*euler[1])*sin(0.5*euler[2]) - sin(0.5*euler[0])*sin(0.5*euler[1])*cos(0.5*euler[2]);
    q[2] = cos(0.5*euler[0])*sin(0.5*euler[1])*cos(0.5*euler[2]) + sin(0.5*euler[0])*cos(0.5*euler[1])*sin(0.5*euler[2]);
    q[3] = sin(0.5*euler[0])*cos(0.5*euler[1])*cos(0.5*euler[2]) - cos(0.5*euler[0])*sin(0.5*euler[1])*sin(0.5*euler[2]);
}

/*******************************************************************************
将四元数转换为欧拉角ZYX
输入:四元数
输出:欧拉角
注：当euler[1] = ±90°，为奇异姿态，此时令euler[2] = 0;
*******************************************************************************/
void MainWindow::quater2Euler(double q[4], double euler[3])
{
    double temp = q[1]*q[3] - q[0]*q[2];
    if(temp < -0.5 + 1e-6)
    {
        euler[0] = 2 * atan2(-q[1], q[0]);
        euler[1] = 0.5 * PI;
        euler[2] = 0.0;
        return;
    }
    if(temp > 0.5 - 1e-6)
    {
        euler[0] = 2 * atan2(q[1], q[0]);
        euler[1] = -0.5 * PI;
        euler[2] = 0.0;
        return;
    }

    euler[0] = atan2(2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[2]*q[2] - 2*q[3]*q[3]);
    euler[1] = asin(-2*q[1]*q[3] + 2*q[0]*q[2]);
    euler[2] = atan2(2*q[2]*q[3] + 2*q[0]*q[1], 1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
}

/********************************
利用四元数计算转换矩阵
输入：末端位置end[3],末端姿态q[4]
输出：转换矩阵TransMtrx[4][4])
*******************************/
void MainWindow::quater2Tr(double end[3], double q[4], double TransMtrx[4][4])
{
    TransMtrx[0][0] = 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3];
    TransMtrx[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
    TransMtrx[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2];
    TransMtrx[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3];
    TransMtrx[1][1] = 1 - 2 * q[1] * q[1] - 2 * q[3] * q[3];
    TransMtrx[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1];
    TransMtrx[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2];
    TransMtrx[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1];
    TransMtrx[2][2] = 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2];

    TransMtrx[0][3] = end[0];
    TransMtrx[1][3] = end[1];
    TransMtrx[2][3] = end[2];

    TransMtrx[3][0] = 0; TransMtrx[3][1] = 0; TransMtrx[3][2] = 0; TransMtrx[3][3] = 1;

    //消除截断误差
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            if (fabs(TransMtrx[i][j]) < 1e-10)
                TransMtrx[i][j] = 0.0;

}

/********************************
利用转换矩阵计算四元数
输入：转换矩阵TransMtrx[4][4]
输出：末端位置end[3],末端姿态q[4]
注：存在正负两组解，慎用！
*******************************/
void MainWindow::Tr2quater(double end[3], double q[4], double TransMtrx[4][4])
{
    end[0] = TransMtrx[0][3];
    end[1] = TransMtrx[1][3];
    end[2] = TransMtrx[2][3];

//    if(curq[0] > -1e-6)
//    {
//        q[0] = 0.5 * sqrt(max(0, 1 + TransMtrx[0][0] + TransMtrx[1][1] + TransMtrx[2][2]));
//        q[1] = 0.5 * sgn(TransMtrx[2][1] - TransMtrx[1][2]) * sqrt(max(0, 1 + TransMtrx[0][0] - TransMtrx[1][1] - TransMtrx[2][2]));
//        q[2] = 0.5 * sgn(TransMtrx[0][2] - TransMtrx[2][0]) * sqrt(max(0, 1 - TransMtrx[0][0] + TransMtrx[1][1] - TransMtrx[2][2]));
//        q[3] = 0.5 * sgn(TransMtrx[1][0] - TransMtrx[0][1]) * sqrt(max(0, 1 - TransMtrx[0][0] - TransMtrx[1][1] + TransMtrx[2][2]));
//    }
//    else
//    {
//        q[0] = -0.5 * sqrt(max(0, 1 + TransMtrx[0][0] + TransMtrx[1][1] + TransMtrx[2][2]));
//        q[1] = -0.5 * sgn(TransMtrx[2][1] - TransMtrx[1][2]) * sqrt(max(0, 1 + TransMtrx[0][0] - TransMtrx[1][1] - TransMtrx[2][2]));
//        q[2] = -0.5 * sgn(TransMtrx[0][2] - TransMtrx[2][0]) * sqrt(max(0, 1 - TransMtrx[0][0] + TransMtrx[1][1] - TransMtrx[2][2]));
//        q[3] = -0.5 * sgn(TransMtrx[1][0] - TransMtrx[0][1]) * sqrt(max(0, 1 - TransMtrx[0][0] - TransMtrx[1][1] + TransMtrx[2][2]));
//    }
    double trace = TransMtrx[0][0] + TransMtrx[1][1] + TransMtrx[2][2];
    if (trace > 0)
    {
        double s = 0.5/sqrt(trace + 1);
        q[0] = 0.25/s;
        q[1] = (TransMtrx[2][1] - TransMtrx[1][2]) * s;
        q[2] = (TransMtrx[0][2] - TransMtrx[2][0]) * s;
        q[3] = (TransMtrx[1][0] - TransMtrx[0][1]) * s;
    }
    else
    {
        if(TransMtrx[0][0] > TransMtrx[1][1] && TransMtrx[0][0] > TransMtrx[2][2])
        {
            double s = 2 * sqrt(1 + TransMtrx[0][0] - TransMtrx[1][1] - TransMtrx[2][2]);
            q[0] = (TransMtrx[2][1] - TransMtrx[1][2]) / s;
            q[1] = 0.25 * s;
            q[2] = (TransMtrx[0][1] + TransMtrx[1][0]) / s;
            q[3] = (TransMtrx[0][2] + TransMtrx[2][0]) / s;
        }
        else if(TransMtrx[1][1] > TransMtrx[2][2])
        {
            double s = 2 * sqrt(1 + TransMtrx[1][1] - TransMtrx[0][0] - TransMtrx[2][2]);
            q[0] = (TransMtrx[0][2] - TransMtrx[2][0]) / s;
            q[1] = (TransMtrx[0][1] + TransMtrx[1][0]) / s;
            q[2] = 0.25 * s;
            q[3] = (TransMtrx[1][2] + TransMtrx[2][1]) / s;
        }
        else
        {
            double s = 2 * sqrt(1 + TransMtrx[2][2] - TransMtrx[0][0] - TransMtrx[1][1]);
            q[0] = (TransMtrx[1][0] - TransMtrx[0][1]) / s;
            q[1] = (TransMtrx[0][2] + TransMtrx[2][0]) / s;
            q[2] = (TransMtrx[1][2] + TransMtrx[2][1]) / s;
            q[3] = 0.25 * s;
        }
    }
}

/********************************
符号函数
输入：数据a
输出：a的正负
*******************************/
int MainWindow::sgn(double a)
{
    if (a > 1e-6)
        return 1;
    else if (a < -1e-6)
        return -1;
    else
        return 0;
}

/********************************
较大值函数
输入：数据a，b
输出：a和b中的较大值
*******************************/
double MainWindow::max(double a, double b)
{
    if (a >= b)
        return a;
    else
        return b;
}

/********************************
利用欧拉角ZYX计算转换矩阵
输入：末端位置end[3],末端姿态euler[3]
输出：转换矩阵TransMtrx[4][4])
*******************************/
void MainWindow::euler2Tr(double end[3], double euler[3], double TransMtrx[4][4])
{
    TransMtrx[0][0] = cos(euler[0])*cos(euler[1]);
    TransMtrx[1][0] = sin(euler[0])*cos(euler[1]);
    TransMtrx[2][0] = -sin(euler[1]);

    TransMtrx[0][1] = cos(euler[0])*sin(euler[1])*sin(euler[2]) - sin(euler[0])*cos(euler[2]);
    TransMtrx[1][1] = sin(euler[0])*sin(euler[1])*sin(euler[2]) + cos(euler[0])*cos(euler[2]);
    TransMtrx[2][1] = cos(euler[1])*sin(euler[2]);

    TransMtrx[0][2] = cos(euler[0])*sin(euler[1])*cos(euler[2]) + sin(euler[0])*sin(euler[2]);
    TransMtrx[1][2] = sin(euler[0])*sin(euler[1])*cos(euler[2]) - cos(euler[0])*sin(euler[2]);
    TransMtrx[2][2] = cos(euler[1])*cos(euler[2]);

    TransMtrx[0][3] = end[0];
    TransMtrx[1][3] = end[1];
    TransMtrx[2][3] = end[2];

    TransMtrx[3][0] = 0;
    TransMtrx[3][1] = 0;
    TransMtrx[3][2] = 0;
    TransMtrx[3][3] = 1;

    //消除截断误差
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            if (fabs(TransMtrx[i][j]) < 1e-10)
                TransMtrx[i][j] = 0.0;
}

/********************************
利用转换矩阵计算欧拉角ZYX
输入：转换矩阵TransMtrx[4][4]
输出：末端位置end[3],末端姿态euler[3]
*******************************/
void MainWindow::Tr2euler(double end[3], double euler[3], double TransMtrx[4][4])
{
    end[0] = TransMtrx[0][3];
    end[1] = TransMtrx[1][3];
    end[2] = TransMtrx[2][3];
    if(TransMtrx[2][0] < -1 + 1e-6)
    {
        euler[0] = atan2(-TransMtrx[0][1], TransMtrx[1][1]);
        euler[1]  = 0.5 * PI;
        euler[2] = 0.0;
        return;
    }
    if(TransMtrx[2][0] > 1 - 1e-6)
    {
        euler[0] = atan2(-TransMtrx[0][1], TransMtrx[1][1]);
        euler[1] = -0.5 * PI;
        euler[2] = 0.0;
        return;
    }
    euler[0] = atan2(TransMtrx[1][0], TransMtrx[0][0]);
    euler[1] = asin(-TransMtrx[2][0]);
    euler[2] = atan2(TransMtrx[2][1], TransMtrx[2][2]);
}

/********************************
正向运动学函数
输入：关节角度joint[6]
输出：转换矩阵TransMtrx[4][4]
*******************************/
void MainWindow::forkine(double joint[6], double TransMtrx[4][4])
{
    double s1 = sin(joint[0]);
    double c1 = cos(joint[0]);
    double s2 = sin(joint[1]);
    double c2 = cos(joint[1]);
    double s4 = sin(joint[3]);
    double c4 = cos(joint[3]);
    double s5 = sin(joint[4]);
    double c5 = cos(joint[4]);
    double s6 = sin(joint[5]);
    double c6 = cos(joint[5]);
    double s23 = sin(joint[1] + joint[2]);
    double c23 = cos(joint[1] + joint[2]);
    double a1 = a[0], a2 = a[1], a3 = a[2], d1 = d[0], d4 = d[3], d6 = d[5];
    double dx = 0.87, dy = 1.498, dz = 296.979;   //从法兰盘坐标系到工具坐标系的修正

    TransMtrx[0][0] = s6 * (c4*s1 - s4 * c1*c23) - c6 * (s5*c1*s23 - c5 * (s1*s4 + c4 * c1*c23));
    TransMtrx[0][1] = c6 * (c4*s1 - s4 * c1*c23) + s6 * (s5*c1*s23 - c5 * (s1*s4 + c4 * c1*c23));
    TransMtrx[0][2] = -c5 * c1*s23 - s5 * (s1*s4 + c4 * c1*c23);
    TransMtrx[0][3] = dx * TransMtrx[0][0] + dy * TransMtrx[0][1] + dz * TransMtrx[0][2] + a1 * c1 - d6 * (c5*c1*s23 + s5 * (s1*s4 + c4 * c1*c23)) - d4 * c1*s23 + a2 * c1*c2 + a3 * c1*c23;

    TransMtrx[1][0] = -c6 * (c5*(c1*s4 - c4 * s1*c23) + s5 * s1*s23) - s6 * (c1*c4 + s4 * s1*c23);
    TransMtrx[1][1] = s6 * (c5*(c1*s4 - c4 * s1*c23) + s5 * s1*s23) - c6 * (c1*c4 + s4 * s1*c23);
    TransMtrx[1][2] = s5 * (c1*s4 - c4 * s1*c23) - c5 * s1*s23;
    TransMtrx[1][3] = dx * TransMtrx[1][0] + dy * TransMtrx[1][1] + dz * TransMtrx[1][2] + a1 * s1 - d4 * s1*s23 + d6 * (s5*(c1*s4 - c4 * s1*c23) - c5 * s1*s23) + a2 * c2*s1 + a3 * s1*c23;

    TransMtrx[2][0] = s4 * s6*s23 - c6 * (s5*c23 + c4 * c5*s23);
    TransMtrx[2][1] = s6 * (s5*c23 + c4 * c5*s23) + c6 * s4*s23;
    TransMtrx[2][2] = c4 * s5*s23 - c5 * c23;
    TransMtrx[2][3] = dx * TransMtrx[2][0] + dy * TransMtrx[2][1] + dz * TransMtrx[2][2] - d4 * c23 - d6 * (c5*c23 - c4 * s5*s23) - a2 * s2 - a3 * s23 + d1;

    TransMtrx[3][0] = 0.0;
    TransMtrx[3][1] = 0.0;
    TransMtrx[3][2] = 0.0;
    TransMtrx[3][3] = 1.0;

    //消除截断误差
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            if (fabs(TransMtrx[i][j]) < 1e-6)
                TransMtrx[i][j] = 0.0;
}

/********************************
逆运动学函数
输入：转换矩阵TransMtrx[4][4]
输出：逆解出的关节角度double expjoint[6]，逆解成功标志位invkflag(0表示成功，1-6表示关节角超限，7表示逆解无解）
*******************************/
int MainWindow::invkine( double expjoint[6], double TransMtrx[4][4])
{
    int invkflag = 0; //逆解正确标志
    double dx = 0.87, dy = 1.498, dz = 296.979;   //从法兰盘坐标系到工具坐标系的修正

    double nx = TransMtrx[0][0];
    double ox = TransMtrx[0][1];
    double ax = TransMtrx[0][2];
    double px = TransMtrx[0][3] - nx * dx - ox * dy - ax * dz;

    double ny = TransMtrx[1][0];
    double oy = TransMtrx[1][1];
    double ay = TransMtrx[1][2];
    double py = TransMtrx[1][3] - ny * dx - oy * dy - ay * dz;

    double nz = TransMtrx[2][0];
    double oz = TransMtrx[2][1];
    double az = TransMtrx[2][2];
    double pz = TransMtrx[2][3] - d[0] - nz * dx - oz * dy - az * dz;

    double theta1 = 0.0, theta2 = 0.0, theta3 = 0.0, theta4 = 0.0, theta5 = 0.0, theta6 = 0.0;

    //计算theta1
    theta1 = atan2(py - d[5] * ay, px - d[5] * ax);
    if (theta1 < 1e-6 && (fabs(theta1 - curJnt[0]) - fabs(theta1 + PI - curJnt[0]) > -1e-6)  && MinJnt[0] <= theta1 + PI && theta1 + PI <= MaxJnt[0])
        theta1 = theta1 + PI;
    else if (theta1 > -1e-6 && (fabs(theta1 - curJnt[0]) - fabs(theta1 - PI - curJnt[0])>-1e-6) && MinJnt[0] <= theta1 - PI && theta1 - PI <= MaxJnt[0])
        theta1 = theta1 - PI;
    expjoint[0] = theta1;

    //计算theta2
    double k1 = 0;
    double k2 = pz - d[5] * az;
    if (fabs(cos(theta1)) > 0.1)
        k1 = (px - d[5] * ax) / cos(theta1) - a[0];
    else
        k1 = (py - d[5] * ay) / sin(theta1) - a[0];
    double k11 = (d[3] * d[3] + a[2] * a[2] - a[1] * a[1] - k1 * k1 - k2 * k2) / (2 * a[1]);
    if (k1 * k1 + k2 * k2 - k11 * k11 < 1e-6)
        invkflag = 7;
    theta2 = atan2(k11, sqrt(k1*k1 + k2 * k2 - k11 * k11)) - atan2(-k1, k2);
    if (theta2 > 1e-6)
            theta2 = theta2 - 2 * PI;
    double temp2 = atan2(k11, -sqrt(k1*k1 + k2 * k2 - k11 * k11)) - atan2(-k1, k2);
    if (temp2 > 1e-6)
            temp2 = temp2 - 2 * PI;
    if ((theta2 < MinJnt[1] || theta2 > MaxJnt[1] || fabs(theta2 - curJnt[1]) - fabs(temp2 - curJnt[1]) > 1e-6)  && (MinJnt[1] <= temp2) && (temp2 <= MaxJnt[1]))
        theta2 = temp2;
    expjoint[1] = theta2;

    //计算theta3
    double I3 = k1 - a[1] * cos(theta2);
    double I4 = k2 + a[1] * sin(theta2);
    double c23 = (I3 * a[2] - I4 * d[3]) / (d[3]*d[3] + a[2]*a[2]);
    double s23 = (-I4 * a[2] - d[3] * I3) / (d[3]*d[3] + a[2]*a[2]);
    theta3 = atan2(s23, c23) - theta2;
    if (theta3 + PI < 1e-6)
        theta3 = theta3 + 2 * PI;
    if (theta3 - PI > -1e-6)
        theta3 = theta3 - 2 * PI;
    expjoint[2] = theta3;

    //计算theta4、theta5
    double temps4s5 = ay * cos(theta1) - ax*sin(theta1);
    double tempc4s5 = -ax*cos(theta1)*cos(theta2 + theta3) + az * sin(theta2 + theta3) - ay * sin(theta1)*cos(theta2 + theta3);
    if(fabs(temps4s5) < 1e-6 && fabs(tempc4s5) < 1e-6)
    {
        theta4 = curJnt[3];
        theta5 = atan2(0, -az * cos(theta2 + theta3) - ax * cos(theta1)*sin(theta2 + theta3) - ay * sin(theta1)*sin(theta2 + theta3));
    }
    else
    {
        theta4 = atan2(temps4s5, tempc4s5);
        double temp4 = atan2(-temps4s5, -tempc4s5);
        theta5 = atan2(-ay * (sin(theta1)*cos(theta4)*cos(theta2 + theta3) - cos(theta1) * sin(theta4)) + az * cos(theta4)*sin(theta2 + theta3) - ax * (cos(theta1)*cos(theta4)*cos(theta2 + theta3) + sin(theta1) * sin(theta4)), -az * cos(theta2 + theta3) - ax * cos(theta1)*sin(theta2 + theta3) - ay * sin(theta1)*sin(theta2 + theta3));
        double temp5 = atan2(-ay * (sin(theta1)*cos(temp4)*cos(theta2 + theta3) - cos(theta1) * sin(temp4)) + az * cos(temp4)*sin(theta2 + theta3) - ax * (cos(theta1)*cos(temp4)*cos(theta2 + theta3) + sin(theta1) * sin(temp4)), -az * cos(theta2 + theta3) - ax * cos(theta1)*sin(theta2 + theta3) - ay * sin(theta1)*sin(theta2 + theta3));
        if(sin(theta5) < 0 && sin(temp5) < 0)
        {
            theta4 = temp4;
            theta5 = temp5;
        }
        else if(sin(theta5) < 0 && sin(temp5) > 0)
            invkflag = 7;
        else if (sin(theta5) > 0 && sin(temp5) < 0)
            if(MinJnt[3] <= temp4 && temp4 <= MaxJnt[3] && MinJnt[4] <= temp5 && temp5 <= MaxJnt[4])
            {
                if(fabs(theta4 - curJnt[3]) - fabs(temp4 - curJnt[3]) > 0)
                {
                    theta4 = temp4;
                    theta5 = temp5;
                }
                else if(MinJnt[3] > temp4 || temp4 > MaxJnt[3] || MinJnt[4] > temp5 || temp5 > MaxJnt[4])
                {
                    theta4 = temp4;
                    theta5 = temp5;
                }
            }
    }
    expjoint[3] = theta4;
    expjoint[4] = theta5;

    //计算theta6
    double s6 = nz * sin(theta4)*sin(theta2 + theta3) - nx * (cos(theta1)*sin(theta4)*cos(theta2 + theta3) - cos(theta4) * sin(theta1)) - ny * (sin(theta1)*sin(theta4)*cos(theta2 + theta3) + cos(theta1) * cos(theta4));
    double c6 = oz * sin(theta4)*sin(theta2 + theta3) - ox * (cos(theta1)*sin(theta4)*cos(theta2 + theta3) - cos(theta4) * sin(theta1)) - oy * (sin(theta1)*sin(theta4)*cos(theta2 + theta3) + cos(theta1) * cos(theta4));
    theta6 = atan2(s6, c6);
    if (theta6 < 1e-6 && (fabs(theta6 - curJnt[5]) - fabs(theta6 + 2*PI - curJnt[5])>0))
        theta6 = theta6 + 2*PI;
    else if (theta6 > -1e-6 && (fabs(theta6 - curJnt[5]) - fabs(theta6 - 2*PI - curJnt[5])>0))
        theta6 = theta6 - 2*PI;

    expjoint[5] = theta6;

    //检查逆解的关节角是否在运动范围内
    int safeCheckFlag = safecheck(expjoint);
    if(safeCheckFlag != 0)
    {
        return safeCheckFlag;
    }
    else
        return invkflag;
}

/********************************
检查关节角度是否在运动范围内
输入：关节角度joint[6]
输出：错误标志(0表示无错误，1-6表示第几关节角有错)
*******************************/
int MainWindow::safecheck(double joint[6])
{
    for (int i = 0; i < 6; i++)
    {
        if (joint[i] - MinJnt[i] < 0 || joint[i] - MaxJnt[i] > 0)
            return i+1;
    }
    return 0;
}

/********************************
在QT界面展示解算结果
输入：当前关节角curJnt，当前末端位置curend，当前欧拉角curEuler
*******************************/
void MainWindow::showResult()
{
    ui->theta1->setText(QString::number(curJnt[0]*180.0/PI, 10, 2));
    ui->theta2->setText(QString::number(curJnt[1]*180.0/PI+90, 10, 2));
    ui->theta3->setText(QString::number(curJnt[2]*180.0/PI, 10, 2));
    ui->theta4->setText(QString::number(curJnt[3]*180.0/PI, 10, 2));
    ui->theta5->setText(QString::number(curJnt[4]*180.0/PI, 10, 2));
    ui->theta6->setText(QString::number(curJnt[5]*180.0/PI, 10, 2));
    ui->cartX->setText(QString::number(curend[0], 10, 2));
    ui->cartY->setText(QString::number(curend[1], 10, 2));
    ui->cartZ->setText(QString::number(curend[2], 10, 2));
    ui->cartPhi->setText(QString::number(curEuler[0]*180.0/PI, 10, 2));
    ui->cartTheta->setText(QString::number(curEuler[1]*180.0/PI, 10, 2));
    ui->cartPsi->setText(QString::number(curEuler[2]*180.0/PI, 10, 2));
}

/********************************
规划拟合函数
输入：期望速度v,起始位置inits,过渡时间transtime,运行时间runtime,当前时刻curtime,间隔时间intertime
输出：速度和位置矩阵vs[2]
*******************************/
void  MainWindow::planfit(double v1, double v2, double inits, double transtime, double runtime, double curtime, double vs[2])
{
    double a0 = v1;
    double a1 = 3.0 * (v2 - v1) / pow(transtime, 2);
    double a2 = 2.0 * (v1 - v2) / pow(transtime, 3);
    if (curtime - transtime <= 1e-6)
    {
        vs[1] = inits + a0 * curtime + 1.0 / 3 * a1*pow(curtime, 3) + 0.25* a2*pow(curtime, 4);
        vs[0] = a0 + a1 * pow(curtime, 2) + a2 * pow(curtime, 3);
        return;
    }

    inits = inits + 0.5 * (v1 + v2) * transtime;

    if (curtime - transtime > -1e-6 && curtime - runtime + transtime < 1e-6)
    {
        vs[0] = v2;
        vs[1] = inits + v2 * (curtime - transtime);
        return;
    }

    inits = inits + v2 * (runtime - 2*transtime);

    a1 = -3.0 * v2 / pow(transtime, 2);
    a2 = 2.0 * v2 / pow(transtime, 3);
    if (curtime - runtime + transtime >= -1e-6 && curtime - runtime <= 1e-6)
    {
        vs[1] = inits + v2 * (curtime - runtime + transtime) + 1.0 / 3 * a1*pow(curtime - runtime + transtime, 3) + 0.25* a2*pow(curtime - runtime + transtime, 4);
        vs[0] = v2 + a1 * pow(curtime - runtime + transtime, 2) + a2 * pow(curtime - runtime + transtime, 3);
        return;
    }

    inits = inits + 0.5 * v2 * transtime;

    if (curtime - runtime > -1e-6)
    {
        vs[0] = 0;
        vs[1] = inits;
        return;
    }
}

/********************************
关节空间规划函数
在主程序中设置好规划的起始点、终止点和速度
*******************************/
void MainWindow::jntplan()
{
    if(mtr->IsStop())
        return;
    double vs[2] = { 0 };
    double intertime = 0.01;
    double transtime = 0.1;
    double Tr[4][4] = {{0.0}};

    curJnt[1] += PI/2;
    //输出结果
    if(JntFile.isOpen())
    {
        for (int i = 0; i < 6; i++)
        {
            JntFile.write((QString::number(curJnt[i]*180/PI)).toUtf8());
            JntFile.write("  ");
            sendJnt(i + 1, curJnt[i], curJntV[i]);
        }
        JntFile.write("\n");
    }
    curJnt[1] -= PI/2;
    if (curtime - totaltime > -1e-6)
    {
        mtr->stop();
        ui->systermState->append(sysState);
        showResult();
        return;
    }

    curtime = curtime + intertime;

    for (int i = 0; i < 6; i++)
    {
        if (fabs(targetJnt[i] - startJnt[i]) < 1e-6)
            continue;

        //关节空间规划
        planfit(0, JntPlanV[i], startJnt[i], transtime, Jntruntime[i], curtime, vs);
        curJnt[i] = vs[1];
        curJntV[i] = vs[0];
    }

    forkine(curJnt, Tr);
    Tr2euler(curend, curEuler, Tr);
    Euler2quater(curq, curEuler);
}

/********************************
关节空间规划运行函数
设定起始关节角、关节角速度和规划时间，定时器周期调用jntplan函数
*******************************/
void MainWindow::runJntPlan()
{
    uint timerID = 0;
    curtime = 0.0;
    double transtime = 0.1;

    //设定起始关节角、关节角速度和规划时间
    for(int i = 0; i < 6;i++)
    {
        startJnt[i] = curJnt[i];
        if (fabs(targetJnt[i] - startJnt[i]) < 1e-6)
            continue;
        if(targetJnt[i] < startJnt[i])
            JntPlanV[i] = -10.0/180.0*PI;
        else
            JntPlanV[i] = 10.0/180.0*PI;
        Jntruntime[i] = (targetJnt[i] - startJnt[i] - JntPlanV[i]* transtime) / JntPlanV[i] + 2 * transtime;
        if (Jntruntime[i] - totaltime > -1e-6)
            totaltime = Jntruntime[i];
    }

    if(safecheck(targetJnt))
    {
        QString s=QString("Error 02: Target joint%1 out of limitation").arg(safecheck(targetJnt));
        ui->systermState->append(s);
        return;
    }
    else
    {
        if(mtr != nullptr)
        {
            delete mtr;
            mtr = nullptr;
        }
        mtr = new PerformanceTimer();
        connect(mtr,SIGNAL(timeout()),this,SLOT(jntplan()));//定时器时间到，运行超时处理函数jntplan()
        timerID = mtr->start(10);//定时器开始工作
        if (timerID == NULL)
        {
            ui->systermState->append("Error 03: Timer create failure!");
            return;
        }
    }
}

/********************************
笛卡尔空间规划函数
在主程序中设置好规划的起始点、终止点和速度
*******************************/
void MainWindow::Lineplan()
{
    if(mtr->IsStop())
        return;
    double vs[2] = { 0 };
    double intertime = 0.01;
    double transtime = 0.1;
    double Tr[4][4] = {{0.0}};
    double lastJnt[6] = { 0.0 };
    double expJnt[6] = { 0.0 };

    //输出结果
    curJnt[1] += PI/2;
    if(CartFile.isOpen())
    {
        for (int i = 0; i < 6; i++)
        {

            CartFile.write((QString::number(curJnt[i]*180/PI)).toUtf8());
            CartFile.write("  ");
            sendJnt(i + 1, curJnt[i], curJntV[i]);

        }
        /*
        for (int i = 0; i < 3; i++)
        {
            CartFile.write((QString::number(curend)).toUtf8());
            CartFile.write("  ");
        }
        for (int i = 0; i < 4; i++)
        {
            CartFile.write((QString::number(curq).toUtf8());
            CartFile.write("  ");
        }
        */
        CartFile.write("\n");
    }
    curJnt[1] -= PI/2;

    //判断是否结束，并在界面上显示最终结果
    if (curtime - totaltime > -1e-6)
    {
        mtr->stop();
        ui->systermState->append(sysState);
        showResult();
        return;
    }

    curtime = curtime + intertime;

    //末端轨迹规划
    for (int i = 0; i < 3; i++)
    {
        if (fabs(targetend[i] - startend[i]) < 1e-6)
            continue;

        planfit(0, cartplanv[i], startend[i], transtime, totaltime, curtime, vs);
        curend[i] = vs[1];
    }

    if (curtime - totaltime > -1e-6)
        curtime = totaltime;

    //四元数规划（Lerp插值）
    for (int i = 0; i < 4; i++)
        curq[i] = (1 - curtime / totaltime) * startq[i] + curtime / totaltime * targetq[i];


    quater2Tr(curend, curq, Tr);
    Tr2euler(curend, curEuler, Tr);

    //关节角逆解
    int invkFlag = invkine(expJnt, Tr);
    if(invkFlag == 7)
    {

        mtr->stop();
        ui->systermState->append("Error 06: Invkine error!");
        showResult();
        return;
    }
    else if (invkFlag)
    {
        mtr->stop();
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        showResult();
        return;
    }

    for (int i = 0; i < 6; i++)
    {
        lastJnt[i] = curJnt[i];
        curJnt[i] = expJnt[i];
        curJntV[i] = (curJnt[i] - lastJnt[i]) / intertime;
    }
}



void MainWindow::runLinePlan()
{
    curtime = 0.0;
    double transtime = 0.1;
    uint timerID = 0;
    double delta[3] = { 0.0 };

    for (int i = 0; i < 3; i++)
    {
        startend[i] = curend[i];
        delta[i] = targetend[i] - startend[i];
    }

    for (int i = 0; i < 4; i++)
    {
        startq[i] = curq[i];
    }

    double tempDelta = pow(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2], 0.5);
    if(fabs(tempDelta) > 1e-6)
        totaltime = tempDelta / pow(cartplanv[0] * cartplanv[0] + cartplanv[1] * cartplanv[1] + cartplanv[2] * cartplanv[2], 0.5) + transtime;
    else
        totaltime = transtime;

    if(mtr != nullptr)
    {
        delete mtr;
        mtr = nullptr;
    }
    mtr = new PerformanceTimer();
    connect(mtr,SIGNAL(timeout()),this,SLOT(Lineplan()));//定时器时间到，运行超时处理函数jntplan()
    timerID = mtr->start(10);//定时器开始工作
    if (timerID == NULL)
    {
        ui->systermState->append("Error 03: Timer create failure!");
        return;
    }
}

void MainWindow::on_goToZero_clicked()
{
    //设定目标关节角
    targetJnt[0] = InitJnt[0]; targetJnt[1] = InitJnt[1]; targetJnt[2] = InitJnt[2];
    targetJnt[3] = InitJnt[3];	targetJnt[4] = InitJnt[4]; targetJnt[5] = InitJnt[5];
    sysState = "Back to zero sucess!";

    if(JntFile.isOpen())
        JntFile.write("goToZero\n");
    runJntPlan();
}

void MainWindow::on_theta1p_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0] + deltaJnt; targetJnt[1] = curJnt[1]; targetJnt[2] = curJnt[2];
    targetJnt[3] = curJnt[3];	targetJnt[4] = curJnt[4]; targetJnt[5] = curJnt[5];
    sysState = "Theta1 plus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta1p\n");
    runJntPlan();
}

void MainWindow::on_theta1m_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0] - deltaJnt; targetJnt[1] = curJnt[1]; targetJnt[2] = curJnt[2];
    targetJnt[3] = curJnt[3];	targetJnt[4] = curJnt[4]; targetJnt[5] = curJnt[5];
    sysState = "Theta1 minus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta1m\n");
    runJntPlan();
}


void MainWindow::on_theta2p_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0]; targetJnt[1] = curJnt[1] + deltaJnt; targetJnt[2] = curJnt[2];
    targetJnt[3] = curJnt[3];	targetJnt[4] = curJnt[4]; targetJnt[5] = curJnt[5];
    sysState = "Theta2 plus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta2p\n");
    runJntPlan();
}

void MainWindow::on_theta2m_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0]; targetJnt[1] = curJnt[1] - deltaJnt; targetJnt[2] = curJnt[2];
    targetJnt[3] = curJnt[3];	targetJnt[4] = curJnt[4]; targetJnt[5] = curJnt[5];
    sysState = "Theta2 minus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta2m\n");
    runJntPlan();
}

void MainWindow::on_theta3p_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0]; targetJnt[1] = curJnt[1]; targetJnt[2] = curJnt[2] + deltaJnt;
    targetJnt[3] = curJnt[3];	targetJnt[4] = curJnt[4]; targetJnt[5] = curJnt[5];
    sysState = "Theta3 plus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta3p\n");
    runJntPlan();
}

void MainWindow::on_theta3m_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0]; targetJnt[1] = curJnt[1]; targetJnt[2] = curJnt[2] - deltaJnt;
    targetJnt[3] = curJnt[3];	targetJnt[4] = curJnt[4]; targetJnt[5] = curJnt[5];
    sysState = "Theta3 minus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta3m\n");
    runJntPlan();
}

void MainWindow::on_theta4p_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0]; targetJnt[1] = curJnt[1]; targetJnt[2] = curJnt[2];
    targetJnt[3] = curJnt[3] + deltaJnt;	targetJnt[4] = curJnt[4]; targetJnt[5] = curJnt[5];
    sysState = "Theta4 plus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta4p\n");
    runJntPlan();
}

void MainWindow::on_theta4m_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0]; targetJnt[1] = curJnt[1]; targetJnt[2] = curJnt[2];
    targetJnt[3] = curJnt[3] - deltaJnt;	targetJnt[4] = curJnt[4]; targetJnt[5] = curJnt[5];
    sysState = "Theta4 minus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta4m\n");
    runJntPlan();
}

void MainWindow::on_theta5p_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0]; targetJnt[1] = curJnt[1]; targetJnt[2] = curJnt[2];
    targetJnt[3] = curJnt[3];	targetJnt[4] = curJnt[4] + deltaJnt; targetJnt[5] = curJnt[5];
    sysState = "Theta5 plus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta5p\n");
    runJntPlan();
}

void MainWindow::on_theta5m_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0]; targetJnt[1] = curJnt[1]; targetJnt[2] = curJnt[2];
    targetJnt[3] = curJnt[3];	targetJnt[4] = curJnt[4] - deltaJnt; targetJnt[5] = curJnt[5];
    sysState = "Theta5 minus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta5m\n");
    runJntPlan();
}

void MainWindow::on_theta6p_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0]; targetJnt[1] = curJnt[1]; targetJnt[2] = curJnt[2];
    targetJnt[3] = curJnt[3];	targetJnt[4] = curJnt[4]; targetJnt[5] = curJnt[5] + deltaJnt;
    sysState = "Theta6 plus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta6p\n");
    runJntPlan();
}

void MainWindow::on_theta6m_clicked()
{
    double deltaJnt = 2.0*PI/180.0;

    targetJnt[0] = curJnt[0]; targetJnt[1] = curJnt[1]; targetJnt[2] = curJnt[2];
    targetJnt[3] = curJnt[3];	targetJnt[4] = curJnt[4]; targetJnt[5] = curJnt[5] - deltaJnt;
    sysState = "Theta6 minus sucess!";

    if(JntFile.isOpen())
        JntFile.write("theta6m\n");
    runJntPlan();
}

void MainWindow::on_cartXp_clicked()
{
    double deltaEnd = 5.0;
    targetend[0] = curend[0] + deltaEnd; targetend[1] = curend[1]; targetend[2] = curend[2];
    sysState = "X plus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    quater2Tr(targetend, curq, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartXp\n");
    runJntPlan();
}

void MainWindow::on_cartXm_clicked()
{
    double deltaEnd = 5.0;
    targetend[0] = curend[0] - deltaEnd; targetend[1] = curend[1]; targetend[2] = curend[2];
    sysState = "X minus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    quater2Tr(targetend, curq, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartXm\n");
    runJntPlan();
}

void MainWindow::on_cartYp_clicked()
{
    double deltaEnd = 5.0;
    targetend[0] = curend[0]; targetend[1] = curend[1] + deltaEnd; targetend[2] = curend[2];
    sysState = "Y plus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    quater2Tr(targetend, curq, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartYp\n");
    runJntPlan();
}

void MainWindow::on_cartYm_clicked()
{
    double deltaEnd = 5.0;
    targetend[0] = curend[0]; targetend[1] = curend[1] - deltaEnd; targetend[2] = curend[2];
    sysState = "Y minus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    quater2Tr(targetend, curq, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartXm\n");
    runJntPlan();
}

void MainWindow::on_cartZp_clicked()
{
    double deltaEnd = 5.0;
    targetend[0] = curend[0]; targetend[1] = curend[1]; targetend[2] = curend[2] + deltaEnd;
    sysState = "Z plus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    quater2Tr(targetend, curq, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartZp\n");
    runJntPlan();
}

void MainWindow::on_cartZm_clicked()
{
    double deltaEnd = 5.0;
    targetend[0] = curend[0]; targetend[1] = curend[1]; targetend[2] = curend[2] - deltaEnd;
    sysState = "Z minus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    quater2Tr(targetend, curq, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartZm\n");
    runJntPlan();
}

void MainWindow::on_cartPsip_clicked()
{
    double deltaEuler = 2.0*PI/180.0;
    double targetEuler[3] = { 0.0 };

    targetEuler[0] = curEuler[0];	targetEuler[1] = curEuler[1];	targetEuler[2] = curEuler[2] + deltaEuler;
    sysState = "Psi plus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    euler2Tr(curend, targetEuler, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartPsip\n");
    runJntPlan();
}

void MainWindow::on_cartPsim_clicked()
{
    double deltaEuler = 2.0*PI/180.0;
    double targetEuler[3] = { 0.0 };

    targetEuler[0] = curEuler[0];	targetEuler[1] = curEuler[1];	targetEuler[2] = curEuler[2] - deltaEuler;
    sysState = "Psi minus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    euler2Tr(curend, targetEuler, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartPsim\n");
    runJntPlan();
}


void MainWindow::on_cartThetap_clicked()
{
    double deltaEuler = 2.0*PI/180.0;
    double targetEuler[3] = { 0.0 };

    targetEuler[0] = curEuler[0];	targetEuler[1] = curEuler[1] + deltaEuler;	targetEuler[2] = curEuler[2];
    sysState = "Theta plus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    euler2Tr(curend, targetEuler, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartThetap\n");
    runJntPlan();
}

void MainWindow::on_cartThetam_clicked()
{
    double deltaEuler = 2.0*PI/180.0;
    double targetEuler[3] = { 0.0 };

    targetEuler[0] = curEuler[0];	targetEuler[1] = curEuler[1] - deltaEuler;	targetEuler[2] = curEuler[2];
    sysState = "Theta minus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    euler2Tr(curend, targetEuler, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartThetam\n");
    runJntPlan();
}

void MainWindow::on_cartPhip_clicked()
{
    double deltaEuler = 2.0*PI/180.0;
    double targetEuler[3] = { 0.0 };

    targetEuler[0] = curEuler[0] + deltaEuler;	targetEuler[1] = curEuler[1];	targetEuler[2] = curEuler[2];
    sysState = "Phi plus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    euler2Tr(curend, targetEuler, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartPhip\n");
    runJntPlan();
}

void MainWindow::on_cartPhim_clicked()
{
    double deltaEuler = 2.0*PI/180.0;
    double targetEuler[3] = { 0.0 };

    targetEuler[0] = curEuler[0] - deltaEuler;	targetEuler[1] = curEuler[1];	targetEuler[2] = curEuler[2];
    sysState = "Phi minus sucess!";

    double Tr[4][4] = {{ 0.0 }};
    euler2Tr(curend, targetEuler, Tr);
    int invkFlag = invkine(targetJnt, Tr);
    if(invkFlag == 7)
    {
        ui->systermState->append("Error 06: Invkine error!");
        return;
    }
    else if (invkFlag)
    {
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        return;
    }

    if(CartFile.isOpen())
        CartFile.write("cartPhim\n");
    runJntPlan();
}

double MainWindow::trajfit(double v1, double v2, double inits, double transtime, double linetime, double cycletime, double cp, double p1, double p2, double theta, double curtime)
{
    double a0 = v1;
    double a1 = 3.0 * (v2 - v1) / pow(transtime, 2);
    double a2 = 2.0 * (v1 - v2) / pow(transtime, 3);
    double retEnd = 0.0;
    if (curtime - transtime <= 1e-6)
    {
        retEnd = inits + a0 * curtime + 1.0 / 3 * a1*pow(curtime, 3) + 0.25* a2*pow(curtime, 4);
        return retEnd;
    }

    inits = inits + 0.5 * (v1 + v2) * transtime;

    if (curtime - transtime > -1e-6 && curtime - linetime < 1e-6)
    {
        retEnd = inits + v2 * (curtime - transtime);
        return retEnd;
    }

    inits = inits + v2 * (linetime - transtime);
    if (curtime - linetime > -1e-6 && curtime - (linetime + cycletime) < 1e-6)
    {
        double dtheta = (curtime - linetime) / cycletime * theta;
        retEnd = cp + sin(theta - dtheta) / sin(theta) * p1 + sin(dtheta) / sin(theta) * p2;

    }
    if (curtime - (linetime + cycletime) > 1e-6)
    {
        retEnd = p2 + cp;
        return retEnd;
    }
    return retEnd;
}

void MainWindow::cartplan()
{
    if(mtr->IsStop())
    {
        return;
    }

    double intertime = 0.01;
    double lastJnt[6] = {0.0};
    double transtime = 0.1;
    double vs[2] = { 0.0 };
    double Tr[4][4] = {{ 0.0 }};
    double expJnt[6] = { 0.0 };

    curJnt[1] += PI/2;
    if(CartFile.isOpen())
    {
        for (int i = 0; i < 6; i++)
        {
            CartFile.write((QString::number(curJnt[i]*180/PI)).toUtf8());
            CartFile.write("  ");
            sendJnt(i + 1, curJnt[i], curJntV[i]);
        }
        CartFile.write("\n");
    }
    curJnt[1] -= PI/2;
    //qDebug() << time.elapsed();
    /*
    for (int i = 0; i < 3; i++)
    {
        CartFile.write((QString::number(curend)).toUtf8());
        CartFile.write("  ");
    }
    for (int i = 0; i < 4; i++)
    {
        CartFile.write((QString::number(curq).toUtf8());
        CartFile.write("  ");
    }
    */

    if (curtime - totaltime > -1e-6)
    {
        curnum++;
        curtime = 0.0;
    }

    if (curnum - recordnum >= -1e-6)
    {
        mtr->stop();
        ui->systermState->append("Cartesian space plan sucess!");
        showResult();
        return;
    }

    curtime = curtime + intertime;

    if (curtime - intertime < 1e-6)
    {
        for (int i = 0; i < 4; i++)
        {
            startq[i] = curq[i];
            targetq[i] = plantraj[curnum][i + 6];
        }
        alphaq1 = acos(startq[0] * targetq[0] + startq[1] * targetq[1] + startq[2] * targetq[2] + startq[3] * targetq[3]);
        totaltime = plantraj[curnum][16] + plantraj[curnum][21];
        if (curnum < recordnum - 1)
        {
            for (int i = 0; i < 3; i++)
            {
                p01[i] = plantraj[curnum][i + 3] - plantraj[curnum][i + 17];
                p03[i] = plantraj[curnum + 1][i] - plantraj[curnum][i + 17];
            }
        }
    }

    for (int i = 0; i < 3; i++)  //末端位置规划
    {
        if (fabs(plantraj[curnum][i + 3] - plantraj[curnum][i]) <= 1e-6)
            continue;
        if (curnum == recordnum - 1)
        {
            planfit(plantraj[curnum][i + 10], plantraj[curnum][i + 13], plantraj[curnum][i], transtime, plantraj[curnum][16], curtime, vs);
            curend[i] = vs[1];
        }
        else
            curend[i] = trajfit(plantraj[curnum][i + 10], plantraj[curnum][i + 13], plantraj[curnum][i], transtime, plantraj[curnum][16], plantraj[curnum][21], plantraj[curnum][i + 17], p01[i], p03[i], plantraj[curnum][20], curtime);
    }

    for (int i = 0; i < 4; i++)   //四元数规划
    {

        if (curtime - plantraj[curnum][16] <= 1e-6 && fabs(sin(alphaq1)) > 1e-6)
            curq[i] = sin((1 - curtime / plantraj[curnum][16])*alphaq1) / sin(alphaq1)*startq[i] + sin(curtime / plantraj[curnum][16] * alphaq1) / sin(alphaq1)*targetq[i];
        else
            curq[i] = targetq[i];
    }

    quater2Tr(curend, curq, Tr);
    Tr2euler(curend, curEuler, Tr);

    //关节角逆解
    int invkFlag = invkine(expJnt, Tr);
    if(invkFlag == 7)
    {

        mtr->stop();
        ui->systermState->append("Error 06: Invkine error!");
        showResult();
        return;
    }
    else if (invkFlag)
    {
        mtr->stop();
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        showResult();
        return;
    }

    for (int i = 0; i < 6; i++)
    {
        lastJnt[i] = curJnt[i];
        curJnt[i] = expJnt[i];
        curJntV[i] = (curJnt[i] - lastJnt[i]) / intertime;
    }

}

void MainWindow::on_cartRun_clicked()
{
    totaltime = 0.0;
    curtime = 0.0;
    curnum = 0;
    uint timerID = 0;
    double CNT = 0.0;
    double delta[3] = { 0.0 };
    double transtime = 0.1;
    double cycleResult[5] = { 0.0 };
    double p1[3], p2[3], p3[3] = { 0.0 };
    double cycleV = 0.0;
    QTime time;
    time.start();
    qDebug() << "start print  =" << time.elapsed();
    //计算每个目标点的参数：起始位置(0-2)、目标位置(3-5)、目标四元数(6-9)、起始速度(10-12)、目标速度(13-15)、直线时间(16)、圆心坐标(17-19)、圆心角（20）、圆弧时间（21）
    for (int i = 0; i < recordnum; i++)
    {
        CNT = 1 + recordtraj[i][8] * (20 - 1) / 100.0;
        double sum_delta = 0.0;
        for(int j = 0; j < 3; j++)
        {
            if (i == 0)
                delta[j] = recordtraj[i][j] - curend[j];
            else
                delta[j] = recordtraj[i][j] - recordtraj[i - 1][j];
            sum_delta += delta[j] * delta[j];
        }
        for(int j = 0; j < 3; j++)
        {
            if(i == recordnum - 1)
                plantraj[i][j + 3] = recordtraj[i][j];
            else
                plantraj[i][j + 3] = recordtraj[i][j] - CNT * delta[j] / sqrt(sum_delta);  //计算目标位置
            plantraj[i][j + 13] = recordtraj[i][7] * delta[j] / sqrt(sum_delta);    //计算目标速度
            if (i == 0)
            {
                plantraj[i][j] = curend[j];
                plantraj[i][j + 10] = 0.0;
            }
            else
            {
                plantraj[i][j] = recordtraj[i - 1][j] + CNT * delta[j] / sqrt(sum_delta);  //计算初始位置
                plantraj[i][j + 10] = recordtraj[i - 1][7] * delta[j] / sqrt(sum_delta);  //计算初始速度
            }
            if (fabs(plantraj[i][j + 13]) > 1e-6)   //计算直线运动时间
            {
                if (i == recordnum - 1)
                    plantraj[i][16] = (plantraj[i][j + 3] - plantraj[i][j] - 0.5 * (plantraj[i][j + 10] + plantraj[i][j + 13]) * transtime) / plantraj[i][j + 13] + transtime;
                else
                    plantraj[i][16] = (plantraj[i][j + 3] - plantraj[i][j] - 0.5 * (plantraj[i][j + 10] + 2 * plantraj[i][j + 13]) * transtime) / plantraj[i][j + 13] + 2 * transtime;
            }
        }
        if (fabs(plantraj[i][16]) < 1e-6)
            plantraj[i][16] = transtime;
        for (int j = 0; j < 4; j++)           //复制目标四元数
            plantraj[i][j + 6] = recordtraj[i][j + 3];
    }
    for (int i = 0; i < recordnum - 1; i++)  //计算圆弧轨迹参数
    {
        for (int j = 0; j < 3; j++)
        {
            p1[j] = recordtraj[i][j];
            p2[j] = plantraj[i][3 + j];
            p3[j] = plantraj[i + 1][j];
        }
        cycleV = recordtraj[i][7];
        calcuCycle(p1, p2, p3, cycleV, cycleResult);
        plantraj[i][17] = cycleResult[0]; plantraj[i][18] = cycleResult[1]; plantraj[i][19] = cycleResult[2];
        plantraj[i][20] = cycleResult[3];
        plantraj[i][21] = cycleResult[4];
    }
    plantraj[recordnum - 1][17] = 0.0; plantraj[recordnum - 1][18] = 0.0; plantraj[recordnum - 1][19] = 0.0;
    plantraj[recordnum - 1][20] = 0.0; plantraj[recordnum - 1][21] = 0.0;

    qDebug() << time.elapsed();
    if(CartFile.isOpen())
        CartFile.write("Cartesian Plan\n");
    if(mtr != nullptr)
    {
        delete mtr;
        mtr = nullptr;
    }
    mtr = new PerformanceTimer();
    connect(mtr,SIGNAL(timeout()),this,SLOT(cartplan()));//定时器时间到，运行超时处理函数jntplan()

    timerID = mtr->start(10);//定时器开始工作

    if (timerID == NULL)
    {
        ui->systermState->append("Error 03: Timer create failure!");
        return;
    }
}

void MainWindow::calcuCycle(double p1[3], double p2[3], double p3[3], double cycleV, double cycleResult[5])
{
    double p12[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
    double p13[3] = {p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]};
    double p23[3] = {p3[0] - p2[0], p3[1] - p2[1], p3[2] - p2[2]};

    double tempa = 2 * squareOfVecMag(p12) / (4 * squareOfVecMag(p12) - squareOfVecMag(p23));
    for (int i = 0; i < 3; i++)
        cycleResult[i] = p1[i] + tempa * p12[i] + tempa * p13[i];
    cycleResult[3] = PI - acos(multiplyOfVec(p12, p13) / sqrt(squareOfVecMag(p12) * squareOfVecMag(p13)));
    double R = sqrt(pow(cycleResult[0] - p2[0], 2) + pow(cycleResult[1] - p2[1], 2) + pow(cycleResult[2] - p2[2], 2));
    cycleResult[4] = R * cycleResult[3] / cycleV;
}

void MainWindow::on_initialize_clicked()
{
    init();
}

void MainWindow::on_record_clicked()
{
    QString strTemp;
    recordtraj[recordnum][0] = curend[0];
    strTemp = QString::number((recordtraj[recordnum][0]), 10, 2);
    strTemp += "  ";
    recordtraj[recordnum][1] = curend[1];
    strTemp += QString::number((recordtraj[recordnum][1]), 10, 2);
    strTemp += "  ";
    recordtraj[recordnum][2] = curend[2];
    strTemp += QString::number((recordtraj[recordnum][2]), 10, 2);
    strTemp += "  ";
    recordtraj[recordnum][3] = curq[0];
    recordtraj[recordnum][4] = curq[1];
    recordtraj[recordnum][5] = curq[2];
    recordtraj[recordnum][6] = curq[3];
    strTemp += QString::number(curEuler[2]*180/PI, 10, 2);
    strTemp += "  ";
    strTemp += QString::number(curEuler[1]*180/PI, 10, 2);
    strTemp += "  ";
    strTemp += QString::number(curEuler[0]*180/PI, 10, 2);
    strTemp += "  ";
    recordtraj[recordnum][7] = ui->cartV->text().toDouble();
    strTemp += ui->cartV->text();
    strTemp += "  ";
    recordtraj[recordnum][8] = ui->cartCNT->text().toDouble();
    strTemp += ui->cartCNT->text();

    ui->cartDisplay->append(strTemp);

    recordnum += 1;
}

void MainWindow::on_lineRun_clicked()
{
    double targetEuler[3] = { 0.0 };

    targetend[0] = ui->lineX->text().toDouble();
    targetend[1] = ui->lineY->text().toDouble();
    targetend[2] = ui->lineZ->text().toDouble();
    targetEuler[0] = ui->linePhi->text().toDouble()*PI/180.0;
    targetEuler[1] = ui->lineTheta->text().toDouble()*PI/180.0;
    targetEuler[2] = ui->linePsi->text().toDouble()*PI/180.0;

    //避免±PI之间的跳动
    if(fabs(targetEuler[0] + 2 * PI - curEuler[0]) < fabs(targetEuler[0] - curEuler[0]))
        targetEuler[0] = targetEuler[0] + 2 * PI;
    else if(fabs(targetEuler[0] - 2 * PI - curEuler[0]) < fabs(targetEuler[0] - curEuler[0]))
        targetEuler[0] = targetEuler[0] + 2 * PI;

    if(fabs(targetEuler[2] + 2 * PI - curEuler[2]) < fabs(targetEuler[2] - curEuler[2]))
        targetEuler[2] = targetEuler[2] + 2 * PI;
    else if(fabs(targetEuler[2] - 2 * PI - curEuler[2]) < fabs(targetEuler[2] - curEuler[2]))
        targetEuler[2] = targetEuler[2] + 2 * PI;

    Euler2quater(targetq, targetEuler);

    double lineV = ui->lineV->text().toDouble();
    double temp = pow(pow(targetend[0]- curend[0], 2) + pow(targetend[1]- curend[1], 2) + pow(targetend[2]- curend[2], 2), 0.5);
    for (int i = 0; i < 3; i++)
        cartplanv[i] = lineV * (targetend[i]- curend[i]) / temp;

    if(CartFile.isOpen())
        CartFile.write("linePlan\n");

    sysState = "Line plan sucess!";

    runLinePlan();
}

void MainWindow::Cycleplan()
{
    if(mtr->IsStop())
        return;

    double Tr[4][4] = {{0.0}};
    double lastJnt[6] = { 0.0 };
    double expJnt[6] = { 0.0 };
    double intertime = 0.01;

    //输出结果
    curJnt[1] += PI/2;
    if(CartFile.isOpen())
    {
        for (int i = 0; i < 6; i++)
        {
            CartFile.write((QString::number(curJnt[i]*180/PI)).toUtf8());
            CartFile.write("  ");
            sendJnt(i + 1, curJnt[i], curJntV[i]);
        }
        /*
        for (int i = 0; i < 3; i++)
        {
            CartFile.write((QString::number(curend)).toUtf8());
            CartFile.write("  ");
        }
        for (int i = 0; i < 4; i++)
        {
            CartFile.write((QString::number(curq).toUtf8());
            CartFile.write("  ");
        }
        */
        CartFile.write("\n");
    }
    curJnt[1] -= PI/2;

    //判断是否结束，并在界面上显示最终结果
    if (curtheta - (theta1 + theta2) > -1e-6)
    {
        mtr->stop();
        ui->systermState->append("Cycle plan sucess!");
        showResult();
        return;
    }

    curtheta = curtheta + intertheta;

    if (curtheta - (theta1 + theta2) > -1e-6)
        curtheta = theta1 + theta2;

    double c = sin(theta1 + theta2 - curtheta) / sin(theta1 + theta2);
    double d = sin(curtheta) / sin(theta1 + theta2);

    //末端轨迹规划
    for (int i = 0; i < 3; i++)
    {
        curend[i] = p0[i] + c * p01[i] + d * p03[i];
    }

    //四元数规划
    if(curtheta <= theta1)
    {
        if(fabs(sin(alphaq1)) > 1e-6)
            for (int i = 0; i < 4; i++)
                curq[i] = sin((1 - curtheta / theta1)*alphaq1) / sin(alphaq1)*startq[i] + sin(curtheta / theta1 * alphaq1) / sin(alphaq1)*targetq1[i];
    }
    else
    {
        if(fabs(sin(alphaq2)) > 1e-6)
            for (int i = 0; i < 4; i++)
                curq[i] = sin((1 - (curtheta - theta1) / theta2)*alphaq2) / sin(alphaq2)*targetq1[i] + sin((curtheta - theta1) / theta2 * alphaq2) / sin(alphaq2)*targetq2[i];
    }

    quater2Tr(curend, curq, Tr);
    Tr2euler(curend, curEuler, Tr);

    //关节角逆解
    int invkFlag = invkine(expJnt, Tr);
    if(invkFlag == 7)
    {

        mtr->stop();
        ui->systermState->append("Error 06: Invkine error!");
        showResult();
        return;
    }
    else if (invkFlag)
    {
        mtr->stop();
        QString s=QString("Error 05: Solved joint%1 out of limitation").arg(invkFlag);
        ui->systermState->append(s);
        showResult();
        return;
    }

    for (int i = 0; i < 6; i++)
    {
        lastJnt[i] = curJnt[i];
        curJnt[i] = expJnt[i];
        curJntV[i] = (curJnt[i] - lastJnt[i]) / intertime;
    }
}

void MainWindow::on_cycleRun_clicked()
{
    uint timerID = 0;
    double euler1[3] = {0.0}, euler2[3] = {0.0};
    double x1 = curend[0], y1 = curend[1], z1 = curend[2];

    //读取参数
    double x2 = ui->cycleX1->text().toDouble();
    double y2 = ui->cycleY1->text().toDouble();
    double z2 = ui->cycleZ1->text().toDouble();
    euler1[0] = ui->cyclePhi1->text().toDouble()*PI/180.0;
    euler1[1] = ui->cycleTheta1->text().toDouble()*PI/180.0;
    euler1[2] = ui->cyclePsi1->text().toDouble()*PI/180.0;

    double x3 = ui->cycleX2->text().toDouble();
    double y3 = ui->cycleY2->text().toDouble();
    double z3 = ui->cycleZ2->text().toDouble();
    euler2[0] = ui->cyclePhi2->text().toDouble()*PI/180.0;
    euler2[1] = ui->cycleTheta2->text().toDouble()*PI/180.0;
    euler2[2] = ui->cyclePsi2->text().toDouble()*PI/180.0;

    //避免±PI之间的跳动
    if(fabs(euler1[0] + 2 * PI - curEuler[0]) < fabs(euler1[0] - curEuler[0]))
        euler1[0] = euler1[0] + 2 * PI;
    else if(fabs(euler1[0] - 2 * PI - curEuler[0]) < fabs(euler1[0] - curEuler[0]))
        euler1[0] = euler1[0] + 2 * PI;

    if(fabs(euler1[2] + 2 * PI - curEuler[2]) < fabs(euler1[2] - curEuler[2]))
        euler1[2] = euler1[2] + 2 * PI;
    else if(fabs(euler1[2] - 2 * PI - curEuler[2]) < fabs(euler1[2] - curEuler[2]))
        euler1[2] = euler1[2] + 2 * PI;

    //避免±PI之间的跳动
    if(fabs(euler2[0] + 2 * PI - curEuler[0]) < fabs(euler2[0] - curEuler[0]))
        euler2[0] = euler2[0] + 2 * PI;
    else if(fabs(euler2[0] - 2 * PI - curEuler[0]) < fabs(euler2[0] - curEuler[0]))
        euler2[0] = euler2[0] + 2 * PI;

    if(fabs(euler2[2] + 2 * PI - curEuler[2]) < fabs(euler2[2] - curEuler[2]))
        euler2[2] = euler2[2] + 2 * PI;
    else if(fabs(euler2[2] - 2 * PI - curEuler[2]) < fabs(euler2[2] - curEuler[2]))
        euler2[2] = euler2[2] + 2 * PI;

    //计算目标四元数以及夹角
    for (int i = 0; i < 4; i++)
        startq[i] = curq[i];
    Euler2quater(targetq1, euler1);
    Euler2quater(targetq2, euler2);
    alphaq1 = acos(startq[0] * targetq1[0] + startq[1] * targetq1[1] + startq[2] * targetq1[2] + startq[3] * targetq1[3]);
    alphaq2 = acos(targetq1[0] * targetq2[0] + targetq1[1] * targetq2[1] + targetq1[2] * targetq2[2] + targetq1[3] * targetq2[3]);

    //判断是否三点共线
    double p12[3] = {x2 - x1, y2 - y1, z2 - z1};
    double p13[3] = {x3 - x1, y3 - y1, z3 - z1};
    if (fabs((x2 - x1) *  (y3 - y2) -  (x3 - x2) * (y2 - y1)) < 1e-6 &&
            fabs((x2 - x1) * (z3 - z2)  -  (x3 - x2) * (z2 - z1)) < 1e-6)
    {
        ui->systermState->append("Error 07: Three points are colliner!!");
        return;
    }

    //计算圆心坐标和半径
    double tempab = squareOfVecMag(p12) * squareOfVecMag(p13) - pow(multiplyOfVec(p12, p13), 2);
    double a = 0.5 * squareOfVecMag(p13) * (squareOfVecMag(p12) - multiplyOfVec(p12, p13)) / tempab;
    double b = 0.5 * (squareOfVecMag(p12) * multiplyOfVec(p12, p13) - squareOfVecMag(p12) * squareOfVecMag(p13))/ (-tempab);
    double x0 = x1 + a * (x2 - x1) + b * (x3 - x1);
    double y0 = y1 + a * (y2 - y1) + b * (y3 - y1);
    double z0 = z1 + a * (z2 - z1) + b * (z3 - z1);
    p01[0] = x1 - x0; p01[1] = y1 - y0; p01[2] = z1 - z0;
    p03[0] = x3 - x0; p03[1] = y3 - y0; p03[2] = z3 - z0;
    p0[0] = x0; p0[1] = y0; p0[2] = z0;
    double p02[3] = {x2 - x0, y2 - y0, z2 - z0};

    double R = (pow(squareOfVecMag(p01), 0.5) + pow(squareOfVecMag(p02), 0.5) + pow(squareOfVecMag(p03), 0.5))/ 3.0;

    //计算圆心角
    theta1 = acos(multiplyOfVec(p01, p02) / (R * R));
    theta2 = acos(multiplyOfVec(p02, p03) / (R * R));
    intertheta = 1e-2 * ui -> cycleV -> text().toDouble() / R;

    curtheta = 0.0;

    if(CartFile.isOpen())
        CartFile.write("cyclePlan\n");
    if(mtr != nullptr)
    {
        delete mtr;
        mtr = nullptr;
    }
    mtr = new PerformanceTimer();
    connect(mtr,SIGNAL(timeout()),this,SLOT(Cycleplan()));//定时器时间到，运行超时处理函数jntplan()
    timerID = mtr->start(10);//定时器开始工作
    if (timerID == NULL)
    {
        ui->systermState->append("Error 03: Timer create failure!");
        return;
    }
}

double MainWindow::squareOfVecMag(double vec[3])
{
    return vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
}

double MainWindow::multiplyOfVec(double vec1[3], double vec2[3])
{
    return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
}
void MainWindow::on_clear_clicked()
{
    recordnum = 0;
    ui->cartDisplay->setText("");
    memset(recordtraj, 0, sizeof(recordtraj));
}

void MainWindow::on_clearSysState_clicked()
{
    ui->systermState->setText("");
}

void MainWindow::on_connect_clicked()
{
    vco.SendType=0;
    vco.RemoteFlag=0;
    vco.ExternFlag=0;
    vco.DataLen=8;
    VCI_INIT_CONFIG vic;
    int baundRate = (int)ui->baundRate->currentText().remove("Kbps").toFloat();
    switch (baundRate)
    {
    case 500:
        vic.Timing0=0x00;
        vic.Timing1=0x1c;
        break;
    case 1000:
        vic.Timing0=0x00;
        vic.Timing1=0x14;
        break;
    default:
        break;
    }
    vic.AccCode=0;
    vic.AccMask=0xffffffff;
    vic.Filter=0;
    vic.Mode=0;
    CANInd = ui->CANCom->currentIndex();

    DWORD dwRel;
    dwRel = VCI_OpenDevice(DeviceType, DeviceInd, Reserved);
    qDebug() << "open" << dwRel;
    if(dwRel == 1)
    {
        dwRel = VCI_ClearBuffer(DeviceType, DeviceInd, CANInd);
        qDebug() << "clear" << dwRel;
        if(dwRel == 1)
        {
            dwRel = VCI_InitCAN(DeviceType, DeviceInd, CANInd, &vic);
            qDebug() << "init" << dwRel;
            if(dwRel == 1)
            {
                dwRel = VCI_StartCAN(DeviceType, DeviceInd, CANInd);
                qDebug() << "start" << dwRel;
                if(dwRel == 1)
                {
                    ui->systermState->append("CAN connect sucess!");
                    return;
                }
            }
        }
    }
    ui->systermState->append("CAN connect failure!");
}

void MainWindow::on_close_clicked()
{
    DWORD dwRel;
    dwRel = VCI_CloseDevice(DeviceType, DeviceInd);
    qDebug() << "close" << dwRel;
    if (dwRel == 1)
        ui->systermState->append("CAN close sucess!");
    else {
        ui->systermState->append("CAN close failure!");
    }
}

void MainWindow::sendData(UINT ID, unsigned char data[8])
{
    DWORD dwRel;
    vco.ID = ID;
    for (int i=0; i < 8; i++)
    {
        vco.Data[i]=data[i];
    }
    dwRel = VCI_Transmit(DeviceType, DeviceInd, CANInd, &vco, 1);
    qDebug()<<"send: " << dwRel;
//    Sleep(10);
//    recieve();
}

void MainWindow::recieve()
{
    VCI_CAN_OBJ receiveData;

    //接收
    ULONG res = 0;

    //获取缓冲区中接收但尚未被读取的帧数
    res = VCI_Receive(DeviceType, DeviceInd, 0, &receiveData,1,10);
    if (res <= 0)
    {
        VCI_ERR_INFO vei;
        VCI_ReadErrInfo(DeviceType,DeviceInd,CANInd,&vei);
        qDebug()<<"Read Data failed"<<"Error Data:"<<QString::number(vei.ErrCode,16);
        return;
    }
    else
    {
        qDebug()<<"receive: " << res;
        qDebug()<<"帧ID: "<<QString::number(receiveData.ID,16);
        qDebug()<<"帧数据: "<<QString::number(receiveData.Data[0],16)<<QString::number(receiveData.Data[1],16);
        qDebug()<<QString::number(receiveData.Data[2],16)<<QString::number(receiveData.Data[3],16);
        qDebug()<<QString::number(receiveData.Data[4],16)<<QString::number(receiveData.Data[5],16);
        qDebug()<<QString::number(receiveData.Data[6],16)<<QString::number(receiveData.Data[7],16);
        qDebug()<<"帧长度: "<<receiveData.DataLen;
        recieve();
    }
}

void MainWindow::on_enable_clicked()
{
    for (int i = 1; i < 7; i ++)
    {
        reset(i);
        //gotoZero();
        setRPDO3(i);
        setRPDO1(i);
        //setTPDO3(i);
        //setTPDO1(i);
        setBasicParams(i);
        guideStart(i);
    }
    ui->systermState->append("Robot enable sucess!");
}

void MainWindow::sendJnt(int JntIndex, double joint, double jointV)
{
    unsigned char data[8] = {0x00};
    INT64 tempJnt = int(joint*425705.59562*10);
    INT64 tempJntV = int(jointV*3948.16562*10);

    //为负数则计算补码，加上0xffffffff + 1
    if (tempJnt < 0)
        tempJnt += 4294967296;
    if (tempJntV < 0)
        tempJntV += 4294967296;
    for (int i = 0; i < 4; i++)
    {
        data[i] = tempJnt % 256;
        data[i + 4] = tempJntV % 256;
        tempJnt /= 256;
        tempJntV /= 256;
        if (tempJnt == 0 && tempJntV == 0)
            break;
    }
    start(JntIndex, data);
}

//第二步复位节点：
//通过NMT复位节点，数据格式为[0 : 81 06 00 00 00 00 00 00];
//通过NMT启动节点，数据格式为[0 : 01 06 00 00 00 00 00 00];
void MainWindow::reset(int JntIndex)
{
    unsigned int id = 0x000;
    unsigned char data[8] = {0x81, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    data[1] = JntIndex;
    sendData(id, data);
    Sleep(1000);
    data[0] = 0x01;
    sendData(id, data);
    Sleep(10);
    qDebug()<<"复位";
}

//第三步回零（用于归零电机的位置数据）：
//设置控制模式（6060-00h）为回零模式，数据格式为[606 : 2f 60 60 00 06 00 00 00];
//设置回零方式（6098-00h）为0x23，数据格式为[606 : 2f 98 60 00 23 00 00 00];
//设置控制字（6040-00h）伺服准备好，数据格式为[606 : 2b 40 60 00 06 00 00 00];
//设置控制字（6040-00h）等待打开伺服使能，数据格式为[606 : 2b 40 60 00 07 00 00 00];
//设置控制字（6040-00h）伺服运行，数据格式为[606 : 2b 40 60 00 0f 00 00 00];
//设置控制字（6040-00h）更新指令，数据格式为[606 : 2b 40 60 00 1f 00 00 00];
//设置控制字（6040-00h）伺服准备好，数据格式为[606 : 2b 40 60 00 06 00 00 00];
void MainWindow::gotoZero()
{
    unsigned int id = 0x606;
    unsigned char data[8] = {0x2f, 0x60, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    sendData(id, data);
    data[1] = 0x98; data[4] = 0x23;
    sendData(id, data);
    data[0] = 0x2b; data[1] = 0x40; data[4] = 0x06;
    sendData(id, data);
    data[4] = 0x07;
    sendData(id, data);
    data[4] = 0x0f;
    sendData(id, data);
    data[4] = 0x1f;
    sendData(id, data);
    data[4] = 0x06;
    sendData(id, data);
    qDebug()<<"回零";
}

//将RPDO3接收的数据配置为目标位置和轮廓速度：
//关闭RPDO3（1402-01h），数据格式为[606 : 23 02 14 01 06 04 00 80];
//清除RPDO3（1602-00h）的映射内容，数据格式为[606 : 2f 02 16 00 00 00 00 00];
//将目标位置参数（607A-00h）映射到RPDO3的第1个子索引上（1602-01h），数据格式为[606 : 23 02 16 01 20 00 7a 60];
//将轮廓速度参数（6081-00h）映射到RPDO3的第2个子索引上（1602-02h），数据格式为[606 : 23 02 16 02 20 00 81 60];
//使能RPDO3（1602-00h）的映射，数据格式为[606 : 2f 02 16 00 02 00 00 00];
//打开RPDO3（1402-01h），数据格式为[606 : 23 02 14 01 06 04 00 00];
void MainWindow::setRPDO3(int JntIndex)
{
    unsigned int id = 0x600 + JntIndex;
    unsigned char data[8] = {0x23, 0x02, 0x14, 0x01, 0x06, 0x04, 0x00, 0x80};
    data[4] = JntIndex;
    sendData(id, data);
    Sleep(10);
    data[0] = 0x2f; data[2] = 0x16; data[3] = 0x00; data[4] = 0x00; data[5] = 0x00; data[7] = 0x00;
    sendData(id, data);
    Sleep(10);
    data[0] = 0x23; data[3] = 0x01; data[4] = 0x20; data[6] = 0x7a; data[7] = 0x60;
    sendData(id, data);
    Sleep(10);
    data[3] = 0x02; data[6] = 0x81;
    sendData(id, data);
    Sleep(10);
    data[0] = 0x2f; data[3] = 0x00; data[4] = 0x02; data[6] = 0x00; data[7] = 0x00;
    sendData(id, data);
    Sleep(10);
    data[0] = 0x23; data[2] = 0x14; data[3] = 0x01; data[4] = JntIndex; data[5] = 0x04;
    sendData(id, data);
    Sleep(10);
    qDebug()<<"RPDO3配置";
}

//将RPDO1接收的数据配置为控制字：
//关闭RPDO1（1400-01h），数据格式为[606 : 23 00 14 01 06 02 00 80];
//清除RPDO1（1600-00h）的映射内容，数据格式为[606 : 2f 00 16 00 00 00 00 00];
//将控制字参数（6040-00h）映射到RPDO1的第1个子索引上（1600-01h），数据格式为[606 : 23 00 16 01 10 00 40 60];
//使能RPDO1（1600-00h）的映射，数据格式为[606 : 2f 00 16 00 01 00 00 00];
//打开RPDO1（1400-01h），数据格式为[606 : 23 00 14 01 06 02 00 00];
void MainWindow::setRPDO1(int JntIndex)
{
    unsigned int id = 0x600 + JntIndex;
    unsigned char data[8] = {0x23, 0x00, 0x14, 0x01, 0x01, 0x02, 0x00, 0x80};
    data[4] = JntIndex;
    sendData(id, data);
    Sleep(10);
    data[0] = 0x2f; data[2] = 0x16; data[3] = 0x00; data[4] = 0x00; data[5] = 0x00; data[7] = 0x00;
    sendData(id, data);
    Sleep(10);
    data[0] = 0x23; data[3] = 0x01; data[4] = 0x10; data[6] = 0x40; data[7] = 0x60;
    sendData(id, data);
    Sleep(10);
    data[0] = 0x2f; data[3] = 0x00; data[4] = 0x01; data[6] = 0x00; data[7] = 0x00;
    sendData(id, data);
    Sleep(10);
    data[0] = 0x23; data[2] = 0x14; data[3] = 0x01; data[4] = JntIndex; data[5] = 0x02;
    sendData(id, data);
    Sleep(10);
    qDebug()<<"RPDO1配置";
}

//将TPDO3发送的数据配置为位置反馈和速度反馈：
//关闭TPDO3（1802-01h），数据格式为[606 : 23 02 18 01 86 03 00 c0];
//清除TPDO3（1a02-00h）的映射内容，数据格式为[606 : 2f 02 1a 00 00 00 00 00];
//将位置反馈参数（6064-00h）映射到TPDO3的第1个子索引上（1a02-01h），数据格式为[606 : 23 02 1a 01 20 00 64 60];
//将轮廓速度参数（606c-00h）映射到TPDO3的第2个子索引上（1a02-02h），数据格式为[606 : 23 02 1a 02 20 00 6c 60];
//使能TPDO3（1a02-00h）的映射，数据格式为[606 : 2f 02 1a 00 02 00 00 00];
//打开TPDO3（1802-01h），数据格式为[606 : 23 02 18 01 86 03 00 40];
void MainWindow::setTPDO3(int JntIndex)
{
    unsigned int id = 0x600 + JntIndex;
    unsigned char data[8] = {0x23, 0x02, 0x18, 0x01, 0x81, 0x03, 0x00, 0xc0};
    data[4] = 0x80 + JntIndex;
    sendData(id, data);
    data[0] = 0x2f; data[2] = 0x1a; data[3] = 0x00; data[4] = 0x00; data[5] = 0x00; data[7] = 0x00;
    sendData(id, data);
    data[0] = 0x23; data[3] = 0x01; data[4] = 0x20; data[6] = 0x64; data[7] = 0x60;
    sendData(id, data);
    data[3] = 0x02; data[6] = 0x6c;
    sendData(id, data);
    data[0] = 0x2f; data[3] = 0x00; data[4] = 0x02; data[6] = 0x00; data[7] = 0x00;
    sendData(id, data);
    data[0] = 0x23; data[2] = 0x18; data[3] = 0x01; data[4] = 0x80 + JntIndex; data[5] = 0x03; data[7] = 0x40;
    sendData(id, data);
    qDebug()<<"TPDO3配置";
}

//将TPDO1发送的数据配置为状态字：
//关闭TPDO1（1800-01h），数据格式为[606 : 23 00 18 01 86 01 00 c0];
//清除TPDO1（1a00-00h）的映射内容，数据格式为[606 : 2f 00 1a 00 00 00 00 00];
//将状态字（6061-00h）映射到TPDO1的第1个子索引上（1a00-01h），数据格式为[606 : 23 00 1a 01 10 00 61 60];
//使能TPDO1（1a00-00h）的映射，数据格式为[606 : 2f 00 1a 00 01 00 00 00];
//打开TPDO1（1800-01h），数据格式为[606 : 23 00 18 01 86 01 00 40];
void MainWindow::setTPDO1(int JntIndex)
{
    unsigned int id = 0x600 + JntIndex;
    unsigned char data[8] = {0x23, 0x00, 0x18, 0x01, 0x81, 0x01, 0x00, 0xc0};
    data[4] = 0x80 + JntIndex;
    sendData(id, data);
    data[0] = 0x2f; data[2] = 0x1a; data[3] = 0x00; data[4] = 0x00; data[5] = 0x00; data[7] = 0x00;
    sendData(id, data);
    data[0] = 0x23; data[3] = 0x01; data[4] = 0x10; data[6] = 0x61; data[7] = 0x60;
    sendData(id, data);
    data[0] = 0x2f; data[3] = 0x00; data[4] = 0x01; data[6] = 0x00; data[7] = 0x00;
    sendData(id, data);
    data[0] = 0x23; data[2] = 0x18; data[3] = 0x01; data[4] = 0x80 + JntIndex; data[5] = 0x01; data[7] = 0x40;
    sendData(id, data);
    qDebug()<<"TPDO1配置";
}

//第五步基本参数配置：
//1）	设置转换因子（包括位置因子、转速编码器因子、转速因子1、加速度因子以及指令极性，此处只修改位置因子，其他因子采用默认值）：
//设置位置因子的分子部分（6093-01h）为0xa0，数据格式为[606 : 23 93 60 01 f0 00 00 00];
//设置位置因子的分母部分（6093-02h）0x01，数据格式为[606 : 23 93 60 02 01 00 00 00];
//其他转换因子的设置跟位置因子类似；
//2）	配置运行模式：
//设置控制模式（6060-00h）为轮廓位置模式，数据格式为[606 : 2f 60 60 00 01 00 00 00];
void MainWindow::setBasicParams(int JntIndex)
{
    unsigned int id = 0x600 + JntIndex;
    unsigned char data[8] = {0x23, 0x93, 0x60, 0x01, 0x40, 0x03, 0x00, 0x00};
    sendData(id, data);
    Sleep(10);
    data[3] = 0x02; data[4] = 0x01;
    sendData(id, data);
    Sleep(10);
    data[0] = 0x2f; data[1] = 0x60; data[3] = 0x00;
    sendData(id, data);
    Sleep(10);
    qDebug()<<"基本参数配置";
}

//启动引导（因为之前已经做过PDO配置，所以此处直接使用PDO发送数据）：
//设置控制字（6040-00h）伺服准备好，数据格式为[206 : 06 00 00 00 00 00 00 00];
//设置控制字（6040-00h）等待打开伺服使能，数据格式为[206 : 07 00 00 00 00 00 00 00];
void MainWindow::guideStart(int JntIndex)
{
    unsigned int id = 0x200 + JntIndex;
    unsigned char data[8] = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendData(id, data);
    Sleep(10);
    data[0] = 0x07;
    sendData(id, data);
    Sleep(10);
    qDebug()<<"引导启动";
}

//启动运行（绝对位置立即更新）
//设置目标位置（607a-00h）和轮廓速度（6081-00h），数据格式为[406 : 00 00 10 00 e8 03 00 00]（其中前四个字节表示目标位置，后四个字节表示轮廓速度）；
//设置控制字（6040-00h）为伺服运行，数据格式为[206 : 2f 00 00 00 00 00 00 00];
//设置控制字（6040-00h）为指令更新，数据格式为[206 : 3f 00 00 00 00 00 00 00];
void MainWindow::start(int JntIndex, unsigned char data[8])
{
    unsigned int id = 0x400 + JntIndex;
    sendData(id, data);
    id = 0x200 + JntIndex;
    unsigned char data2[8] = {0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendData(id, data2);
    data2[0] = 0x3f;
    sendData(id, data2);
    qDebug()<<"启动";
}

//停止运行：
//设置控制字（6040-00h）伺服准备好，数据格式为[206 : 07 00 00 00 00 00 00 00]即可立即停止运行。
void MainWindow::on_stop_clicked()
{
    unsigned int id;
    unsigned char data[8] = {0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for(int i = 1; i < 7; i++)
    {
        id = 0x200 + i;
        sendData(id, data);
        qDebug()<<"停止: "<<i;
    }
    ui->systermState->append("Robot stop sucess!");

    //关闭计时器
    if(nullptr != mtr)
    {
        mtr->stop();
        delete mtr;
        mtr = nullptr;
    }
}


