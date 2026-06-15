#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QDate>
#include <QTimer>
#include <QElapsedTimer>
#include <QMessageBox>
//引入qt中串口通信需要的头文件
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTcpSocket>
#include <QtNetwork>
#include <QUdpSocket>
#include <deque>
#include <fstream>
#include "common.h"
#include "robotdescription.h"
#include "straightlinetrajectorygenerator.h"
#include "ethercatthread.h"
#include "socketthread.h"
#include "vision.h"

struct AdmittanceParams {
    double M_xy = 1.0;        // XY 导纳质量 (kg)
    double B_xy = 300.0;      // XY 导纳阻尼 (N·s/m)
    double K_xy = 0.0;        // XY 导纳刚度 (N/m)
    double M_rz = 0.01;       // 绕Z旋转导纳转动惯量 (kg·m²)
    double B_rz = 300.0;      // 绕Z旋转导纳阻尼 (N·m·s/rad)
    double K_rz = 0.0;        // 绕Z旋转导纳刚度 (N·m/rad)
    double mz_gain = 5.0;     // Mz 力矩放大倍数
    double delta_xy_max = 0.01;   // XY 偏移限幅 (m)
    double delta_rz_max = 5.0 * M_PI / 180.0;  // Z轴旋转限幅 (rad)
    double force_threshold = 0.5; // 力死区阈值 (N)
    double torque_threshold = 0.05; // 力矩死区阈值 (N·m)
};

struct AdmittanceState {
    double vel_x = 0.0, vel_y = 0.0;  // XY 速度积分状态
    double pos_x = 0.0, pos_y = 0.0;  // XY 位置积分状态
    double vel_rz = 0.0;               // Z旋转速度积分状态
    double pos_rz = 0.0;               // Z旋转位置积分状态
    void reset() { vel_x = vel_y = pos_x = pos_y = vel_rz = pos_rz = 0.0; }
};

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

#define ControlTime (EC_CYCLETIME_US / 1000) //10ms

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    int calSimpleLine(int32_t currentPos, int32_t targetPos, std::vector<int32_t> & line, int num = 0);

    //机械臂相关start
    RobotDescription robotDescription;
    Pose lastFeedbackPositionPose();
    void makeTrajectory(TRAJECTORY_TYPE type);
    void makeMoveL();
    //机械臂相关end

protected:

private slots:
    void onGetEngDatas(EngDatas engDatas);
    void onSendRunOver();
    void    onThread_RobotParam(RobotParam param);
    void    onConnected();
    void    onDisconnected();
    void    onSocketStateChange(QAbstractSocket::SocketState socketState);
    void    onSocketReadyRead();//读取socket传入的数据
    void    onUDPReadyRead();
    void    on_actConnect_clicked();
    void    on_actDisconnect_clicked();
    void on_actionArm_Setting_triggered(bool checked);
    void on_reset1_clicked();
    void on_reset2_clicked();
    void on_reset3_clicked();
    void on_reset4_clicked();
    void on_reset5_clicked();
    void on_reset6_clicked();
    void on_jihuo1_clicked();
    void on_jihuo2_clicked();
    void on_jihuo3_clicked();
    void on_jihuo4_clicked();
    void on_jihuo5_clicked();
    void on_jihuo6_clicked();
    void on_run1_clicked();
    void on_run2_clicked();
    void on_run3_clicked();
    void on_run4_clicked();
    void on_run5_clicked();
    void on_run6_clicked();
    void on_disable1_clicked();
    void on_disable2_clicked();
    void on_disable3_clicked();
    void on_disable4_clicked();
    void on_disable5_clicked();
    void on_disable6_clicked();
    void run_one_joint(int jointID, QString& text, InputsCMD& cmd);
    void on_reset7_clicked();
    void on_jihuo7_clicked();
    void on_run7_clicked();
    void on_disable7_clicked();
    void on_movez_go_clicked();
    void on_let_same_clicked();
    void on_lineGo_clicked();
    void on_actStart_clicked();
    void on_actStop_clicked();
    void on_visual_start_clicked();
    void on_visual_stop_clicked();
    void on_adjustGo_clicked();
    void on_run_by_vision_clicked();
    void on_moveLGo_clicked();
    void on_admApply_clicked();

private:
    EngDatas engDatas;
    Ui::MainWindow *ui;
    QDateTime curDateTime;          //当前时间
    EtherCatThread    etherThread;
    QString char2QString(const char *charresult, uint length);
    //socket
    SocketThread socketT;
    QTcpSocket  *tcpClient;
    QUdpSocket  *udpSocket;
    bool exeByVision = false;
    QString getLocalIP();//获取本机IP地址
    void udpSet();
    QLabel  *LabSocketState;  //状态栏显示标签

    // 导纳控制
    AdmittanceParams admParams;
    AdmittanceState  admState;
    KDL::Rotation sensorRot;  // 六维力传感器坐标系到法兰盘坐标系的旋转（绕Z轴135°）
    M4313_TxPDO ftZeroDrift;  // 六维力零漂
    void computeAdmittanceDelta(const M4313_TxPDO& ft, const KDL::Rotation& R_tool,
                                double dt, double& delta_x, double& delta_y, double& delta_rz);
};
#endif // MAINWINDOW_H
