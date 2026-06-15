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
};
#endif // MAINWINDOW_H
