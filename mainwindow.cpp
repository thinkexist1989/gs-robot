#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(&socketT, SIGNAL(sendRobotParam(RobotParam)), this, SLOT(onThread_RobotParam(RobotParam)));
    socketT.start();
    tcpClient=new QTcpSocket(this); //创建socket变量
    LabSocketState=new QLabel("Socket状态：");//状态栏标签
    LabSocketState->setMinimumWidth(250);
    ui->statusbar->addWidget(LabSocketState);
    connect(tcpClient,SIGNAL(connected()),this,SLOT(onConnected()));
    connect(tcpClient,SIGNAL(disconnected()),this,SLOT(onDisconnected()));
    connect(tcpClient,SIGNAL(stateChanged(QAbstractSocket::SocketState)),this,SLOT(onSocketStateChange(QAbstractSocket::SocketState)));
    connect(tcpClient,SIGNAL(readyRead()),this,SLOT(onSocketReadyRead()));
    //启动线程
    connect(&etherThread, SIGNAL(sendEngDatas(EngDatas)), this, SLOT(onGetEngDatas(EngDatas)));
    connect(&etherThread, SIGNAL(sendRunOver()), this, SLOT(onSendRunOver()));
    etherThread.start();
    udpSet();
    //载入机械臂的配置信息
    robotDescription.setControlTime(ControlTime);
    bool bRes=robotDescription.SetRobotDH();
    if(!bRes)
    {
        printf("载入机械臂的配置信息FALSE.\n");
    }
}

QString MainWindow::getLocalIP()
{
    QString   hostName=QHostInfo::localHostName();//本地主机名
    QHostInfo hostInfo=QHostInfo::fromName(hostName);
    QString   localIP="";
    QList<QHostAddress> addList=hostInfo.addresses();//
    if (!addList.isEmpty())
    for (int i=0;i<addList.count();i++)
    {
        QHostAddress aHost=addList.at(i);
        if (QAbstractSocket::IPv4Protocol==aHost.protocol())
        {
            localIP=aHost.toString();
            break;
        }
    }
    return localIP;
}

void MainWindow::udpSet()
{
    QString localIP=getLocalIP();//本机IP
    this->setWindowTitle(this->windowTitle() + "----本机IP:"+localIP);
    ui->comboTargetIP->addItem(localIP);
    udpSocket=new QUdpSocket(this);//用于与连接的客户端通讯的QTcpSocket
    connect(udpSocket,SIGNAL(stateChanged(QAbstractSocket::SocketState)), this,SLOT(onSocketStateChange(QAbstractSocket::SocketState)));
    onSocketStateChange(udpSocket->state());
    connect(udpSocket,SIGNAL(readyRead()), this,SLOT(onUDPReadyRead()));
}

void MainWindow::onGetEngDatas(EngDatas engDatas)
{
    this->engDatas = engDatas;
    for (int i=0; i<7; i++) {
        if(i<=3){ // 0 1 2 3
            robotDescription.dCurJointValue[i] = engDatas.ActualStatus[i].position * 360.0 / CNT_PER_CYCLE / BIG_JOINT_DACC;
        }else{ // 4 5 6
            robotDescription.dCurJointValue[i] = engDatas.ActualStatus[i].position * 360.0 / CNT_PER_CYCLE / SMALL_JOINT_DACC;
        }
    }
    ui->Statusword1->setText(QString::number(engDatas.ActualStatus[0].statusword));
    ui->Statusword2->setText(QString::number(engDatas.ActualStatus[1].statusword));
    ui->Statusword3->setText(QString::number(engDatas.ActualStatus[2].statusword));
    ui->Statusword4->setText(QString::number(engDatas.ActualStatus[3].statusword));
    ui->Statusword5->setText(QString::number(engDatas.ActualStatus[4].statusword));
    ui->Statusword6->setText(QString::number(engDatas.ActualStatus[5].statusword));
    ui->Statusword7->setText(QString::number(engDatas.ActualStatus[6].statusword));
    ui->Status2word1->setText(QString::number(engDatas.ActualStatus[0].statusword, 2));
    ui->Status2word2->setText(QString::number(engDatas.ActualStatus[1].statusword, 2));
    ui->Status2word3->setText(QString::number(engDatas.ActualStatus[2].statusword, 2));
    ui->Status2word4->setText(QString::number(engDatas.ActualStatus[3].statusword, 2));
    ui->Status2word5->setText(QString::number(engDatas.ActualStatus[4].statusword, 2));
    ui->Status2word6->setText(QString::number(engDatas.ActualStatus[5].statusword, 2));
    ui->Status2word7->setText(QString::number(engDatas.ActualStatus[6].statusword, 2));

    ui->ActualPos1->setText(QString::number(engDatas.ActualStatus[0].position * 360.0 / CNT_PER_CYCLE / BIG_JOINT_DACC));
    ui->ActualPos2->setText(QString::number(engDatas.ActualStatus[1].position * 360.0 / CNT_PER_CYCLE / BIG_JOINT_DACC));
    ui->ActualPos3->setText(QString::number(engDatas.ActualStatus[2].position * 360.0 / CNT_PER_CYCLE / BIG_JOINT_DACC));
    ui->ActualPos4->setText(QString::number(engDatas.ActualStatus[3].position * 360.0 / CNT_PER_CYCLE / BIG_JOINT_DACC));
    ui->ActualPos5->setText(QString::number(engDatas.ActualStatus[4].position * 360.0 / CNT_PER_CYCLE / SMALL_JOINT_DACC));
    ui->ActualPos6->setText(QString::number(engDatas.ActualStatus[5].position * 360.0 / CNT_PER_CYCLE / SMALL_JOINT_DACC));
    ui->ActualPos7->setText(QString::number(engDatas.ActualStatus[6].position * 360.0 / CNT_PER_CYCLE / SMALL_JOINT_DACC));
    ui->ActualVel1->setText(QString::number(engDatas.ActualStatus[0].velocity));
    ui->ActualVel2->setText(QString::number(engDatas.ActualStatus[1].velocity));
    ui->ActualVel3->setText(QString::number(engDatas.ActualStatus[2].velocity));
    ui->ActualVel4->setText(QString::number(engDatas.ActualStatus[3].velocity));
    ui->ActualVel5->setText(QString::number(engDatas.ActualStatus[4].velocity));
    ui->ActualVel6->setText(QString::number(engDatas.ActualStatus[5].velocity));
    ui->ActualVel7->setText(QString::number(engDatas.ActualStatus[6].velocity));
    ui->ModeOpD1->setText(QString::number(engDatas.ActualStatus[0].ModeOp));
    ui->ModeOpD2->setText(QString::number(engDatas.ActualStatus[1].ModeOp));
    ui->ModeOpD3->setText(QString::number(engDatas.ActualStatus[2].ModeOp));
    ui->ModeOpD4->setText(QString::number(engDatas.ActualStatus[3].ModeOp));
    ui->ModeOpD5->setText(QString::number(engDatas.ActualStatus[4].ModeOp));
    ui->ModeOpD6->setText(QString::number(engDatas.ActualStatus[5].ModeOp));
    ui->ModeOpD7->setText(QString::number(engDatas.ActualStatus[6].ModeOp));
}

MainWindow::~MainWindow()
{
    etherThread.stopThread();
    if (etherThread.isRunning()){
        etherThread.stopThread();
    }
    delete ui;
}

void MainWindow::onConnected()
{ //connected()信号槽函数
    LabSocketState->setText("scoket状态：ConnectedState");
    ui->actConnect->setEnabled(false);
    ui->actDisconnect->setEnabled(true);
}

void MainWindow::onDisconnected()
{//disConnected()信号槽函数
    LabSocketState->setText("scoket状态：UnconnectedState");
    ui->actConnect->setEnabled(true);
    ui->actDisconnect->setEnabled(false);
}

void MainWindow::onSocketStateChange(QAbstractSocket::SocketState socketState)
{//stateChange()信号槽函数
    switch(socketState)
    {
    case QAbstractSocket::UnconnectedState:
        LabSocketState->setText("scoket状态：UnconnectedState");
        break;
    case QAbstractSocket::HostLookupState:
        LabSocketState->setText("scoket状态：HostLookupState");
        break;
    case QAbstractSocket::ConnectingState:
        LabSocketState->setText("scoket状态：ConnectingState");
        break;
    case QAbstractSocket::ConnectedState:
        LabSocketState->setText("scoket状态：ConnectedState");
        break;
    case QAbstractSocket::BoundState:
        LabSocketState->setText("scoket状态：BoundState");
        break;
    case QAbstractSocket::ClosingState:
        LabSocketState->setText("scoket状态：ClosingState");
        break;
    case QAbstractSocket::ListeningState:
        LabSocketState->setText("scoket状态：ListeningState");
        break;
    }
}

void MainWindow::onSocketReadyRead()
{//readyRead()信号槽函数
    while(tcpClient->canReadLine()){
        qDebug()<<tcpClient->readLine();
    }
}

void MainWindow::on_run_by_vision_clicked()
{
    if(ui->run_by_vision->text() == "运动"){
        ui->run_by_vision->setText("停运");
        ui->run_by_vision->setStyleSheet("background-color:red");
        exeByVision = true;
    } else {
        ui->run_by_vision->setText("运动");
        ui->run_by_vision->setStyleSheet("");
    }
}

void MainWindow::onSendRunOver()
{
    std::cout << "onSendRunOver" <<std::endl;
    if(ui->run_by_vision->text() == "停运"){
        exeByVision = true;
    }
}

void MainWindow::onUDPReadyRead()
{
    //读取收到的数据报
    while(udpSocket->hasPendingDatagrams())
    {
        QByteArray      datagram;
        int datasize = udpSocket->pendingDatagramSize();
        if(datasize < (int)sizeof(DCMEASUREDATA)){
            continue;
        }
        datagram.resize(datasize);
        QHostAddress    peerAddr;
        quint16         peerPort;
        udpSocket->readDatagram(datagram.data(), datagram.size(), &peerAddr, &peerPort);
        QString         str = char2QString(datagram.data(), sizeof(DCMEASUREDATA));
        Pose pose;
        double roll, pitch, yaw;
        int parse_result = parsingDCMEASUREDATA((unsigned char*)datagram.data(), pose, roll, pitch, yaw);
        qDebug() << "视觉parse_result:" << parse_result;
        if(parse_result == 1 && exeByVision){
            exeByVision = false;
            // 执行末端运动指令
            switch (ui->adjust_ctrl->currentIndex()) {
            case 0:{
                // 默认运动方式：先姿态调整，再XY直线，最后Z直线
                ui->set_roll ->setText(QString::number(roll));
                ui->set_pitch->setText(QString::number(pitch));
                ui->set_yaw  ->setText(QString::number(yaw));
                ui->x_dis->setText(QString::number(pose.x));
                ui->y_dis->setText(QString::number(pose.y));
                ui->z_dis->setText(QString::number(pose.z));
                makeTrajectory(ADJUST_LINE_LINE);
                break;
            }
            case 1:
                // 1. 调整末端姿态
                ui->set_roll ->setText(QString::number(roll));
                ui->set_pitch->setText(QString::number(pitch));
                ui->set_yaw  ->setText(QString::number(yaw));
                makeTrajectory(ADJUST_TRAJECTORY);
                break;
            case 2:
                // 2. xy方向直线运动
                ui->x_dis->setText(QString::number(pose.x));
                ui->y_dis->setText(QString::number(pose.y));
                ui->z_dis->setText("0");
                makeTrajectory(LINE_TRAJECTORY);
                break;
            case 3:
                // 3. z方向直线运动
                ui->x_dis->setText("0");
                ui->y_dis->setText("0");
                ui->z_dis->setText(QString::number(pose.z));
                makeTrajectory(LINE_TRAJECTORY);
                break;
            }
        }
    }
}

void MainWindow::on_actConnect_clicked()
{
    //连接到服务器
    QString     addr=ui->comboServer->currentText();
    quint16     port=ui->spinPort->value();
    tcpClient->connectToHost(addr,port);
}


void MainWindow::on_actDisconnect_clicked()
{
    //断开与服务器的连接
    if (tcpClient->state()==QAbstractSocket::ConnectedState){
        tcpClient->disconnectFromHost();
    }
}

void MainWindow::onThread_RobotParam(RobotParam param){
    //socket
    tcpClient->write((char*)&param, sizeof(param));
}

void MainWindow::on_actionArm_Setting_triggered(bool checked)
{
    qDebug() << "on_actionArm_Setting_triggered=" << checked;
}

void MainWindow::on_reset1_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 0;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_RESET;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_reset2_clicked()
{  
    InputsCMD cmd;
    cmd.slave_index = 1;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_RESET;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_reset3_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 2;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_RESET;
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_reset4_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 3;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_RESET;
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_reset5_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 4;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_RESET;
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_reset6_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 5;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_RESET;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_reset7_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 6;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_RESET;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_jihuo1_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 0;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_1;
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_jihuo2_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 1;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_1;
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_jihuo3_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 2;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_1;
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_jihuo4_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 3;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_1;
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_jihuo5_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 4;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_1;
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_jihuo6_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 5;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_1;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_jihuo7_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 6;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_1;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::run_one_joint(int jointID, QString& text, InputsCMD& cmd){
    double position = text.toDouble();
    double outpos = 0.0;
    if(jointID <= 3){ // 0 1 2 3
        outpos = position / 360.0 * CNT_PER_CYCLE * BIG_JOINT_DACC;
    } else { // 4 5 6
        outpos = position / 360.0 * CNT_PER_CYCLE * SMALL_JOINT_DACC;
    }
    cmd.slave_index = jointID;
    cmd.exe_index = 0xFF;
    cmd.cmd = STATE_RUN;

    int32_t now = engDatas.ActualStatus[jointID].position;
    std::vector<int32_t> line;
    calSimpleLine(now, outpos, line);
    cmd.profileDatas[jointID].position = line;
}

void MainWindow::on_run1_clicked()
{
    QString text = ui->joint_1->text();
    InputsCMD cmd;
    run_one_joint(0, text, cmd);
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_run2_clicked()
{
    QString text = ui->joint_2->text();
    InputsCMD cmd;
    run_one_joint(1, text, cmd);
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_run3_clicked()
{
    QString text = ui->joint_3->text();
    InputsCMD cmd;
    run_one_joint(2, text, cmd);
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_run4_clicked()
{
    QString text = ui->joint_4->text();
    InputsCMD cmd;
    run_one_joint(3, text, cmd);
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_run5_clicked()
{
    QString text = ui->joint_5->text();
    InputsCMD cmd;
    run_one_joint(4, text, cmd);
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_run6_clicked()
{
    QString text = ui->joint_6->text();
    InputsCMD cmd;
    run_one_joint(5, text, cmd);
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_run7_clicked()
{
    QString text = ui->joint_7->text();
    InputsCMD cmd;
    run_one_joint(6, text, cmd);
    etherThread.setInputsCMD(cmd);
}


void MainWindow::on_disable1_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 0;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_DISABLE;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_disable2_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 1;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_DISABLE;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_disable3_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 2;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_DISABLE;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_disable4_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 3;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_DISABLE;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_disable5_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 4;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_DISABLE;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_disable6_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 5;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_DISABLE;
    etherThread.setInputsCMD(cmd);
}

void MainWindow::on_disable7_clicked()
{
    InputsCMD cmd;
    cmd.slave_index = 6;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_DISABLE;
    etherThread.setInputsCMD(cmd);
}

int MainWindow::calSimpleLine(int32_t currentPos, int32_t targetPos, std::vector<int32_t> & line, int num){
    // 2个参,单位都是脉冲
    printf("%d, %d \n", currentPos, targetPos);
    line.clear();
    if(num == 0){
        if (targetPos >= currentPos + CNT_PER_DEBUG){
            for (int32_t i = currentPos;  i<=targetPos;  ) {
                i = i+CNT_PER_DEBUG;
                line.push_back(i);
            }
        } else if(targetPos <= currentPos - CNT_PER_DEBUG){
            for (int32_t i = currentPos;  i>=targetPos;  ) {
                i = i-CNT_PER_DEBUG;
                line.push_back(i);
            }
        }
    } else {
        if (currentPos < targetPos){
            int span = (targetPos - currentPos)/num;
            for (int j=0, i = currentPos; j<num; j++) {
                i = i+span;
                line.push_back(i);
            }
        } else if (currentPos > targetPos){
            int span = (currentPos - targetPos)/num;
            for (int j=0, i = currentPos; j<num; j++) {
                i = i-span;
                line.push_back(i);
            }
        }
    }
    return 1;
}

void MainWindow::on_movez_go_clicked()
{
    double Vel = ui->v_set->text().toDouble(); //  deg
    double acc = ui->acc_set->text().toDouble();
    double goalPos[7];
    goalPos[0] = ui->go1->text().toDouble();
    goalPos[1] = ui->go2->text().toDouble();
    goalPos[2] = ui->go3->text().toDouble();
    goalPos[3] = ui->go4->text().toDouble();
    goalPos[4] = ui->go5->text().toDouble();
    goalPos[5] = ui->go6->text().toDouble();
    goalPos[6] = ui->go7->text().toDouble();

    int error=robotDescription.MOVEZ(goalPos[0], goalPos[1], goalPos[2], goalPos[3], goalPos[4], goalPos[5],  goalPos[6], acc);
    if(error<0)
    {
    }
    else
    {
        InputsCMD cmd;
        cmd.slave_index = 7;
        cmd.exe_index   = 0xff;
        cmd.cmd         = STATE_RUN;
        for (int i=0; i<7; i++) {
            std::vector<int32_t> line;
            for (CJointTheta cjt : robotDescription.Trace_queue) {
                if(i<= 3){
                    line.push_back(cjt.JointValue[i] * CNT_PER_CYCLE * BIG_JOINT_DACC / 360.0);
                } else {
                    line.push_back(cjt.JointValue[i] * CNT_PER_CYCLE * SMALL_JOINT_DACC / 360.0);
                }
            }
            cmd.profileDatas[i].position = line;
            printf("joint %d:", i+1);
            for (int i=0; i<line.size(); i++) {
                printf("%d ", line.at(i));
            }
            printf("\n");
        }
        etherThread.setInputsCMD(cmd);
    }
}

Pose MainWindow::lastFeedbackPositionPose()
{
    for (int i=0; i<KDL_ROBOT_JOINT_NUM; i++) {
        robotDescription.lastFeedbackAngle[i] = robotDescription.dCurJointValue[i]*KDL::deg2rad;
    }
    RobotParam param;
    param.cmd[0] = 'P';
    param.x = robotDescription.dCurJointValue[0];
    param.y = robotDescription.dCurJointValue[1];
    param.z = robotDescription.dCurJointValue[2];
    param.alf = robotDescription.dCurJointValue[3];
    param.beta = robotDescription.dCurJointValue[4];
    param.gama = robotDescription.dCurJointValue[5];
    tcpClient->write((char*)&param, sizeof(param));

    KDL::Frame p_out;
    robotDescription.computeForwardKinematics(p_out);
    double roll, pitch, yaw;
    p_out.M.GetRPY(roll, pitch, yaw);
    double qx,qy,qz,qw;
    p_out.M.GetQuaternion(qx,qy,qz,qw);
    ui->xx->setText(QString::number( p_out.p[0]));
    ui->YY->setText(QString::number( p_out.p[1]));
    ui->ZZ->setText(QString::number( p_out.p[2]));
    ui->roll ->setText(QString::number(roll * KDL::rad2deg));
    ui->pitch->setText(QString::number(pitch * KDL::rad2deg));
    ui->yaw  ->setText(QString::number(yaw * KDL::rad2deg));
    Pose current_pose(p_out.p[0], p_out.p[1], p_out.p[2], qx,qy,qz,qw);
    std::cout << "Rotation:" << p_out.M << std::endl;
    std::cout << "roll, pitch, yaw :" << roll *KDL::rad2deg << " " << pitch *KDL::rad2deg << " " << yaw   *KDL::rad2deg << std::endl;
    return current_pose;
}

QString MainWindow::char2QString(const char *charresult, uint length)
{
    QString src = "";
    for (uint i =0; i<length; i++) {
        QString strNew16 = QString("%1").arg((uchar)(charresult[i]), 2, 16, QLatin1Char('0'));
        src.append(strNew16);
        src.append(' ');
    }
    return src;
}

void MainWindow::on_let_same_clicked()
{
    lastFeedbackPositionPose();
    ui->joint_1->setText(QString::number(robotDescription.dCurJointValue[0]));
    ui->joint_2->setText(QString::number(robotDescription.dCurJointValue[1]));
    ui->joint_3->setText(QString::number(robotDescription.dCurJointValue[2]));
    ui->joint_4->setText(QString::number(robotDescription.dCurJointValue[3]));
    ui->joint_5->setText(QString::number(robotDescription.dCurJointValue[4]));
    ui->joint_6->setText(QString::number(robotDescription.dCurJointValue[5]));
    ui->joint_7->setText(QString::number(robotDescription.dCurJointValue[6]));

    ui->go1->setText(QString::number(robotDescription.dCurJointValue[0]));
    ui->go2->setText(QString::number(robotDescription.dCurJointValue[1]));
    ui->go3->setText(QString::number(robotDescription.dCurJointValue[2]));
    ui->go4->setText(QString::number(robotDescription.dCurJointValue[3]));
    ui->go5->setText(QString::number(robotDescription.dCurJointValue[4]));
    ui->go6->setText(QString::number(robotDescription.dCurJointValue[5]));
    ui->go7->setText(QString::number(robotDescription.dCurJointValue[6]));
}

void MainWindow::on_lineGo_clicked()
{
    makeTrajectory(LINE_TRAJECTORY);
}

void MainWindow::on_adjustGo_clicked()
{
    makeTrajectory(ADJUST_TRAJECTORY);
}

void MainWindow::makeTrajectory(TRAJECTORY_TYPE type){
    Pose start_pose = lastFeedbackPositionPose();
    bool jointLimit = ui->cb_jointLimit->currentIndex()==0 ? true:false;;
    double Vel = ui->v_set->text().toDouble(); // deg
    double acc = ui->acc_set->text().toDouble();
    std::vector<Pose> poses;
    double duration = 5;
    switch (type) {
    case LINE_TRAJECTORY:{
        StraightLineTrajectoryGenerator generator(acc, Vel, 0.003);
        Pose targetPose;
        generator.generateLineTrajectory(
                    start_pose,
                    ui->x_dis->text().toDouble(),
                    ui->y_dis->text().toDouble(),
                    ui->z_dis->text().toDouble(),
                    poses, duration, targetPose);
        break;
    }
    case ADJUST_TRAJECTORY:{
        StraightLineTrajectoryGenerator generator(acc, Vel, 0.003);
        KDL::Rotation rot = KDL::Rotation::RPY(ui->set_roll->text().toDouble() * KDL::deg2rad,
                                               ui->set_pitch->text().toDouble() * KDL::deg2rad,
                                               ui->set_yaw->text().toDouble() * KDL::deg2rad);
        double x,y,z,w;
        rot.GetQuaternion(x,y,z,w);
        Pose targetPose;
        generator.generate_end_effector_adjustment_trajectory(
                    start_pose,
                    x,y,z,w,
                    poses, duration, targetPose);
        break;
    }
    case ADJUST_LINE_LINE:{
        // 111
        StraightLineTrajectoryGenerator generator1(acc, Vel, 0.003);
        KDL::Rotation rot = KDL::Rotation::RPY(ui->set_roll->text().toDouble() * KDL::deg2rad,
                                               ui->set_pitch->text().toDouble() * KDL::deg2rad,
                                               ui->set_yaw->text().toDouble() * KDL::deg2rad);
        double x,y,z,w;
        rot.GetQuaternion(x,y,z,w);
        Pose targetPose;
        generator1.generate_end_effector_adjustment_trajectory(
                    start_pose,
                    x,y,z,w,
                    poses, duration, targetPose);
        //222
        StraightLineTrajectoryGenerator generator2(acc, Vel, 0.003);
        start_pose = targetPose;
        generator2.generateLineTrajectory(
                    start_pose,
                    ui->x_dis->text().toDouble(),
                    ui->y_dis->text().toDouble(),
                    0.0,
                    poses, duration, targetPose);
        //333 当xy误差可接受时，走z方向
        if(abs(ui->x_dis->text().toDouble()) < 0.0004 && abs(ui->y_dis->text().toDouble()) < 0.0004){
            StraightLineTrajectoryGenerator generator3(acc, Vel, 0.003);
            start_pose = targetPose;
            generator3.generateLineTrajectory(
                        start_pose,
                        0.0,
                        0.0,
                        ui->z_dis->text().toDouble(),
                        poses, duration, targetPose);
        }
        break;
    }
    default:
        break;
    }
    // 打开 CSV 文件
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_time);
    std::ostringstream oss;
    oss << "joint_trajectory_"
        << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
        << ".csv";
    std::string filename = oss.str();
    std::ofstream csv(filename);
    if (!csv.is_open()) {
        std::cerr << "无法打开 CSV 文件！" << std::endl;
        return;
    }
    // 写 CSV 表头
    csv << "time";
    for (int i = 0; i < KDL_ROBOT_JOINT_NUM; ++i)
        csv << ",pos" << i+1;
    csv << "\n";
    std::array<std::vector<int32_t>, 7> lines;
    std::queue< std::array<double, KDL_ROBOT_JOINT_NUM> > joints;
    int scan10 = 0;
    std::cout << "poses.size=" << poses.size() << std::endl;
    for (unsigned int j = 0; j<poses.size(); j++ ) {
        Pose &my_pose = poses.at(j);
        KDL::Vector pos(my_pose.x, my_pose.y, my_pose.z);
        KDL::Rotation rot = KDL::Rotation::Quaternion(
                my_pose.xx,
                my_pose.yy,
                my_pose.zz,
                my_pose.ww
        );
        double out[KDL_ROBOT_JOINT_NUM] = {0.0};
        int n = robotDescription.computeInverseKinematics(rot, pos, out, jointLimit);
        if(n == -1){
            return;
        }
        std::array<double, KDL_ROBOT_JOINT_NUM> joint_angle;
        for (int i=0; i<KDL_ROBOT_JOINT_NUM; i++) {
            double eps = out[i] * KDL::rad2deg - robotDescription.lastFeedbackAngle[i] * KDL::rad2deg;
            if(jointLimit && (abs(eps) > 0.02035)){
                // 步伐超过200脉冲, 相当于度: 200×360÷65536÷121=0.00907961
                std::cout << "差值过大,运动中止,j=" << j+1 << "|i=" << i+1 << "|eps=" << eps << std::endl;
                return;
            }
            if(i<= 3){
                lines[i].push_back(out[i] * CNT_PER_CYCLE * BIG_JOINT_DACC / 2 / KDL::PI);
            } else {
                lines[i].push_back(out[i] * CNT_PER_CYCLE * SMALL_JOINT_DACC / 2 / KDL::PI);
            }
            robotDescription.lastFeedbackAngle[i] = out[i];
            csv << "," << out[i] * KDL::rad2deg;
            joint_angle[i] = out[i] * KDL::rad2deg;
        }
        if(scan10 % 10 == 0){
            joints.push(joint_angle);
        }
        scan10++;
        csv << "\n";
    }
    //socket
    socketT.joints = joints;
    socketT.send_msg = true;
    InputsCMD cmd;
    cmd.slave_index = 7;
    cmd.exe_index   = 0xff;
    cmd.cmd         = STATE_RUN;
    for (int i=0; i<KDL_ROBOT_JOINT_NUM; i++) {
        cmd.profileDatas[i].position = lines[i];
    }
    etherThread.setInputsCMD(cmd);
    csv.close();
    return;
}

void MainWindow::on_actStart_clicked()
{
    //绑定端口
    quint16 port = 9000; //本机UDP端口
    if (udpSocket->bind(port))//绑定端口成功
    {
        qDebug() << "**绑定端口：" << QString::number(udpSocket->localPort());
        ui->actStart->setEnabled(false);
        ui->actStop->setEnabled(true);
    } else {
        qDebug() << "**绑定失败";
    }
}


void MainWindow::on_actStop_clicked()
{
    //解除绑定
    udpSocket->abort(); //不能解除绑定
    ui->actStart->setEnabled(true);
    ui->actStop->setEnabled(false);
    qDebug() << "**已解除绑定" ;
}


void MainWindow::on_visual_start_clicked()
{
    //发送消息
    QString      targetIP=ui->comboTargetIP->currentText(); //目标IP
    QHostAddress targetAddr(targetIP);
    quint16      targetPort=ui->spinTargetPort->value();//目标port
    uchar msg[8] = {0x1B, 0x90, 0x24, 0x08, 0x00, 0x00, 0x00, 0x2C};
    udpSocket->writeDatagram((char*)msg,sizeof(msg), targetAddr, targetPort); //发出数据报
    qDebug() << "**测量" ;
}

void MainWindow::on_visual_stop_clicked()
{
    //发送消息
    QString      targetIP=ui->comboTargetIP->currentText(); //目标IP
    QHostAddress targetAddr(targetIP);
    quint16      targetPort=ui->spinTargetPort->value();//目标port
    uchar msg[8] = {0x1B, 0x90, 0x26, 0x08, 0x00, 0x00, 0x00, 0x2E};
    udpSocket->writeDatagram((char*)msg,sizeof(msg), targetAddr, targetPort); //发出数据报
    qDebug() << "**待机" ;
}
