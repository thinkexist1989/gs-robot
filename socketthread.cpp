#include "socketthread.h"

void SocketThread::run()
{
    qRegisterMetaType<RobotParam>("RobotParam");//注册新类型,发送sendRobotParam信号用
    while (true) {
        if(send_msg){
            if(joints.size() > 0){
                std::array<double, KDL_ROBOT_JOINT_NUM> joint_angle = joints.front();
                RobotParam param;
                param.cmd[0] = 'P';
                param.x = joint_angle[0];
                param.y = joint_angle[1];
                param.z = joint_angle[2];
                param.alf = joint_angle[3];
                param.beta = joint_angle[4];
                param.gama = joint_angle[5];
                emit sendRobotParam(param);  //发射信号
                joints.pop();
            } else {
                send_msg = false;
            }
            msleep(100);
        } else {
            msleep(1000);
        }
    }
}

SocketThread::SocketThread(QObject *parent) : QThread(parent)
{

}
