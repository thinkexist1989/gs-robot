#ifndef SOCKETTHREAD_H
#define SOCKETTHREAD_H

#include <QThread>
#include <vector>
#include <queue>
#include "common.h"
#include "robotdescription.h"

class SocketThread : public QThread
{
    Q_OBJECT
protected:
    void    run()  Q_DECL_OVERRIDE;  //线程任务
public:
    explicit SocketThread(QObject *parent = nullptr);
    bool    send_msg;
    std::queue<std::array<double, KDL_ROBOT_JOINT_NUM> > joints;
signals:
    void    sendRobotParam(RobotParam param);
};

#endif // SOCKETTHREAD_H
