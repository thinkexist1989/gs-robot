#include "robotdescription.h"

RobotDescription::RobotDescription()
{

}

void RobotDescription::setDH()
{
    // 1. 定义机器人运动链  DH_Craig1989 double a, double alpha, double d, double theta
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(
                                        DHParams[0][1], DHParams[0][2]*KDL::deg2rad, DHParams[0][0], DHParams[0][3]*KDL::deg2rad)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(
                                        DHParams[1][1], DHParams[1][2]*KDL::deg2rad, DHParams[1][0], DHParams[1][3]*KDL::deg2rad)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(
                                        DHParams[2][1], DHParams[2][2]*KDL::deg2rad, DHParams[2][0], DHParams[2][3]*KDL::deg2rad)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(
                                        DHParams[3][1], DHParams[3][2]*KDL::deg2rad, DHParams[3][0], DHParams[3][3]*KDL::deg2rad)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(
                                        DHParams[4][1], DHParams[4][2]*KDL::deg2rad, DHParams[4][0], DHParams[4][3]*KDL::deg2rad)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(
                                        DHParams[5][1], DHParams[5][2]*KDL::deg2rad, DHParams[5][0], DHParams[5][3]*KDL::deg2rad)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(
                                        DHParams[6][1], DHParams[6][2]*KDL::deg2rad, DHParams[6][0], DHParams[6][3]*KDL::deg2rad)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    sxRobot.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame::DH_Craig1989(
                                        ManipulatorParams[0][1],
                                        ManipulatorParams[0][2]*KDL::deg2rad,
                                        ManipulatorParams[0][0],
                                        ManipulatorParams[0][3]*KDL::deg2rad)));
}

int RobotDescription::computeForwardKinematics(KDL::Frame &p_out){
    KDL::ChainFkSolverPos_recursive fksolver(sxRobot);
    unsigned int joint_num = sxRobot.getNrOfJoints();
    KDL::JntArray q_in(joint_num);
    for (unsigned int i=0; i<joint_num; i++) {
        q_in(i) = lastFeedbackAngle[i];
    }
    int state = fksolver.JntToCart(q_in, p_out);
    if(state>=0){
        return 1;
    } else {
        return -1;
    }
}
int RobotDescription::computeInverseKinematics(const KDL::Frame &p_in, double out[KDL_ROBOT_JOINT_NUM], bool jointLimit){
    double R,P,Y; //姿态 GetRPY
    p_in.M.GetRPY(R,P,Y);
    KDL::Rotation rot = rot.RPY(R,P,Y);
    KDL::Vector   pos = p_in.p;
    int result = 0;
    result = computeInverseKinematics(rot, pos, out, jointLimit);
    return result;
}

int RobotDescription::computeInverseKinematics(const KDL::Rotation &rot, const KDL::Vector &pos, double out[KDL_ROBOT_JOINT_NUM], bool jointLimit){
    static int index = 0;
    index++;
    // 1. 设置目标末端位姿
    KDL::Frame goal_pose = KDL::Frame(rot, pos);

    // 2. 初始化关节角度和结果容器
    unsigned int num_jnts = sxRobot.getNrOfJoints();
    KDL::JntArray seed_array = KDL::JntArray(num_jnts);
    for (size_t i = 0; i < num_jnts; i++){
        seed_array(i) = lastFeedbackAngle[i];
    }
    KDL::JntArray result_angles = KDL::JntArray(num_jnts);

    // 3. 权重矩阵（前3个位置，后4个方向）
    Eigen::Matrix<double, 6, 1> L;
    L(0)=0.01;L(1)=0.01;L(2)=0.01;  L(3)=1;L(4)=1;L(5)=1;

    // 4. 创建逆运动学求解器
    KDL::JntArray q_min(num_jnts), q_max(num_jnts);
    for(unsigned int i=0; i<num_jnts; i++){
        q_max(i) = DHParams[i][4]*KDL::deg2rad;
        q_min(i) = DHParams[i][5]*KDL::deg2rad;
    }

#if SOLVER_POS_LMA
    KDL::ChainIkSolverPos_LMA  ik_solver(sxRobot);
    // 5. 求解逆运动学
    int status = ik_solver.CartToJnt(seed_array, goal_pose, result_angles);
    // 6. 结果处理
    switch (status) {
        case KDL::SolverI::E_NOERROR:
            for (unsigned int i = 0; i < result_angles.rows(); ++i) {
                out[i] = result_angles(i);
                //std::cout << "j" << (i+1) << ":" << out[i] << "rad" << std::endl;
            }
            return 1;
        case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED:
            std::cerr << "错误：超过最大迭代次数" << index << std::endl;
            return -1;
        case KDL::SolverI::E_NOT_UP_TO_DATE:
            std::cerr << "错误：模型未更新" << index << std::endl;
            return -1;
        default:
            std::cerr << "未知错误: " << status << " " << index << std::endl;
            return -1;
    }
#endif

#if SOLVER_POS_NR_JL
    KDL::ChainFkSolverPos_recursive fk_solver(sxRobot);
    KDL::ChainIkSolverVel_pinv      ik_vel_solver(sxRobot);
    KDL::ChainIkSolverPos_NR_JL     ik_solver(sxRobot, q_min, q_max, fk_solver, ik_vel_solver, 10000, 1e-4);
    // 5. 求解逆运动学
    int status = ik_solver.CartToJnt(seed_array, goal_pose, result_angles);

    // 6. 结果处理
    switch (status) {
        case KDL::SolverI::E_NOERROR:
            for (unsigned int i = 0; i < result_angles.rows(); ++i) {
                out[i] = result_angles(i);
                //std::cout << "j" << (i+1) << ":" << out[i] << "rad" << std::endl;
            }
            return 1;
        case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED:
            std::cerr << "错误：超过最大迭代次数" << index << std::endl;
            return -1;
        case KDL::SolverI::E_NOT_UP_TO_DATE:
            std::cerr << "错误：模型未更新" << index << std::endl;
            return -1;
        default:
            std::cerr << "未知错误: " << status << " " << index << std::endl;
            return -1;
    }
#endif
    return 0;
}

bool RobotDescription::SetRobotDH(void)
{
    FILE *file=NULL;
    file = fopen("robotdh.txt", "r+");
    if (file == NULL)
    {
        return false;
    }
    char sBbuf[100]={0};
    fscanf(file, "%s", sBbuf);
    if(strcmp(sBbuf,"DH"))
    {
        fclose(file);
        return false;
    }
    fscanf(file, "%s",sBbuf);//跳过说明
    fscanf(file, "%f %f %f", &this->initX, &this->initY, &this->initZ);
    //自由度
    fscanf(file, "%d %d", &this->nDof, &this->nToolsNum);
    if(nDof != KDL_ROBOT_JOINT_NUM)
    {
        fclose(file);
        return false;
    }
    for (int j=0;j<nDof;j++)
    {
        float d,a,alf,theta,max,min;
        fscanf(file, "%f %f %f %f %f %f", &d,&a,&alf,&theta,&max,&min);
        DHParams[j][0]=d;
        DHParams[j][1]=a;
        DHParams[j][2]=alf;
        DHParams[j][3]=theta;
        DHParams[j][4]=max;
        DHParams[j][5]=min;
    }
    if((nToolsNum<=3) && (nToolsNum>=1))
    {
        for(int j=0;j<nToolsNum;j++)
        {
            float d,a,alf,theta;
            fscanf(file, "%f %f %f %f ", &d,&a,&alf,&theta);
            ManipulatorParams[j][0]=d;
            ManipulatorParams[j][1]=a;
            ManipulatorParams[j][2]=alf;
            ManipulatorParams[j][3]=theta;
        }
    }
    else
    {
        fclose(file);
        return false;
    }
    fclose(file);
    setDH();
    return true;
}

void RobotDescription::setControlTime(int ctrlTime){
    ControlTime = ctrlTime;
}

int RobotDescription::MOVEZ(double pos1, double pos2, double pos3, double pos4, double pos5, double pos6, double pos7, double accmax)
{
    double dCurrentPos[KDL_ROBOT_JOINT_NUM], deltPos[KDL_ROBOT_JOINT_NUM], deltT[KDL_ROBOT_JOINT_NUM],
            maxTime, yt[KDL_ROBOT_JOINT_NUM], vt[KDL_ROBOT_JOINT_NUM], at[KDL_ROBOT_JOINT_NUM], goalPos[KDL_ROBOT_JOINT_NUM];
    double t1[KDL_ROBOT_JOINT_NUM], t2[KDL_ROBOT_JOINT_NUM],vcc[KDL_ROBOT_JOINT_NUM];
    goalPos[0] = pos1;
    goalPos[1] = pos2;
    goalPos[2] = pos3;
    goalPos[3] = pos4;
    goalPos[4] = pos5;
    goalPos[5] = pos6;
    goalPos[6] = pos7;
    for (int i = 0; i < KDL_ROBOT_JOINT_NUM; i++)
    {
        dCurrentPos[i] = dCurJointValue[i];
        deltPos[i] = goalPos[i] - dCurrentPos[i];
        t1[i] = sqrt(3 * fabs(deltPos[i]) / (5 * accmax));
        vcc[i] = accmax*t1[i];
        if (vcc[i] < 0.0000001)
        {
            t2[i] = 0;
            deltT[i] = 0;
        }
        else
        {
            t2[i] = fabs(2*deltPos[i]) / (5*vcc[i]);
            deltT[i] = t1[i] * 2 + t2[i];
        }
    }
    //obtain the max time
    int jointmaxt = 0;
    maxTime = 0;
    for (int i = 0; i < KDL_ROBOT_JOINT_NUM; i++)
    {
        if (deltT[i]>maxTime)
        {
            maxTime = deltT[i];
            jointmaxt = i;
        }
    }
    double a[KDL_ROBOT_JOINT_NUM], V[KDL_ROBOT_JOINT_NUM];
    double tf=maxTime;
    //求最大时间下的各个关节加速度和速度
    for (int  i = 0; i < KDL_ROBOT_JOINT_NUM; i++)
    {
        a[i] = 3* deltPos[i]/(5*t1[jointmaxt]* t1[jointmaxt]);
        V[i] = a[i] * t1[jointmaxt];
    }
    if (maxTime<0.0000001)
    {
        //目标位置与现有位置相同，不需要动
        return -1;
    }
    int num = int(ceil(maxTime*1000.0 / ControlTime));
    if (num <= 0)
    {
        return -1;
    }
    ////////////////
    /*FILE *file = NULL;
    errno_t errLoad;
    errLoad = fopen_s(&file, "test.txt", "w+");
    if (errLoad != 0)
    {
        return -1;
    }*/

    ///////////////
    //创建动态数组，存储每个点矩阵
    Trace_queue.clear();
    for (int t = 1;t <= num;t++)
    {
        CJointTheta pos;
        for (int i=0; i < KDL_ROBOT_JOINT_NUM; i++)
        {
            if((t*ControlTime/1000.0f)<=t1[jointmaxt])
            {
                yt[i]=dCurrentPos[i]+a[i]/2*t*(ControlTime/1000.0f)*t*(ControlTime/1000.0f);
                vt[i]=a[i]*t*(ControlTime/1000.0f);
                at[i]=a[i];
            }
            else if((t*ControlTime/1000.0f)<=(tf-t1[jointmaxt]))
            {
                yt[i]=(goalPos[i]+dCurrentPos[i]-V[i]*tf)/2+V[i]*t*ControlTime/1000.0f;
                vt[i]=V[i];
                at[i]=0;
            }
            else
            {
                yt[i]=goalPos[i]-a[i]/2*tf*tf+a[i]*tf*t*ControlTime/1000.0f-a[i]/2*(t*ControlTime/1000.0f)*(t*ControlTime/1000.0f);
                vt[i]=a[i]*tf-a[i]*(t*ControlTime/1000.0f);
                at[i]=0-a[i];
            }
            pos.JointValue[i]=yt[i];
            pos.Velocity[i]=fabs(vt[i]);//fabs(V[i]);//vt[i];
            pos.Acceleration[i]=fabs(at[i]);//fabs(a[i]);//at[i];
        }
        ////////////////////////
        //fprintf_s(file, "%f\n", at[0]);
        //fprintf_s(file, "%f %f %f\r\n", yt[0],vt[0],at[0]);
        //////////////////////////
        Trace_queue.push_back(pos);
    }
    //fclose(file);
    return 0;
}
