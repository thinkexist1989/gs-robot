#ifndef ROBOTDESCRIPTION_H
#define ROBOTDESCRIPTION_H

#include <deque>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/framevel.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <istream>
#define  KDL_ROBOT_JOINT_NUM 7
#ifndef SOLVER_POS_LMA
#define SOLVER_POS_LMA 1
#endif

struct CJointTheta
{
    double JointValue[KDL_ROBOT_JOINT_NUM];
    double Velocity[KDL_ROBOT_JOINT_NUM];
    double Acceleration[KDL_ROBOT_JOINT_NUM];
};

class RobotDescription
{
public:
    float  initX;
    float  initY;
    float  initZ;
    int    nDof = KDL_ROBOT_JOINT_NUM;
    int    nToolsNum;
    int    ControlTime;
    double DHParams[KDL_ROBOT_JOINT_NUM][6];  //DH参数 7个自由度, d/a/alf/theta/max/min;
    double ManipulatorParams[3][4];  //末端工具参数
    RobotDescription();
    std::deque<CJointTheta> Trace_queue;//机械臂1关节空间轨迹点
    bool SetRobotDH(void);
    double dCurJointValue[KDL_ROBOT_JOINT_NUM]; //unit:deg 机械臂各个关节值，从传感器读到的实际值，需要按采集周期更新
    double lastFeedbackAngle[KDL_ROBOT_JOINT_NUM]; //unit:rad
    void setControlTime(int ctrlTime);
    int MOVEZ(double pos1, double pos2, double pos3, double pos4, double pos5, double pos6, double pos7, double accmax); //梯形规划
    void setDH();
    int computeForwardKinematics(KDL::Frame &p_out);
    int computeInverseKinematics(const KDL::Frame &p_in, double out[KDL_ROBOT_JOINT_NUM], bool jointLimit = false);
    int computeInverseKinematics(const KDL::Rotation &rot, const KDL::Vector &pos,
                                 double out[KDL_ROBOT_JOINT_NUM], bool jointLimit = false);
private:
    KDL::Chain sxRobot;
};

#endif // ROBOTDESCRIPTION_H
