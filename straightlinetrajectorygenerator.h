#ifndef STRAIGHTLINETRAJECTORYGENERATOR_H
#define STRAIGHTLINETRAJECTORYGENERATOR_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <ctime>
#include <ruckig/ruckig.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>

// 简单的向量和位姿结构
struct Pose {
    double x, y, z;           // 位置
    double xx, yy, zz, ww;    // Quaternion
    Pose() : x(0), y(0), z(0), xx(0), yy(0), zz(0), ww(1.0) {}
    Pose(double x, double y, double z, double xx, double yy, double zz, double ww)
        : x(x), y(y), z(z), xx(xx), yy(yy), zz(zz), ww(ww) {}
    void printPose() const{
        KDL::Rotation rot = KDL::Rotation::Quaternion(xx, yy, zz, ww);
        double roll, pitch, yaw;
        rot.GetRPY(roll, pitch, yaw);
        std::cout << "Pose:" << x << " " << y << " " << z << " "
                  << xx << " " << yy << " " << zz << " " << ww << std::endl;
        std::cout << "RPY:" << roll * KDL::rad2deg << " " <<  pitch * KDL::rad2deg << " " << yaw * KDL::rad2deg << std::endl;
    }
};

typedef enum TRAJECTORY_TYPE {
    LINE_TRAJECTORY   = 0x00,  // 只进行直线运动
    ADJUST_TRAJECTORY = 0x01,  // 只进行姿态调整
    ADJUST_LINE_LINE  = 0x02   // 先姿态调整，再XY直线，最后Z直线
} TRAJECTORY_TYPE;

// 直线轨迹生成器
class StraightLineTrajectoryGenerator {
private:
    ruckig::Ruckig<7> otg;  // 7自由度位姿, 注意这里的自由度不是关节个数，而是位置参数，姿态参数,总共7个参数
    ruckig::InputParameter<7> input;
    ruckig::OutputParameter<7> output;
    double dt;  // 时间步长
    // 计算单位方向向量
    void normalize(double& dx, double& dy, double& dz);

    Pose calculateAdjustmentTargetPose(const Pose& start_pose, const double xx, const double yy,  const double zz, const double ww);

    // 从方向向量计算直线运动目标位姿
    Pose calculateLineTargetPose(const Pose& start_pose,
                            double direction_x, double direction_y, double direction_z // 末端XYZ方向直线运动
                            );
    // 生成轨迹
    bool generateTrajectory(const Pose& start_pose,
                            const Pose& target_pose,
                           std::vector<Pose>& poses,
                           double& duration);

public:
    // 构造函数
    StraightLineTrajectoryGenerator(const double acc_limit,
                                    const double v_limit, double time_step = 0.001);

    // 根据roll, pitch, yaw 固定角调整末端的姿态，位置不动。
    bool generate_end_effector_adjustment_trajectory(const Pose& start_pose, const double xx, const double yy,  const double zz, const double ww,
                                                     std::vector<Pose>& poses,
                                                     double& duration, Pose& target_pose);

    // 生成直线轨迹
    bool generateLineTrajectory(const Pose& start_pose,
                           double direction_x, double direction_y, double direction_z,
                           std::vector<Pose>& poses,
                           double& duration, Pose& target_pose);
};

#endif // STRAIGHTLINETRAJECTORYGENERATOR_H
