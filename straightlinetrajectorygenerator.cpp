#include "straightlinetrajectorygenerator.h"

// 构造函数
StraightLineTrajectoryGenerator::StraightLineTrajectoryGenerator(const double acc_limit,
                                const double v_limit, double time_step) : dt(time_step) {
    // 设置默认限制
    input.max_velocity     = {v_limit, v_limit, v_limit, v_limit, v_limit, v_limit, v_limit};  // 位置和角速度限制
    input.max_acceleration = {acc_limit, acc_limit, acc_limit, acc_limit, acc_limit, acc_limit, acc_limit};  // 加速度限制
    input.max_jerk         = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};  // 加加速度限制
    input.min_velocity     = {-v_limit, -v_limit, -v_limit, -v_limit, -v_limit, -v_limit, -v_limit};
    input.min_acceleration = {-acc_limit, -acc_limit, -acc_limit, -acc_limit, -acc_limit, -acc_limit, -acc_limit};
}

// 计算单位方向向量
void StraightLineTrajectoryGenerator::normalize(double& dx, double& dy, double& dz) {
    double length = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (length > 1e-6) {
        dx /= length;
        dy /= length;
        dz /= length;
    }
}

Pose StraightLineTrajectoryGenerator::calculateAdjustmentTargetPose(const Pose& start_pose, const double xx, const double yy,  const double zz, const double ww){
    // 目标坐标系C,相对于末端坐标系B的旋转矩阵
    KDL::Rotation rotBC = KDL::Rotation::Quaternion(xx, yy, zz, ww);
    // 末端坐标系B,相对于基坐标系A的旋转矩阵
    KDL::Rotation rotAB = KDL::Rotation::Quaternion(
            start_pose.xx,
            start_pose.yy,
            start_pose.zz,
            start_pose.ww
    );
    // 目标坐标系C,相对于基坐标系A的旋转矩阵
    KDL::Rotation rotAC = rotAB * rotBC;
    double a,b,c,d;
    rotAC.GetQuaternion(a,b,c,d);
    // 计算目标姿态
    Pose target = start_pose;
    target.xx = a;
    target.yy = b;
    target.zz = c;
    target.ww = d;
    // 保持相同位置
    target.x = start_pose.x;
    target.y = start_pose.y;
    target.z = start_pose.z;
    return target;
}

// 从方向向量计算直线运动目标位姿
Pose StraightLineTrajectoryGenerator::calculateLineTargetPose(const Pose& start_pose,
                        double direction_x, double direction_y, double direction_z // 末端XYZ方向直线运动
                        ) {
    // 末端坐标系B,相对于基坐标系A的旋转矩阵
    KDL::Rotation rotAB = KDL::Rotation::Quaternion(
            start_pose.xx,
            start_pose.yy,
            start_pose.zz,
            start_pose.ww
    );
    // 构造方向向量
    KDL::Vector PB(direction_x, direction_y, direction_z);
    // 旋转矩阵乘以向量
    KDL::Vector PA = rotAB * PB;
    // 计算目标位置
    Pose target = start_pose;
    target.x = start_pose.x + PA.x() ;
    target.y = start_pose.y + PA.y() ;
    target.z = start_pose.z + PA.z() ;

    // 保持相同姿态（直线运动通常不改变姿态）
    target.xx = start_pose.xx;
    target.yy = start_pose.yy;
    target.zz = start_pose.zz;
    target.ww = start_pose.ww;
    return target;
}

// 根据roll, pitch, yaw 固定角调整末端的姿态，位置不动。
bool StraightLineTrajectoryGenerator::generate_end_effector_adjustment_trajectory(const Pose& start_pose,
                                                                                  const double xx, const double yy,  const double zz, const double ww,
                                                 std::vector<Pose>& poses,
                                                 double& duration, Pose& target_pose){
    // 1. 计算目标位姿
    std::cout << "start_pose:";
    start_pose.printPose();
    target_pose = calculateAdjustmentTargetPose(start_pose, xx, yy, zz, ww);
    std::cout << "target_pose:";
    target_pose.printPose();
    bool status = generateTrajectory(start_pose, target_pose, poses, duration);
    return status;
}

// 生成直线轨迹
bool StraightLineTrajectoryGenerator::generateLineTrajectory(const Pose& start_pose,
                       double direction_x, double direction_y, double direction_z,
                       std::vector<Pose>& poses,
                       double& duration, Pose& target_pose) {
    std::cout << "start_pose:";
    start_pose.printPose();
    // 1. 计算目标位姿
    target_pose = calculateLineTargetPose(start_pose, direction_x, direction_y, direction_z);
    std::cout << "target_pose:";
    target_pose.printPose();
    bool status = generateTrajectory(start_pose, target_pose, poses, duration);
    return status;
}

// 生成轨迹
bool StraightLineTrajectoryGenerator::generateTrajectory(const Pose& start_pose,
                        const Pose& target_pose,
                       std::vector<Pose>& poses,
                       double& duration) {  // 默认5秒完成
    // 2. 设置起始状态
    // 位置: [x, y, z, qx, qy, qz, qw] 但Ruckig需要速度/加速度，我们只控制位置
    input.current_position = {start_pose.x, start_pose.y, start_pose.z, start_pose.xx, start_pose.yy, start_pose.zz, start_pose.ww};
    input.current_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // 3. 设置目标状态
    input.target_position = {target_pose.x, target_pose.y, target_pose.z, target_pose.xx, target_pose.yy, target_pose.zz, target_pose.ww};
    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // 4. 生成轨迹
    ruckig::Trajectory<7> trajectory;
    ruckig::Result result = otg.calculate(input, trajectory);
    if (result == ruckig::Result::ErrorInvalidInput) {
        std::cerr << "输入参数无效，无法计算轨迹！" << std::endl;
        return false;
    }
    // 5. 打开 CSV 文件
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_time);
    std::ostringstream oss;
    oss << "ruckig_trajectory_"
        << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
        << ".csv";
    std::string filename = oss.str();
    std::ofstream csv(filename);
    if (!csv.is_open()) {
        std::cerr << "无法打开 CSV 文件！" << std::endl;
        return false;
    }
    // 6. 写 CSV 表头
    csv << "time";
    for (int i = 0; i < 7; ++i)
        csv << ",pos" << i+1;
    csv << "\n";
    // 7. 获取并打印轨迹总时长
    duration = trajectory.get_duration();
    std::cout << "轨迹计算成功！总时长: " << duration << " 秒." << std::endl;
    // 示例：批量提取离散时间点的轨迹 (例如以 0.01s 为间隔)
    std::array<double, 7> pos, vel, acc;
    for (double t = 0.0; t <= duration; t += dt) {
        trajectory.at_time(t, pos, vel, acc);
        // 将 pos, vel, acc 存入容器或发送给下游系统
        poses.push_back(Pose(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]));
//        std::cout << "t=" << t << "s | Pos: " << pos[0] << " " <<
//        pos[1] << " " << pos[2] << " " << pos[3] << " " << pos[4] << " " << pos[5] << " "<<  pos[6] << " " << std::endl;
        csv << std::fixed << std::setprecision(7) << t;
        for (int i = 0; i < 7; ++i) {
            csv << "," << pos[i];
        }
        csv << "\n";
    }
    csv.close();
    return true;
}
