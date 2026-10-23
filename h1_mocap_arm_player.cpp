// =============================================================================
// h1_mocap_arm_player.cpp
//
// 在 h1_mocap_player.cpp 的基础上**只换手臂控制**，底盘、围栏、夹爪、
// 连接/初始化/回放流程全部保持原样不动。
//
// 手臂两种模式（默认 joint）：
//   --arm joint : 关节级，底层 setMotorControl_low，下发**绝对关节角**
//                 姿态取自已跑通的官方表 h1_p0_s4_03_left_arm_poses.cpp
//                 home / raise / straight（肘 0.339 rad = 19.4° 伸直）
//                 来源：arm_joint_player.cpp + gen_official.py
//   --arm pose  : 原来的高层末端位姿 setArm_high（原 h1_mocap_player 逻辑，
//                 完整保留，用于对照）
//
// 为什么底盘末端接口变了（逻辑没变）：
//   关节级控制必须 LOW_LEVEL 模式，LOW_LEVEL 下 setChassis_high 不可用，
//   因此底盘改为直接给左右轮速度（换算与官方 h1_exec_low.cpp 完全一致：
//   轮半径 r=0.10m，半轴距 0.20m，v_l=v-w*0.2, v_r=v+w*0.2, Speed=v/r）。
//   2m 围栏、限速、微分插值全部沿用原代码，v/w 语义不变。
//   --arm pose 时仍走 HIGH_LEVEL + setChassis_high，和原来一模一样。
//
// 输入 CSV（动捕，原格式不变）：
//     t,v,w,lx,ly,lz,lroll,lpitch,lyaw,rx,ry,rz,rroll,rpitch,ryaw,gripper,phase
// 关节序列 CSV（可选，--joint-csv；缺省用内置官方姿态自动生成）：
//     t,m00,...,m22,phase
//
// 运行：
//     ./h1_mocap_arm_player h1_player/data/mission_demo.csv --speed 1.0
//     ./h1_mocap_arm_player mission.csv --joint-csv data/arm_joint_official.csv
//     ./h1_mocap_arm_player mission.csv --arm pose        # 原来的末端位姿方式
//     ./h1_mocap_arm_player mission.csv --ip 10.42.0.1 --lift 0.5 --no-chassis
// =============================================================================

#include "H1_Robot.hpp"
#include "MotorStateMonitor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

static std::atomic<bool> g_run{true};
static void on_sigint(int) { g_run = false; }

// ============================= 测试值修改区域 =============================
// 控制频率：1 秒 rate 次循环（与测试程序保持一致）
static const int rate = 500;
static const int time_sleep = 1000 * 1000 / rate;  // 每次循环间隔（微秒）

// 底盘速度上限（与参考测试程序一致：前进 0.2 m/s，转向 0.1 rad/s，这里略放宽）
static const float max_vx = 0.20f;   // 底盘线速度上限 (m/s)
static const float max_wz = 0.50f;   // 底盘角速度上限 (rad/s)

// 2m 半径安全围栏（防止走出 2m 范围；与 Python 端双重保险）
static const float max_range       = 2.0f;  // 围栏半径 (m)
static const float range_soft_zone = 0.2f;  // 围栏前软减速带宽 (m)

// 夹爪量程：高层用 0~1.45；底层（关节模式）用 0~1.0（官方姿态表口径）
static const float gripper_open = 0.0f;
static const float gripper_close = 1.45f;
static const float gripper_close_low = 1.0f;

// 微分插值步长：除夹爪/升降外的电机建议用微分插值下发，否则控制不稳定
static const float max_dv = 0.02f;    // 底盘线速度变化量 (m/s)/周期
static const float max_dw = 0.05f;    // 底盘角速度变化量 (rad/s)/周期
static const float max_dpos = 0.002f; // 手臂平移变化量 (m)/周期
static const float max_dang = 0.005f; // 手臂旋转变化量 (rad)/周期

// ---- 关节级新增 ----
// 每电机推荐 KP/KD（官方 h1_low_level_KPKD.cpp）
static const float motor_kp[23] = {0, 0, 0, 500, 500, 20, 20, 250, 250, 250, 250, 30,
                                   40, 40, 10, 250, 250, 250, 250, 30, 40, 40, 10};
static const float motor_kd[23] = {0, 0, 0, 5, 5, 1, 1, 5, 5, 5, 5, 1,
                                   1, 1, 1, 5, 5, 5, 5, 1, 1, 1, 1};
static const float max_dangle = 0.0035f;   // 单周期最大转动 (rad) ≈1.75 rad/s
static const float wheel_r = 0.10f;        // 轮半径 (m)，与 h1_exec_low.cpp 一致
static const float wheel_half_b = 0.20f;   // 半轴距 (m)
static const float wheel_speed_max = 2.0f; // 轮速上限 (rad/s)
// =========================================================================

// ---------------------------- 动捕帧（原样） ----------------------------
struct Frame {
    double t = 0.0;
    float v = 0, w = 0;
    float lx = 0, ly = 0, lz = 0, lroll = 0, lpitch = 0, lyaw = 0;
    float rx = 0, ry = 0, rz = 0, rroll = 0, rpitch = 0, ryaw = 0;
    float gripper = 0.0f;  // 0 张开 .. 1 闭合
    int phase = 3;         // 0 approach 1 holding 2 released 3 no_event 4 mission
};

// 底盘 2m 围栏限速：积分 (x, y, yaw)，预测下一步位置，按比例缩放速度。
struct ChassisRange {
    float max_distance = max_range;
    float soft_zone = range_soft_zone;
    float soft_start = max_distance - soft_zone;
    float x = 0.0f, y = 0.0f, yaw = 0.0f;
    double last_t = -1.0;
    bool reached_limit = false;

    std::pair<float, float> limit(float v, float w, double t) {
        double dt = (last_t < 0.0) ? 0.0 : std::max(0.0, t - last_t);
        last_t = t;
        // 1) 速度裁剪
        v = std::max(-max_vx, std::min(max_vx, v));
        w = std::max(-max_wz, std::min(max_wz, w));
        // 2) 预测
        float new_yaw = yaw + w * static_cast<float>(dt);
        float new_x = x + v * std::cos(new_yaw) * static_cast<float>(dt);
        float new_y = y + v * std::sin(new_yaw) * static_cast<float>(dt);
        float new_dist = std::sqrt(new_x * new_x + new_y * new_y);
        float cur_dist = std::sqrt(x * x + y * y);
        // 3) 缩放因子
        float s = 1.0f;
        if (cur_dist >= max_distance - 1e-6f) {
            s = 0.0f; reached_limit = true;
        } else if (new_dist > max_distance) {
            float step = new_dist - cur_dist;
            s = (step > 1e-6f) ? std::max(0.0f, std::min(1.0f, (max_distance - cur_dist) / step)) : 0.0f;
        } else if (soft_zone > 0.0f && new_dist > soft_start) {
            float ratio = std::max(0.0f, std::min(1.0f, (new_dist - soft_start) / soft_zone));
            s = 1.0f - 0.5f * ratio;  // 软减速到 50%
        }
        v *= s; w *= s;
        // 4) 用实际下发速度更新里程计
        yaw += w * static_cast<float>(dt);
        x   += v * std::cos(yaw) * static_cast<float>(dt);
        y   += v * std::sin(yaw) * static_cast<float>(dt);
        return {v, w};
    }
};

// 把 cur 朝 tgt 移动，单步不超过 max_step（微分插值）
static float approach(float cur, float tgt, float max_step) {
    float d = tgt - cur;
    if (d > max_step) d = max_step;
    if (d < -max_step) d = -max_step;
    return cur + d;
}

static std::vector<std::string> split(const std::string& line, char sep) {
    std::vector<std::string> out;
    std::string cell;
    std::istringstream ss(line);
    while (std::getline(ss, cell, sep)) out.push_back(cell);
    return out;
}

static std::vector<Frame> loadFrames(const std::string& path) {
    std::vector<Frame> frames;
    std::ifstream f(path);
    if (!f) {
        std::cerr << "[error] 无法打开数据文件: " << path << std::endl;
        return frames;
    }
    std::string line;
    std::getline(f, line);  // 跳过表头
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::vector<std::string> c = split(line, ',');
        if (c.size() < 16) continue;
        Frame fr;
        fr.t = std::stod(c[0]);
        fr.v = std::stof(c[1]);
        fr.w = std::stof(c[2]);
        fr.lx = std::stof(c[3]);
        fr.ly = std::stof(c[4]);
        fr.lz = std::stof(c[5]);
        fr.lroll = std::stof(c[6]);
        fr.lpitch = std::stof(c[7]);
        fr.lyaw = std::stof(c[8]);
        fr.rx = std::stof(c[9]);
        fr.ry = std::stof(c[10]);
        fr.rz = std::stof(c[11]);
        fr.rroll = std::stof(c[12]);
        fr.rpitch = std::stof(c[13]);
        fr.ryaw = std::stof(c[14]);
        fr.gripper = std::stof(c[15]);
        fr.phase = c.size() > 16 ? std::stoi(c[16]) : 3;
        frames.push_back(fr);
    }
    return frames;
}

// -------------------- 关节级：姿态表 + 序列（新增） --------------------
// 左臂 7 关节：肩俯仰 肩外展 肩旋转 肘 腕yaw 腕pitch 腕roll
static const float kHomeL[7]     = { 0.000f,  0.200f,  0.000f, -0.600f, 0.000f,  0.000f,  0.000f};
static const float kRaiseL[7]    = {-1.829f,  0.090f, -0.112f,  0.625f, 0.015f, -0.024f,  0.447f};
static const float kStraightL[7] = {-1.609f,  0.132f, -0.124f,  0.339f, 0.009f, -0.037f,  0.464f};
static const float kMirror[7]    = { 1.f, -1.f, -1.f, 1.f, -1.f, 1.f, -1.f};  // 右臂镜像

static const int kLeft[7]  = {7, 8, 9, 10, 11, 12, 13};   // MOTOR_LEFT_ARM_1..7
static const int kRight[7] = {15, 16, 17, 18, 19, 20, 21}; // MOTOR_RIGHT_ARM_1..7
static const int kGripL = 14, kGripR = 22;                 // MOTOR_*_ARM_8

struct JointRow {
    double t = 0.0;
    float a[23] = {0};
    std::string phase;
};

static float smoothstep_u(float u) { return u * u * (3.f - 2.f * u); }

// 用官方姿态生成关节序列（与 gen_official.py 完全一致）
static std::vector<JointRow> buildOfficialSeq() {
    const double dt = 1.0 / rate;
    std::vector<JointRow> rows;
    double t = 0.0;
    auto emit = [&](const float l[7], const char* phase, double dur) {
        int n = std::max(1, static_cast<int>(dur * rate));
        for (int i = 0; i < n; ++i) {
            JointRow r;
            r.t = t;
            r.phase = phase;
            for (int k = 0; k < 7; ++k) {
                r.a[kLeft[k]] = l[k];
                r.a[kRight[k]] = l[k] * kMirror[k];
            }
            r.a[kGripL] = gripper_open;
            r.a[kGripR] = gripper_open;
            rows.push_back(r);
            t += dt;
        }
    };
    auto trans = [&](const float a[7], const float b[7], const char* phase, double dur) {
        int n = std::max(1, static_cast<int>(dur * rate));
        for (int i = 0; i < n; ++i) {
            float u = smoothstep_u(static_cast<float>(i) / std::max(1, n - 1));
            JointRow r;
            r.t = t;
            r.phase = phase;
            for (int k = 0; k < 7; ++k) {
                float v = a[k] + (b[k] - a[k]) * u;
                r.a[kLeft[k]] = v;
                r.a[kRight[k]] = v * kMirror[k];
            }
            r.a[kGripL] = gripper_open;
            r.a[kGripR] = gripper_open;
            rows.push_back(r);
            t += dt;
        }
    };
    emit(kHomeL, "rest", 0.5);
    trans(kHomeL, kRaiseL, "raise", 2.5);
    trans(kRaiseL, kStraightL, "extend", 2.0);
    emit(kStraightL, "hold", 10.0);
    trans(kStraightL, kRaiseL, "retract1", 2.0);
    trans(kRaiseL, kHomeL, "retract2", 2.5);
    emit(kHomeL, "rest", 0.5);
    return rows;
}

static std::vector<JointRow> loadJointCsv(const std::string& path) {
    std::vector<JointRow> rows;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[error] 无法打开关节序列: " << path << std::endl;
        return rows;
    }
    std::string line;
    std::getline(f, line);  // 表头
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto c = split(line, ',');
        if (c.size() < 24) continue;
        JointRow r;
        r.t = std::stod(c[0]);
        for (int i = 0; i < 23; ++i) r.a[i] = std::stof(c[1 + i]);
        r.phase = c[24];
        rows.push_back(r);
    }
    return rows;
}

// 关节级下发
static void jointSendOne(H1Robot& robot, int id, float q) {
    Motor_Control c{};
    c.Position = q;
    c.Speed = 0;
    c.Torque = 0;
    c.KP = motor_kp[id];
    c.KD = motor_kd[id];
    robot.setMotorControl_low(static_cast<EtherCAT_Motor_Index>(id), c);
}

static void jointSendArms(H1Robot& robot, const float cur[23]) {
    for (int k = 0; k < 7; ++k) {
        jointSendOne(robot, kLeft[k], cur[kLeft[k]]);
        jointSendOne(robot, kRight[k], cur[kRight[k]]);
    }
}

static bool jointReadMotor(H1Robot& robot, int id, float& out) {
    Motor_Information s{};
    if (!robot.getMotorState(static_cast<EtherCAT_Motor_Index>(id), s)) return false;
    out = s.Position_Actual;
    return true;
}

static bool jointReadArms(H1Robot& robot, float cur[23]) {
    for (int k = 0; k < 7; ++k) {
        if (!jointReadMotor(robot, kLeft[k], cur[kLeft[k]])) return false;
        if (!jointReadMotor(robot, kRight[k], cur[kRight[k]])) return false;
    }
    jointReadMotor(robot, kGripL, cur[kGripL]);
    jointReadMotor(robot, kGripR, cur[kGripR]);
    return true;
}

// LOW_LEVEL 下的底盘：v/w -> 左右轮速度（换算与官方 h1_exec_low.cpp 一致）
static void sendChassisWheels(H1Robot& robot, float v, float w) {
    const float vl = v - w * wheel_half_b;
    const float vr = v + w * wheel_half_b;
    Motor_Control cl{}, cr{};
    cl.Speed = std::max(-wheel_speed_max, std::min(wheel_speed_max, vl / wheel_r));
    cr.Speed = std::max(-wheel_speed_max, std::min(wheel_speed_max, vr / wheel_r));
    cl.Position = 0; cr.Position = 0;
    cl.Torque = 0;   cr.Torque = 0;
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, cl);
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, cr);
}

static void stopChassisWheels(H1Robot& robot) {
    Motor_Control z{};
    z.Speed = 0;
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, z);
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, z);
}

static void doLift(H1Robot& robot, float target) {
    Motor_Control c{};
    c.Position = std::max(0.f, std::min(0.80f, target));
    c.Speed = 0;
    c.Torque = 0;
    robot.setWaist_low(EtherCAT_Motor_Index::MOTOR_LIFT, c);
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LIFT, c);
}

// =========================== 动捕回放（核心） ===========================
void MocapPlayback(H1Robot& robot, const std::vector<Frame>& frames,
                   const std::vector<JointRow>& jointSeq, bool armJoint,
                   bool useChassis, float liftZ, float speed) {
    std::cout << "Start MocapPlayback, frames = " << frames.size()
              << ", speed = " << speed
              << ", arm = " << (armJoint ? "joint(底层关节角)" : "pose(高层末端位姿)")
              << ", range = " << max_range << "m"
              << ", vx<=" << max_vx << " m/s, wz<=" << max_wz << " rad/s"
              << std::endl;

    // 当前值（用于微分插值）
    float cur_v = 0.0f, cur_w = 0.0f;
    float cur_l[6] = {0, 0, 0, 0, 0, 0};
    float cur_r[6] = {0, 0, 0, 0, 0, 0};
    float cur_j[23] = {0};
    int sleep_us = static_cast<int>(time_sleep / (speed > 0 ? speed : 1.0f));

    ChassisRange chassis;
    std::string last_joint_phase;
    std::size_t ji = 0;  // 关节序列按时间推进的指针

    if (armJoint) {
        // 起点 = 机器人真实关节角，避免跳变
        if (!jointReadArms(robot, cur_j)) {
            std::cerr << "[warn] 读关节角失败，按 0 起步（请确认周围无障碍）\n";
        }
        std::cout << "  实际起点 L1=" << cur_j[7] << " L2=" << cur_j[8]
                  << " L4=" << cur_j[10] << " | R1=" << cur_j[15]
                  << " R2=" << cur_j[16] << " R4=" << cur_j[18] << "\n";
    }

    for (size_t i = 0; i < frames.size() && g_run; ++i) {
        const Frame& fr = frames[i];

        // ---------------- 底盘：先经 2m 围栏限速，再微分插值下发 ----------------
        auto [v_safe, w_safe] = chassis.limit(fr.v, fr.w, fr.t);
        cur_v = approach(cur_v, v_safe, max_dv);
        cur_w = approach(cur_w, w_safe, max_dw);
        if (useChassis) {
            if (armJoint) sendChassisWheels(robot, cur_v, cur_w);
            else          robot.setChassis_high(cur_v, cur_w);
        }

        if (armJoint) {
            // ---------------- 关节级手臂（取当前时刻的关节帧） ----------------
            while (ji + 1 < jointSeq.size() && jointSeq[ji].t < fr.t) ++ji;
            const JointRow& jr = jointSeq.empty() ? JointRow() : jointSeq[ji];
            for (int k = 0; k < 7; ++k) {
                cur_j[kLeft[k]] = approach(cur_j[kLeft[k]], jr.a[kLeft[k]], max_dangle);
                cur_j[kRight[k]] = approach(cur_j[kRight[k]], jr.a[kRight[k]], max_dangle);
            }
            jointSendArms(robot, cur_j);
            if (liftZ > 0.01f) doLift(robot, liftZ);

            // 夹爪：沿用动捕 CSV 的 gripper 列，映射到底层量程 0~1.0
            Motor_Control g{};
            g.Position = gripper_open + (gripper_close_low - gripper_open) * fr.gripper;
            g.Speed = 0; g.Torque = 0;
            g.KP = motor_kp[kGripL]; g.KD = motor_kd[kGripL];
            robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8, g);
            robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8, g);

            if (!jointSeq.empty() && jr.phase != last_joint_phase) {
                std::cout << "  [arm] t=" << fr.t << "s 阶段=" << jr.phase
                          << " 肘=" << jr.a[10] << " 肩俯仰=" << jr.a[7]
                          << " 肩外展=" << jr.a[8] << "\n";
                last_joint_phase = jr.phase;
            }
        } else {
            // ---------------- 原来的高层末端位姿手臂（未改动） ----------------
            cur_l[0] = approach(cur_l[0], fr.lx, max_dpos);
            cur_l[1] = approach(cur_l[1], fr.ly, max_dpos);
            cur_l[2] = approach(cur_l[2], fr.lz, max_dpos);
            cur_l[3] = approach(cur_l[3], fr.lroll, max_dang);
            cur_l[4] = approach(cur_l[4], fr.lpitch, max_dang);
            cur_l[5] = approach(cur_l[5], fr.lyaw, max_dang);

            ArmPose lpose;
            lpose.x = cur_l[0]; lpose.y = cur_l[1]; lpose.z = cur_l[2];
            lpose.roll = cur_l[3]; lpose.pitch = cur_l[4]; lpose.yaw = cur_l[5];
            robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(lpose));

            cur_r[0] = approach(cur_r[0], fr.rx, max_dpos);
            cur_r[1] = approach(cur_r[1], fr.ry, max_dpos);
            cur_r[2] = approach(cur_r[2], fr.rz, max_dpos);
            cur_r[3] = approach(cur_r[3], fr.rroll, max_dang);
            cur_r[4] = approach(cur_r[4], fr.rpitch, max_dang);
            cur_r[5] = approach(cur_r[5], fr.ryaw, max_dang);

            ArmPose rpose;
            rpose.x = cur_r[0]; rpose.y = cur_r[1]; rpose.z = cur_r[2];
            rpose.roll = cur_r[3]; rpose.pitch = cur_r[4]; rpose.yaw = cur_r[5];
            robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(rpose));

            Motor_Control grip;
            grip.Position = gripper_open + (gripper_close - gripper_open) * fr.gripper;
            grip.Speed = 0;
            grip.Torque = 0;
            robot.setGripper_high(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8, grip);
            robot.setGripper_high(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8, grip);
        }

        if (i % rate == 0) {
            std::cout << "  t=" << fr.t << "s phase=" << fr.phase
                      << " v_req=" << fr.v << " w_req=" << fr.w
                      << " v_safe=" << v_safe << " w_safe=" << w_safe
                      << " dist=" << std::sqrt(chassis.x * chassis.x + chassis.y * chassis.y)
                      << " grip=" << fr.gripper
                      << (chassis.reached_limit ? " [RANGE_LIMIT]" : "")
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }

    // 收尾：底盘停止
    if (useChassis) {
        if (armJoint) { stopChassisWheels(robot); stopChassisWheels(robot); }
        else          robot.setChassis_high(0, 0);
    }
    std::cout << "MocapPlayback finished. final distance = "
              << std::sqrt(chassis.x * chassis.x + chassis.y * chassis.y)
              << " m" << std::endl;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, on_sigint);
    std::cout << std::fixed << std::setprecision(6);

    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <动捕.csv> [--arm joint|pose] "
                     "[--joint-csv x.csv] [--speed 1.0] [--ip IP] [--lift 0.5] "
                     "[--no-chassis]\n";
        return 1;
    }
    std::string data_path = argv[1];
    float speed = 1.0f;
    bool armJoint = true;      // 默认关节级（修复后的手臂控制）
    bool useChassis = true;
    float liftZ = 0.0f;
    std::string joint_csv, ip;

    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (a == "--arm") { auto v = next(); armJoint = (v != "pose"); }
        else if (a == "--joint-csv") joint_csv = next();
        else if (a == "--speed") speed = std::stof(next());
        else if (a == "--ip") ip = next();
        else if (a == "--lift") liftZ = std::stof(next());
        else if (a == "--no-chassis") useChassis = false;
        else if (i == 2) speed = std::stof(a);
    }
    if (speed <= 0.0f) speed = 1.0f;

    // 加载动捕指令
    std::vector<Frame> frames = loadFrames(data_path);
    if (frames.empty()) {
        std::cerr << "[error] 数据为空或读取失败: " << data_path << std::endl;
        std::cerr << "       请先用: python run.py --clip <片段> [--lock-chassis] "
                     "--export-h1 " << data_path << std::endl;
        return 1;
    }
    std::cout << "Loaded " << frames.size() << " frames from " << data_path << std::endl;

    // 关节序列：外部 CSV 或内置官方姿态
    std::vector<JointRow> jointSeq;
    if (armJoint) {
        jointSeq = joint_csv.empty() ? buildOfficialSeq() : loadJointCsv(joint_csv);
        if (jointSeq.empty()) {
            std::cerr << "[error] 关节序列为空（" << joint_csv << "）\n";
            return 1;
        }
        std::cout << "Joint seq " << jointSeq.size() << " 行, 时长 "
                  << jointSeq.back().t << "s ("
                  << (joint_csv.empty() ? "内置官方姿态" : joint_csv) << ")\n";
        if (frames.size() > jointSeq.size()) {
            std::cout << "[info] 动捕比关节序列长：关节序列走完后手臂保持在最后一帧（收臂位）\n";
        }
    }

    // 实例化机器人
    // H1Robot robot("192.168.3.43");   // 无线连接，IP 不固定
    // H1Robot robot("172.31.200.1");   // 有线连接，IP 固定
    H1Robot robot(ip.empty() ? "" : ip.c_str());
    if (!ip.empty()) std::cout << "连接机器人 " << ip << " ...\n";

    // 创建并启动电机状态监控
    MotorStateMonitor monitor(robot);
    monitor.start();

    if (!robot.robot_connect()) {  // 尝试连接机器人
        std::cerr << "Robot connection failed." << std::endl;
        return 1;
    }

    // 选择控制模式：关节级必须底层；末端位姿沿用原来的高层
    if (armJoint) {
        if (!robot.switchControlMode(MotorControlMode::LOW_LEVEL)) {
            std::cerr << "Failed to switch to LOW_LEVEL mode." << std::endl;
            return 1;
        }
        std::cout << "模式 LOW_LEVEL（关节级手臂控制）\n";
    } else {
        if (!robot.switchControlMode(MotorControlMode::HIGH_LEVEL)) {
            std::cerr << "Failed to switch to HIGH_LEVEL mode." << std::endl;
            return 1;
        }
        std::cout << "模式 HIGH_LEVEL（原末端位姿手臂控制）\n";
    }

    if (liftZ > 0.01f && armJoint) {
        std::cout << "升降到 " << liftZ << "（避免手臂擦地/撞底盘）\n";
        doLift(robot, liftZ);
        std::this_thread::sleep_for(std::chrono::milliseconds(6000));
    }

    if (robot.robot_init()) {  // 尝试初始化机器人（动作）
        std::cout << "Robot initialized successfully." << std::endl;
    } else {
        std::cerr << "Robot initialization failed." << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // ============ 回放动捕动作 ============
    MocapPlayback(robot, frames, jointSeq, armJoint, useChassis, liftZ, speed);

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    if (robot.robot_deinit()) {  // 反初始化
        std::cout << "Robot deinitialized successfully." << std::endl;
    } else {
        std::cerr << "Robot deinitialization failed." << std::endl;
        return 1;
    }

    return 0;
}
