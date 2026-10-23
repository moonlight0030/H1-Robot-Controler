# h1_mocap_arm_player（动捕回放 + 修复后的手臂控制）

在 `h1_mocap_player.cpp` 基础上**只换手臂控制**，底盘、2m 围栏、夹爪、
连接/初始化/回放流程全部保持原样。

- 底盘：`ChassisRange` 2m 围栏、限速 `vx≤0.20 / wz≤0.50`、微分插值 `max_dv/max_dw`
  —— **代码原封不动**
- 夹爪：仍用动捕 CSV 的 `gripper` 列
- 手臂：**默认换成关节级**（底层 `setMotorControl_low` + 官方绝对关节角）

## 整合了哪三个文件

| 来源 | 用在哪 |
| --- | --- |
| `arm_joint_player.cpp` | 关节级下发：KP/KD 表、读真实起点、微分插值、`armPose` 部分 |
| `gen_official.py` | 官方姿态序列 HOME / RAISE / STRAIGHT（肘 0.339 = 19.4° 伸直） |
| `arm_motion_player.cpp` | 高层末端位姿路径（保留为 `--arm pose`，对照用） |

## 手臂两种模式

| 模式 | 控制方式 | 控制模式 | 适用场景 |
| --- | --- | --- | --- |
| `--arm joint`（默认） | `setMotorControl_low` 绝对关节角 | LOW_LEVEL | **修复版**，手臂能抬起并伸直 |
| `--arm pose` | `setArm_high` 末端位姿 | HIGH_LEVEL | 原来的方式，完整保留作对照 |

关节姿态表（左臂；右臂按 `[1,-1,-1,1,-1,1,-1]` 镜像）：

| 姿态 | 肩俯仰 | 肩外展 | 肩旋转 | 肘 | 腕yaw | 腕pitch | 腕roll |
| --- | --- | --- | --- | --- | --- | --- | --- |
| HOME 收臂 | 0.000 | +0.200 | 0.000 | −0.600 | 0 | 0 | 0 |
| RAISE 抬起 | −1.829 | +0.090 | −0.112 | +0.625 | 0.015 | −0.024 | 0.447 |
| STRAIGHT 伸出 | −1.609 | +0.132 | −0.124 | **+0.339（19.4°）** | 0.009 | −0.037 | 0.464 |

时间轴：`rest 0.5 → raise 2.5 → extend 2.0 → hold 10.0 → retract1 2.0 → retract2 2.5 → rest 0.5`
（夹爪全程张开 = 0）

## 为什么底盘末端接口改了（逻辑没改）

关节级控制**必须 LOW_LEVEL 模式**，该模式下 `setChassis_high` 不可用，
所以底盘改为直接给左右轮速度，换算与官方 `h1_exec_low.cpp` 完全一致：

```
r = 0.10 m, 半轴距 = 0.20 m
v_l = v - w*0.2,  v_r = v + w*0.2
Speed = v / r,  限幅 ±2.0 rad/s
```

2m 围栏、限速、微分插值、里程计全部沿用原代码，`v/w` 语义不变。
`--arm pose` 时仍走 `HIGH_LEVEL + setChassis_high`，和原来一模一样。

## 用法

```bash
./build.sh /path/to/h1_sdk

# 默认：底盘按动捕走 + 手臂关节级抬起伸直（内置官方姿态）
./h1_mocap_arm_player h1_player/data/mission_demo.csv

# 用生成的关节序列文件
python gen_official.py                       # -> data/arm_joint_official.csv
./h1_mocap_arm_player mission.csv --joint-csv data/arm_joint_official.csv

# 原来的末端位姿方式（对照）
./h1_mocap_arm_player mission.csv --arm pose

# 其它
./h1_mocap_arm_player mission.csv --speed 0.5      # 慢放
./h1_mocap_arm_player mission.csv --ip 10.42.0.1   # SDK 在用户主机（无线）
./h1_mocap_arm_player mission.csv --lift 0.5       # 先升升降，避免擦地
./h1_mocap_arm_player mission.csv --no-chassis     # 只动手臂，底盘不动
```

动捕 CSV 格式不变：`t,v,w,lx,ly,lz,lroll,lpitch,lyaw,rx,ry,rz,rroll,rpitch,ryaw,gripper,phase`
（由 `mocap_robot_controller` 的 `--export-h1` 导出）。

关节序列走完后手臂保持在最后一帧（收臂位），底盘继续跑完。

## 微调手臂姿态

```bash
python gen_official.py --elbow 0.20        # 肘越小越直（官方 0.339 = 19.4°）
python gen_official.py --pitch -1.83       # 肩俯仰：负得越多抬得越高
python gen_official.py --roll 0.09         # 肩外展：离身体的角度
python gen_official.py --hold 10           # 保持时长
```

生成的 CSV 经 `--joint-csv` 播放，观察哪个最合适再定。

## 目录

```
h1_mocap_arm_player/
├── h1_mocap_arm_player.cpp   # 整合后的播放器
├── gen_official.py           # 官方关节姿态序列生成器
├── CMakeLists.txt / build.sh
└── data/                     # 生成的关节序列 CSV
```

## 上机注意

1. 第一次先用 `--no-chassis` 只跑手臂，确认方向/姿态对。
2. 关节模式启动时会**读真实关节角**再平滑过渡，不会跳变。
3. Ctrl+C 立即停轮并退出。
4. 长时间举臂建议改 `GRAVITY_COMPENSATION_LEVEL`（代码里 `switchControlMode` 处改）。
