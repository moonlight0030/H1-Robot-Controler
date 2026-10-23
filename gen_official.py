#!/usr/bin/env python3
"""生成官方关节姿态序列 CSV（给 h1_mocap_arm_player 的 --joint-csv 用）。

姿态取自已跑通的官方姿态表 h1_p0_s4_03_left_arm_poses.cpp：
    HOME     收臂  [ 0.000,  0.200,  0.000, -0.600, 0.000,  0.000,  0.000]
    RAISE    抬起  [-1.829,  0.090, -0.112,  0.625, 0.015, -0.024,  0.447]
    STRAIGHT 伸出  [-1.609,  0.132, -0.124,  0.339, 0.009, -0.037,  0.464]
                    肩俯仰   肩外展  肩旋转    肘    腕yaw 腕pitch 腕roll

右臂镜像 [1, -1, -1, 1, -1, 1, -1]。夹爪 0 = 张开。

用法：
    python gen_official.py                         # -> data/arm_joint_official.csv
    python gen_official.py --out my.csv --hold 10 --rate 500
    python gen_official.py --elbow 0.20            # 微调肘角（越小越直）
"""

import argparse
import csv
import os

HOME = [0.000, 0.200, 0.000, -0.600, 0.000, 0.000, 0.000]
RAISE = [-1.829, 0.090, -0.112, 0.625, 0.015, -0.024, 0.447]
STRAIGHT = [-1.609, 0.132, -0.124, 0.339, 0.009, -0.037, 0.464]

MIRROR = [1, -1, -1, 1, -1, 1, -1]


def mirror(v):
    return [v[i] * MIRROR[i] for i in range(7)]


def lerp(a, b, u):
    return [a[i] + (b[i] - a[i]) * u for i in range(7)]


def smooth(u):
    return u * u * (3 - 2 * u)


def write_csv(path, rate=500.0, hold=10.0, straight=None):
    straight = straight or STRAIGHT
    dt = 1.0 / rate
    t = 0.0
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + [f"m{i:02d}" for i in range(23)] + ["phase"])

        def emit(l, r, grip, phase, dur):
            nonlocal t
            for _ in range(int(dur * rate)):
                row = [round(t, 4)] + [0.0] * 23 + [phase]
                for i in range(7):
                    row[1 + 7 + i] = round(l[i], 5)
                    row[1 + 15 + i] = round(r[i], 5)
                row[1 + 14] = round(grip, 5)
                row[1 + 22] = round(grip, 5)
                w.writerow(row)
                t += dt

        def transition(a_l, a_r, b_l, b_r, grip, phase, dur):
            nonlocal t
            n = max(1, int(dur * rate))
            for k in range(n):
                u = smooth(k / max(1, n - 1))
                l = lerp(a_l, b_l, u)
                r = lerp(a_r, b_r, u)
                row = [round(t, 4)] + [0.0] * 23 + [phase]
                for i in range(7):
                    row[1 + 7 + i] = round(l[i], 5)
                    row[1 + 15 + i] = round(r[i], 5)
                row[1 + 14] = round(grip, 5)
                row[1 + 22] = round(grip, 5)
                w.writerow(row)
                t += dt

        hl, hr = HOME, mirror(HOME)
        rl, rr = RAISE, mirror(RAISE)
        sl, sr = straight, mirror(straight)

        emit(hl, hr, 0.0, "rest", 0.5)
        transition(hl, hr, rl, rr, 0.0, "raise", 2.5)
        transition(rl, rr, sl, sr, 0.0, "extend", 2.0)
        emit(sl, sr, 0.0, "hold", hold)
        transition(sl, sr, rl, rr, 0.0, "retract1", 2.0)
        transition(rl, rr, hl, hr, 0.0, "retract2", 2.5)
        emit(hl, hr, 0.0, "rest", 0.5)
    return t


def main():
    p = argparse.ArgumentParser(description="生成官方关节姿态序列 CSV")
    p.add_argument("--out", default="data/arm_joint_official.csv")
    p.add_argument("--rate", type=float, default=500.0)
    p.add_argument("--hold", type=float, default=10.0, help="保持时长 (s)")
    p.add_argument("--elbow", type=float, help="伸出姿态的肘角 (rad)，默认 0.339；越小越直")
    p.add_argument("--pitch", type=float, help="伸出姿态的肩俯仰 (rad)，默认 -1.609")
    p.add_argument("--roll", type=float, help="伸出姿态的肩外展 (rad)，默认 0.132")
    a = p.parse_args()

    straight = list(STRAIGHT)
    if a.elbow is not None:
        straight[3] = a.elbow
    if a.pitch is not None:
        straight[0] = a.pitch
    if a.roll is not None:
        straight[1] = a.roll

    dur = write_csv(a.out, a.rate, a.hold, straight)
    print(f"已生成 {a.out}  时长 {dur:.2f}s @ {a.rate:g}Hz")
    print(f"  伸出姿态 肩俯仰={straight[0]:+.3f} 肩外展={straight[1]:+.3f} "
          f"肘={straight[3]:+.3f} ({straight[3] * 57.2958:+.1f}°)")


if __name__ == "__main__":
    main()
