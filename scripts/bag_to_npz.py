#!/usr/bin/env python3
"""balance_node で記録した ROSBAG を、模倣学習用の npz ファイルに変換する。

使い方: python3 bag_to_npz.py <ROSBAGのフォルダ> <出力フォルダ> [pitch_target] [kp kd kv kx]
"""
import sys
import os
import glob
import math
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

FALL_ANGLE = 0.6                # これ以上傾いたら転倒とみなし、そこで打ち切る [rad]
MATCH_WINDOW_NS = 20_000_000    # IMU を受け取ってから 20 ms 以内の cmd_vel を対応する指令とみなす


def is_teleop(cmd):
    # teleop_node が出す値は {-1, 0, 1} だけなので、それで見分ける
    if cmd.angular.z != 0.0:
        return True
    return cmd.linear.x in (-1.0, 0.0, 1.0)


def read_bag(path):
    # ROSBAG から /imu と /cmd_vel を読み出す（teleop の指令は除く）
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id='sqlite3'),
                ConverterOptions('cdr', 'cdr'))
    imus, cmds = [], []
    n_teleop = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/imu':
            imus.append((t, deserialize_message(data, Imu)))
        elif topic == '/cmd_vel':
            m = deserialize_message(data, Twist)
            if is_teleop(m):
                n_teleop += 1
            else:
                cmds.append((t, m.linear.x))
    return imus, cmds, n_teleop


def convert(path, pitch_target):
    imus, cmds, n_teleop = read_bag(path)
    cmd_t = np.array([c[0] for c in cmds], dtype=np.int64)
    cmd_v = np.array([c[1] for c in cmds])

    rows = []
    v_prev, x_prev, t_prev = 0.0, 0.0, None
    last_k = -1
    for t_bag, m in imus:
        # クォータニオン → ピッチ角（本当の直立が 0 になるよう pitch_target を引く）
        q = m.orientation
        s = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, s))) - pitch_target
        rate = m.angular_velocity.y
        t_sim = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

        # この IMU を受けて balance_node が出した指令を探す（同じ指令を二重に使わない）
        k = int(np.searchsorted(cmd_t, t_bag))
        if k <= last_k:
            k = last_k + 1
        if k >= len(cmd_t) or cmd_t[k] - t_bag > MATCH_WINDOW_NS:
            continue
        last_k = k
        v = float(cmd_v[k])

        # 転倒したら、そのエピソードはここで打ち切る
        if abs(pitch) > FALL_ANGLE:
            break

        # 観測: [ピッチ角, ピッチ角速度, 直前の速度指令, 移動量]、行動: 速度指令
        # x は balance_node と同じく「速度指令 × dt」を積み上げて再現する
        dt = 0.0 if t_prev is None else t_sim - t_prev
        rows.append([t_sim, dt, pitch, rate, v_prev, x_prev, v])
        x_prev = x_prev + v * dt
        v_prev = v
        t_prev = t_sim

    return np.array(rows), n_teleop


def estimate_x0(arr, kp, kd, kv, kx):
    # 記録開始時に balance_node が内部に持っていた x の値を、PID の式から逆算する
    dt, pitch, rate, v_prev, x_rec, v = (arr[:, i] for i in range(1, 7))
    mask = dt > 1e-4
    mask[0] = False   # 最初のサンプルは v_prev が不明なので使わない
    a = (v[mask] - v_prev[mask]) / dt[mask]
    x_true = (a - kp * pitch[mask] - kd * rate[mask] - kv * v_prev[mask]) / kx
    return float(np.median(x_true - x_rec[mask]))


def check(arr, kp, kd, kv, kx):
    # 再現した観測を PID の式に入れ直し、記録された指令とどれだけずれるかを調べる（検算）
    dt, pitch, rate, v_prev, x_prev, v = (arr[:, i] for i in range(1, 7))
    a = kp * pitch + kd * rate + kv * v_prev + kx * x_prev
    v_pred = v_prev + a * dt
    err = np.abs(v_pred[1:] - v[1:])
    return float(np.max(err)), float(np.mean(err))


def main():
    bags_dir, out_dir = sys.argv[1], sys.argv[2]
    pitch_target = float(sys.argv[3]) if len(sys.argv) > 3 else -0.18
    gains = [float(g) for g in sys.argv[4:8]] if len(sys.argv) >= 8 else None
    os.makedirs(out_dir, exist_ok=True)

    # ROSBAG のフォルダを1つずつ変換する
    for path in sorted(glob.glob(os.path.join(bags_dir, '*'))):
        if not os.path.isdir(path):
            continue
        name = os.path.basename(path)
        arr, n_teleop = convert(path, pitch_target)
        x0 = 0.0
        if gains and len(arr) > 0:
            # 記録開始時の x を逆算して、移動量の列を補正する
            x0 = estimate_x0(arr, *gains)
            arr[:, 5] += x0
        if len(arr) == 0:
            print(f'{name}: データなし')
            continue
        np.savez(os.path.join(out_dir, name + '.npz'),
                 t=arr[:, 0], dt=arr[:, 1],
                 state=arr[:, 2:6], action=arr[:, 6:7])
        msg = (f'{name}: {len(arr)} サンプル, {arr[-1, 0] - arr[0, 0]:.1f} 秒, '
               f'除いたteleop指令={n_teleop}, '
               f'開始時のx={x0:+.2f} m, '
               f'ピッチ角の範囲=[{math.degrees(arr[:, 2].min()):+.1f}, '
               f'{math.degrees(arr[:, 2].max()):+.1f}] 度')
        if gains:
            emax, emean = check(arr, *gains)
            msg += f', PID検算の誤差 最大={emax:.2e} 平均={emean:.2e}'
        print(msg)


if __name__ == '__main__':
    main()

