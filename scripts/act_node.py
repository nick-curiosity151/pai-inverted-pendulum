#!/usr/bin/env python3
"""学習した ACT で倒立振子を制御する ROS 2 ノード。

pixi 環境で実行する。
使い方: pixi run --no-install python /workspace/scripts/act_node.py [モデルのフォルダ]
"""
import sys
import math
import time

import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from lerobot.common.policies.act.modeling_act import ACTPolicy

POLICY_PATH = sys.argv[1] if len(sys.argv) > 1 else "/workspace/models/pend_balance_act"
PITCH_TARGET = -0.18     # 本当の直立のときの IMU の値（記録時と同じ）
CONTROL_PERIOD = 0.02    # 推論の周期 [s]（学習データと同じ 50 Hz）
FALL_ANGLE = 0.6         # これ以上傾いたら転倒とみなして止める [rad]
MAX_VEL = 2.0            # 速度指令の上限 [m/s]


class ActNode(Node):
    def __init__(self):
        super().__init__('act_node')
        torch.set_num_threads(2)
        self.policy = ACTPolicy.from_pretrained(POLICY_PATH)
        self.policy.eval()
        self.policy.reset()

        self.v_cmd = 0.0        # 今出している速度指令
        self.v_prev = 0.0       # 直前の推論で出した速度指令（観測に使う）
        self.x = 0.0            # 移動量（速度指令を積分して求める。学習データと同じ方法）
        self.prev_t = None
        self.last_ctrl_t = None
        self.fallen = False
        self.count = 0

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.get_logger().info(f'ACT モデルを読み込みました: {POLICY_PATH}')

    def imu_cb(self, msg):
        # ピッチ角（本当の直立が 0 になるよう補正）と角速度
        q = msg.orientation
        s = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, s))) - PITCH_TARGET
        rate = msg.angular_velocity.y

        # シミュレーション時刻
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_t is None:
            self.prev_t = t
            self.last_ctrl_t = t - CONTROL_PERIOD   # 最初の IMU ですぐ推論する
        dt = t - self.prev_t
        self.prev_t = t
        if dt < 0.0 or dt > 0.1:
            dt = 0.0

        # 移動量は、出している速度指令を積分して求める（IMU ごと）
        self.x += self.v_cmd * dt

        if abs(pitch) > FALL_ANGLE:
            if not self.fallen:
                self.get_logger().warn(f'転倒しました (pitch={math.degrees(pitch):+.1f} 度)')
                self.fallen = True
            self.v_cmd = 0.0
        elif t - self.last_ctrl_t >= CONTROL_PERIOD - 1e-6:
            # 50 Hz ごとに ACT で推論する（間の IMU では前回の指令を出し続ける）
            self.last_ctrl_t = t
            state = torch.tensor([[pitch, rate, self.v_prev, self.x]], dtype=torch.float32)
            t0 = time.perf_counter()
            with torch.inference_mode():
                action = self.policy.select_action({
                    "observation.state": state,
                    "observation.environment_state": state.clone(),
                })
            infer_ms = (time.perf_counter() - t0) * 1000.0
            v = max(-MAX_VEL, min(MAX_VEL, float(action[0, 0])))
            self.v_prev = v
            self.v_cmd = v

            self.count += 1
            if self.count % 25 == 0:    # 0.5秒ごとに表示
                self.get_logger().info(
                    f'pitch={math.degrees(pitch):+6.2f} deg  rate={rate:+6.2f}  '
                    f'v={v:+5.2f}  x={self.x:+6.2f}  推論={infer_ms:5.1f} ms')

        cmd = Twist()
        cmd.linear.x = self.v_cmd
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = ActNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


