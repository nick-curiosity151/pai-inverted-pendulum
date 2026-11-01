import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


class BalanceNode(Node):
    def __init__(self):
        super().__init__('balance_node')
        # 傾き・角速度のゲイン
        self.declare_parameter('kp', 20.0)
        self.declare_parameter('kd', 3.0)
        # 速度・位置のゲイン（正の値）
        self.declare_parameter('kv', 0.5)
        self.declare_parameter('kx', 0.2)
        self.declare_parameter('max_vel', 2.0)
        self.declare_parameter('fall_angle', 0.6)
        self.declare_parameter('sign', 1.0)
        self.declare_parameter('pitch_target', -0.05)   # 本当の直立のときのIMUの値 [rad]

        self.v = 0.0        # 速度指令（積分した値）
        self.x = 0.0        # 位置（速度指令を積分した推定値）
        self.prev_t = None
        self.count = 0

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.get_logger().info('balance_node started')

    def imu_cb(self, msg):
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        kv = self.get_parameter('kv').value
        kx = self.get_parameter('kx').value
        max_vel = self.get_parameter('max_vel').value
        fall_angle = self.get_parameter('fall_angle').value
        sign = self.get_parameter('sign').value

        # クォータニオン → ピッチ角
        q = msg.orientation
        s = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, s)))
        pitch = pitch - self.get_parameter('pitch_target').value
        pitch_rate = msg.angular_velocity.y

        # dt（シミュレーション時刻）
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_t is None:
            self.prev_t = t
            return
        dt = t - self.prev_t
        self.prev_t = t
        if dt <= 0.0 or dt > 0.1:
            return

        if abs(pitch) > fall_angle:
            # 転倒したら止めて、状態をリセット
            self.v = 0.0
            self.x = 0.0
        else:
            a = kp * pitch + kd * pitch_rate + kv * self.v + kx * self.x
            self.v += a * dt
            self.v = max(-max_vel, min(max_vel, self.v))
            self.x += self.v * dt

        cmd = Twist()
        cmd.linear.x = sign * self.v
        self.pub.publish(cmd)

        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(
                f'pitch={math.degrees(pitch):+6.2f} deg  '
                f'rate={pitch_rate:+6.2f}  v={self.v:+5.2f} m/s  x={self.x:+6.2f} m')


def main():
    rclpy.init()
    node = BalanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

