#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import struct
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, UInt32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


SOF0 = 0xAA
SOF1 = 0x55
EOF0 = 0x0D
EOF1 = 0x0A

MSG_ID_ENCODER = 0x11
FRAME_LEN = 34  # 固定编码器上行帧长度


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    h = 0.5 * yaw
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(h)
    q.w = math.cos(h)
    return q


class EncoderRxNode(Node):
    def __init__(self):
        super().__init__('encoder_rx_node')

        # 串口参数
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        # 底盘参数（与STM32保持一致）
        self.declare_parameter('wheel_radius', 0.0375)   # m
        self.declare_parameter('track_width', 0.1806)    # m
        self.declare_parameter('wheel_base', 0.1719)     # m

        # 编码器参数（与STM32保持一致）
        self.declare_parameter('motor_encoder_ppr', 13)
        self.declare_parameter('motor_gear_ratio', 30)
        self.declare_parameter('encoder_quadrature_multiplier', 4)
        self.declare_parameter('encoder_on_motor_shaft', True)

        # 编码器方向（与ENCODER_DIR_A/B/C/D一致）
        self.declare_parameter('encoder_dir_a', 1)
        self.declare_parameter('encoder_dir_b', -1)
        self.declare_parameter('encoder_dir_c', 1)
        self.declare_parameter('encoder_dir_d', -1)

        # frame id
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # logger打印频率
        self.declare_parameter('print_hz', 5.0)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        ppr = float(self.get_parameter('motor_encoder_ppr').value)
        gear = float(self.get_parameter('motor_gear_ratio').value)
        quad = float(self.get_parameter('encoder_quadrature_multiplier').value)
        on_motor = bool(self.get_parameter('encoder_on_motor_shaft').value)

        self.dir_map = [
            int(self.get_parameter('encoder_dir_a').value),
            int(self.get_parameter('encoder_dir_b').value),
            int(self.get_parameter('encoder_dir_c').value),
            int(self.get_parameter('encoder_dir_d').value),
        ]

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        print_hz = float(self.get_parameter('print_hz').value)
        if print_hz <= 1e-3:
            print_hz = 5.0

        gear_factor = gear if on_motor else 1.0
        self.pulses_per_rev = ppr * quad * gear_factor
        if self.pulses_per_rev <= 0.0:
            self.pulses_per_rev = 1.0

        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.get_logger().info(
            f'Listening {port}@{baud}, pulses_per_rev={self.pulses_per_rev:.1f}'
        )

        # 发布原始数据
        self.pub_delta = self.create_publisher(Int32MultiArray, '/encoder_delta', 10)
        self.pub_meta = self.create_publisher(UInt32MultiArray, '/encoder_meta', 10)

        # 发布 odom
        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)

        # 里程状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx_body = 0.0
        self.vy_body = 0.0
        self.omega = 0.0

        self.last_seq = None
        self.last_t_ms = None

        # 串口轮询 + 里程打印
        self.poll_timer = self.create_timer(0.002, self.poll)  # 500Hz
        self.print_timer = self.create_timer(1.0 / print_hz, self.print_odom)

    def print_odom(self):
        self.get_logger().info(
            f'odom x={self.x:.3f} m, y={self.y:.3f} m, th={self.theta:.3f} rad, '
            f'vx={self.vx_body:.3f} m/s, vy={self.vy_body:.3f} m/s, w={self.omega:.3f} rad/s'
        )

    def poll(self):
        if self.ser.in_waiting <= 0:
            return

        while self.ser.in_waiting >= 2:
            b0 = self.ser.read(1)
            if not b0:
                return
            if b0[0] != SOF0:
                continue

            b1 = self.ser.read(1)
            if not b1:
                return
            if b1[0] != SOF1:
                continue

            rest = self.ser.read(FRAME_LEN - 2)
            if len(rest) != FRAME_LEN - 2:
                return

            frame = bytes([SOF0, SOF1]) + rest
            self.parse_frame(frame)
            break

    def parse_frame(self, frame: bytes):
        if len(frame) != FRAME_LEN:
            return
        if frame[32] != EOF0 or frame[33] != EOF1:
            return
        if frame[2] != 27:
            return
        if frame[3] != MSG_ID_ENCODER:
            return

        calc_crc = crc16_ccitt_false(frame[2:30])  # Byte2..Byte29
        rx_crc = frame[30] | (frame[31] << 8)
        if calc_crc != rx_crc:
            return

        seq, t_ms, dt_us, da, db, dc, dd = struct.unpack('<HIIiiii', frame[4:30])

        # 丢帧判断（重复帧忽略）
        if self.last_seq is not None:
            diff = (seq - self.last_seq) & 0xFFFF
            if diff == 0:
                return
            if diff > 1:
                self.get_logger().warn(f'seq lost: {self.last_seq} -> {seq}')
        self.last_seq = seq
        self.last_t_ms = t_ms

        # 发布原始增量
        msg_d = Int32MultiArray()
        msg_d.data = [da, db, dc, dd]
        self.pub_delta.publish(msg_d)

        msg_m = UInt32MultiArray()
        msg_m.data = [seq, t_ms, dt_us]
        self.pub_meta.publish(msg_m)

        # 方向修正
        dcounts = [
            da * self.dir_map[0],
            db * self.dir_map[1],
            dc * self.dir_map[2],
            dd * self.dir_map[3],
        ]

        dt = float(dt_us) * 1e-6
        if dt <= 0.0:
            return
        if dt > 0.1:
            dt = 0.1

        # 每轮线速度
        wheel_circ = 2.0 * math.pi * self.wheel_radius
        v_wheels = []
        for d in dcounts:
            rev = float(d) / self.pulses_per_rev
            v_wheels.append((rev * wheel_circ) / dt)

        v_fl, v_fr, v_rl, v_rr = v_wheels

        L = 0.5 * (self.track_width + self.wheel_base)
        if L <= 1e-6:
            L = 1e-6

        # 与STM32一致的反解
        self.vx_body = (v_fl + v_fr + v_rl + v_rr) * 0.25
        self.vy_body = (v_fl - v_fr - v_rl + v_rr) * 0.25
        self.omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * L)

        # 中点积分
        theta_mid = self.theta + 0.5 * self.omega * dt
        c = math.cos(theta_mid)
        s = math.sin(theta_mid)

        vx_world = self.vx_body * c - self.vy_body * s
        vy_world = self.vx_body * s + self.vy_body * c

        self.x += vx_world * dt
        self.y += vy_world * dt
        self.theta += self.omega * dt

        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta < -math.pi:
            self.theta += 2.0 * math.pi

        self.publish_odom()

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)

        odom.twist.twist.linear.x = self.vx_body
        odom.twist.twist.linear.y = self.vy_body
        odom.twist.twist.angular.z = self.omega

        self.pub_odom.publish(odom)


def main():
    rclpy.init()
    node = EncoderRxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()