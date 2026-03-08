#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import termios
import tty
import select
import time
import struct
import argparse
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial


def clamp_i16(v: float) -> int:
    iv = int(round(v))
    if iv > 32767:
        return 32767
    if iv < -32768:
        return -32768
    return iv


def clamp_i32(v: float) -> int:
    iv = int(round(v))
    if iv > 2147483647:
        return 2147483647
    if iv < -2147483648:
        return -2147483648
    return iv


def pack_cmd_frame(vx_mps: float, vy_mps: float, wz_radps: float) -> bytes:
    # 下行控制帧: AA 55 LEN(6) vx vy wz XOR 0D 0A
    vx_mm_s = clamp_i16(vx_mps * 1000.0)
    vy_mm_s = clamp_i16(vy_mps * 1000.0)
    wz_mrad_s = clamp_i16(wz_radps * 1000.0)

    payload = struct.pack('<Bhhh', 6, vx_mm_s, vy_mm_s, wz_mrad_s)  # Byte2..Byte8
    checksum = 0
    for b in payload:
        checksum ^= b

    return bytes([0xAA, 0x55]) + payload + bytes([checksum, 0x0D, 0x0A])


def pack_host_odom_frame(x_m: float, y_m: float, theta_rad: float, flags: int = 1) -> bytes:
    # 回传帧: AA 55 LEN(14) ID(0x21) X_mm(int32) Y_mm(int32) THETA_mdeg(int32) FLAGS(uint8) XOR 0D 0A
    msg_id = 0x21
    x_mm = clamp_i32(x_m * 1000.0)
    y_mm = clamp_i32(y_m * 1000.0)
    theta_mdeg = clamp_i32(theta_rad * 180000.0 / math.pi)

    payload = struct.pack('<BiiiB', msg_id, x_mm, y_mm, theta_mdeg, (flags & 0xFF))
    length = 14

    frame_wo_tail = bytes([0xAA, 0x55, length]) + payload
    checksum = 0
    for b in frame_wo_tail[2:]:  # XOR(Byte2..Byte16)
        checksum ^= b

    return frame_wo_tail + bytes([checksum, 0x0D, 0x0A])


class TestControlBridge(Node):
    def __init__(self, port: str, baud: int, hz: float):
        super().__init__('test_control')

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)

        self.speed_linear = 0.20
        self.speed_angular = 0.80

        self.key_release_timeout = 0.12
        self.last_key_time = 0.0
        self.active_key = None

        self.manual_mode = True

        self.auto_cmd = Twist()
        self.auto_last_time = 0.0
        self.auto_timeout = 0.5

        # 定时动作状态
        self.timed_active = False
        self.timed_end_time = 0.0
        self.timed_vx = 0.0
        self.timed_vy = 0.0
        self.timed_wz = 0.0

        # 回传里程（来自 /odom）
        self.host_x = 0.0
        self.host_y = 0.0
        self.host_theta = 0.0
        self.host_odom_ok = False

        # 自动模式输入
        self.sub_auto = self.create_subscription(Twist, '/cmd_vel', self.cb_auto, 10)
        # 订阅里程
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, 10)

        # 控制发送
        self.timer = self.create_timer(1.0 / hz, self.loop)
        # 回传发送（20Hz）
        self.feedback_timer = self.create_timer(0.05, self.send_host_odom_feedback)

        # 键盘设置
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.print_help()
        self.get_logger().info(f'Bridge started: {port} @ {baud}, {hz}Hz | mode=MANUAL')

    def destroy_node(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    def print_help(self):
        print("\n=== Key Bindings ===")
        print("W/S : 前进/后退")
        print("A/D : 左移/右移")
        print("Q/E : 左旋/右旋")
        print("SPACE: 刹车")
        print("T    : 切换 MANUAL/AUTO")
        print("X    : 退出")
        print("Timed Action:")
        print("1: 前进1s  2: 前进3s  3: 前进5s")
        print("4: 左移2s  5: 右移2s")
        print("6: 左旋2s  7: 右旋2s")
        print("====================\n")

    def cb_auto(self, msg: Twist):
        self.auto_cmd = msg
        self.auto_last_time = time.monotonic()

    def cb_odom(self, msg: Odometry):
        # 从 /odom 取位姿
        self.host_x = msg.pose.pose.position.x
        self.host_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.host_theta = math.atan2(siny_cosp, cosy_cosp)

        self.host_odom_ok = True

    def read_key_nonblock(self):
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

    def manual_cmd_from_key(self, key: str):
        vx = 0.0
        vy = 0.0
        wz = 0.0

        if key == 'w':
            vx = self.speed_linear
        elif key == 's':
            vx = -self.speed_linear
        elif key == 'a':
            vy = self.speed_linear
        elif key == 'd':
            vy = -self.speed_linear
        elif key == 'q':
            wz = self.speed_angular
        elif key == 'e':
            wz = -self.speed_angular

        return vx, vy, wz

    def start_timed_action(self, vx: float, vy: float, wz: float, duration_s: float):
        now = time.monotonic()
        self.timed_active = True
        self.timed_end_time = now + duration_s
        self.timed_vx = vx
        self.timed_vy = vy
        self.timed_wz = wz
        self.active_key = None
        self.get_logger().info(f'timed action: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}, t={duration_s:.1f}s')

    def stop_all(self):
        self.timed_active = False
        self.active_key = None
        self.ser.write(pack_cmd_frame(0.0, 0.0, 0.0))

    def send_host_odom_feedback(self):
        # 若还没收到 /odom，可保持发0并置 flag=0
        flags = 1 if self.host_odom_ok else 0
        frame = pack_host_odom_frame(self.host_x, self.host_y, self.host_theta, flags=flags)
        self.ser.write(frame)

    def loop(self):
        now = time.monotonic()
        key = self.read_key_nonblock()

        if key is not None:
            key = key.lower()

            if key in ('w', 'a', 's', 'd', 'q', 'e'):
                self.active_key = key
                self.last_key_time = now
                self.timed_active = False

            elif key == '1':
                self.start_timed_action(self.speed_linear, 0.0, 0.0, 1.0)
            elif key == '2':
                self.start_timed_action(self.speed_linear, 0.0, 0.0, 3.0)
            elif key == '3':
                self.start_timed_action(self.speed_linear, 0.0, 0.0, 5.0)
            elif key == '4':
                self.start_timed_action(0.0, self.speed_linear, 0.0, 2.0)
            elif key == '5':
                self.start_timed_action(0.0, -self.speed_linear, 0.0, 2.0)
            elif key == '6':
                self.start_timed_action(0.0, 0.0, self.speed_angular, 2.0)
            elif key == '7':
                self.start_timed_action(0.0, 0.0, -self.speed_angular, 2.0)

            elif key == ' ':
                self.stop_all()

            elif key == 't':
                self.manual_mode = not self.manual_mode
                mode = 'MANUAL' if self.manual_mode else 'AUTO'
                self.get_logger().info(f'mode -> {mode}')
                self.active_key = None
                self.timed_active = False

            elif key == 'x':
                self.stop_all()
                raise KeyboardInterrupt

        # MANUAL + 定时动作
        if self.manual_mode and self.timed_active:
            if now <= self.timed_end_time:
                vx, vy, wz = self.timed_vx, self.timed_vy, self.timed_wz
            else:
                self.timed_active = False
                vx, vy, wz = 0.0, 0.0, 0.0

        # MANUAL + 按住走松手停
        elif self.manual_mode:
            if self.active_key is not None and (now - self.last_key_time) > self.key_release_timeout:
                self.active_key = None

            if self.active_key is None:
                vx, vy, wz = 0.0, 0.0, 0.0
            else:
                vx, vy, wz = self.manual_cmd_from_key(self.active_key)

        # AUTO
        else:
            if (now - self.auto_last_time) > self.auto_timeout:
                vx, vy, wz = 0.0, 0.0, 0.0
            else:
                vx = self.auto_cmd.linear.x
                vy = self.auto_cmd.linear.y
                wz = self.auto_cmd.angular.z

        self.ser.write(pack_cmd_frame(vx, vy, wz))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--hz', type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = TestControlBridge(args.port, args.baud, args.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()