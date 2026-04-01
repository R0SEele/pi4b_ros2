#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32MultiArray, UInt32MultiArray
import serial
import struct
import math
import time
import sys
import select
import termios
import tty


SOF0 = 0xAA
SOF1 = 0x55
EOF0 = 0x0D
EOF1 = 0x0A

MSG_ID_ENCODER = 0x11
FRAME_LEN = 46  # MCU 端 telemetry 帧长度已扩展到 46 字节（包含 IMU）

# 工具函数
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
    vx_mm_s = clamp_i16(vx_mps * 1000.0)
    vy_mm_s = clamp_i16(vy_mps * 1000.0)
    wz_mrad_s = clamp_i16(wz_radps * 1000.0)
    payload = struct.pack('<Bhhh', 6, vx_mm_s, vy_mm_s, wz_mrad_s)
    checksum = 0
    for b in payload:
        checksum ^= b
    return bytes([0xAA, 0x55]) + payload + bytes([checksum, 0x0D, 0x0A])

def pack_host_odom_frame(x_m: float, y_m: float, theta_rad: float, flags: int = 1) -> bytes:
    msg_id = 0x21
    x_mm = clamp_i32(x_m * 1000.0)
    y_mm = clamp_i32(y_m * 1000.0)
    theta_mdeg = clamp_i32(theta_rad * 180000.0 / math.pi)
    payload = struct.pack('<BiiiB', msg_id, x_mm, y_mm, theta_mdeg, (flags & 0xFF))
    length = 14
    frame_wo_tail = bytes([0xAA, 0x55, length]) + payload
    checksum = 0
    for b in frame_wo_tail[2:]:
        checksum ^= b
    return frame_wo_tail + bytes([checksum, 0x0D, 0x0A])

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

def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    q = Quaternion()
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class CarControllerNode(Node):
    def __init__(self):
        super().__init__('car_controller_node')
        from sensor_msgs.msg import Imu
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        # 参数声明
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('send_odom_feedback', False)
        self.declare_parameter('enable_encoder_odom', True)
        self.declare_parameter('print_hz', 5.0)
        self.declare_parameter('enable_keyboard_control', True)
        self.declare_parameter('keyboard_linear_speed', 0.3)
        self.declare_parameter('keyboard_angular_speed', 0.5)

        self.declare_parameter('wheel_radius', 0.0375)
        self.declare_parameter('track_width', 0.1806)
        self.declare_parameter('wheel_base', 0.1719)
        self.declare_parameter('motor_encoder_ppr', 13)
        self.declare_parameter('motor_gear_ratio', 30)
        self.declare_parameter('encoder_quadrature_multiplier', 4)
        self.declare_parameter('encoder_on_motor_shaft', True)
        self.declare_parameter('encoder_dir_a', 1)
        self.declare_parameter('encoder_dir_b', -1)
        self.declare_parameter('encoder_dir_c', 1)
        self.declare_parameter('encoder_dir_d', -1)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        # 获取参数
        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud = self.get_parameter('serial_baud').get_parameter_value().integer_value
        self.send_odom_feedback_enabled = bool(self.get_parameter('send_odom_feedback').value)
        self.enable_encoder_odom = bool(self.get_parameter('enable_encoder_odom').value)
        self.enable_keyboard_control = bool(self.get_parameter('enable_keyboard_control').value)
        self.keyboard_linear_speed = float(self.get_parameter('keyboard_linear_speed').value)
        if self.keyboard_linear_speed <= 0.0:
            self.keyboard_linear_speed = 0.1
        self.keyboard_angular_speed = float(self.get_parameter('keyboard_angular_speed').value)
        if self.keyboard_angular_speed <= 0.0:
            self.keyboard_angular_speed = 0.8

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
        
        # 小车速度
        self.speed_linear = 0.2
        self.speed_angular = 0.8
        
        # 串口初始化
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.02)
            self.get_logger().info(f"串口已连接：{self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"串口连接失败：{e}")
            sys.exit(1)
        
        # 状态变量
        self.host_x = 0.0
        self.host_y = 0.0
        self.host_theta = 0.0
        self.host_odom_ok = False
        self.vx_body = 0.0
        self.vy_body = 0.0
        self.omega = 0.0
        self.last_seq = None
        self.last_t_ms = None
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.last_roll = 0.0
        self.last_pitch = 0.0
        self.keyboard_fd = None
        self.keyboard_old_termios = None
        self.keyboard_ready = False
        
        # 订阅话题
        self.sub_auto = self.create_subscription(Twist, '/cmd_vel', self.cb_auto, 10)

        self.pub_delta = None
        self.pub_meta = None
        self.pub_odom = None
        self.poll_timer = None
        self.print_timer = None
        if self.enable_encoder_odom:
            self.pub_delta = self.create_publisher(Int32MultiArray, '/encoder_delta', 10)
            self.pub_meta = self.create_publisher(UInt32MultiArray, '/encoder_meta', 10)
            self.pub_odom = self.create_publisher(Odometry, '/wheel_odom', 20)
            self.poll_timer = self.create_timer(0.002, self.poll)
            self.print_timer = self.create_timer(1.0 / print_hz, self.print_odom)
        
        # 定时器
        self.timer = self.create_timer(0.05, self.loop)
        self.feedback_timer = None
        if self.send_odom_feedback_enabled and self.enable_encoder_odom:
            self.feedback_timer = self.create_timer(0.05, self.send_odom_feedback)

        if self.enable_keyboard_control:
            self.init_keyboard_input()
        
        self.get_logger().info("小车控制节点启动成功")
        self.get_logger().info("当前为独立运行模式：仅保留ROS2通信、键盘控制与里程计发布")
        if self.send_odom_feedback_enabled:
            self.get_logger().info("已启用里程计回传到底盘")
        else:
            self.get_logger().info("未启用里程计回传，仅在树莓派侧使用/打印里程计")
        if self.enable_encoder_odom:
            self.get_logger().info(f"已在控制节点内启用编码器里程计解析: {self.port}@{self.baud}")
        else:
            self.get_logger().warn("未启用编码器里程计解析，将不发布/打印里程计")
        if self.enable_keyboard_control:
            self.get_logger().info(
                f"已启用键盘控制: W前进 S后退 A左移 D右移 Q左旋 E右旋, X或空格停止, "
                f"线速度={self.keyboard_linear_speed:.3f}m/s, 角速度={self.keyboard_angular_speed:.3f}rad/s"
            )

    def init_keyboard_input(self):
        if not sys.stdin.isatty():
            self.get_logger().warn("标准输入不是TTY，无法启用键盘控制")
            self.enable_keyboard_control = False
            return
        try:
            self.keyboard_fd = sys.stdin.fileno()
            self.keyboard_old_termios = termios.tcgetattr(self.keyboard_fd)
            tty.setcbreak(self.keyboard_fd)
            self.keyboard_ready = True
        except Exception as e:
            self.get_logger().warn(f"键盘初始化失败，已禁用键盘控制: {e}")
            self.enable_keyboard_control = False
            self.keyboard_ready = False

    def restore_keyboard_input(self):
        if self.keyboard_ready and self.keyboard_fd is not None and self.keyboard_old_termios is not None:
            try:
                termios.tcsetattr(self.keyboard_fd, termios.TCSADRAIN, self.keyboard_old_termios)
            except Exception:
                pass
        self.keyboard_ready = False

    def read_key_nonblocking(self):
        if not self.keyboard_ready:
            return None
        try:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not readable:
                return None
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                # 尝试清掉方向键等转义序列的剩余字节
                readable2, _, _ = select.select([sys.stdin], [], [], 0.0)
                while readable2:
                    _ = sys.stdin.read(1)
                    readable2, _, _ = select.select([sys.stdin], [], [], 0.0)
                return None
            return ch
        except Exception:
            return None

    def handle_keyboard_control(self, now: float):
        key = self.read_key_nonblocking()
        if key is not None:
            k = key.lower()
            linear_speed = self.keyboard_linear_speed
            angular_speed = self.keyboard_angular_speed
            if k == 'w':
                if self.current_vx == linear_speed and self.current_vy == 0.0 and self.current_wz == 0.0:
                    self.stop()
                else:
                    self.current_vx, self.current_vy, self.current_wz = linear_speed, 0.0, 0.0
                    self.ser.write(pack_cmd_frame(self.current_vx, self.current_vy, self.current_wz))
            elif k == 's':
                if self.current_vx == -linear_speed and self.current_vy == 0.0 and self.current_wz == 0.0:
                    self.stop()
                else:
                    self.current_vx, self.current_vy, self.current_wz = -linear_speed, 0.0, 0.0
                    self.ser.write(pack_cmd_frame(self.current_vx, self.current_vy, self.current_wz))
            elif k == 'a':
                if self.current_vx == 0.0 and self.current_vy == linear_speed and self.current_wz == 0.0:
                    self.stop()
                else:
                    self.current_vx, self.current_vy, self.current_wz = 0.0, linear_speed, 0.0
                    self.ser.write(pack_cmd_frame(self.current_vx, self.current_vy, self.current_wz))
            elif k == 'd':
                if self.current_vx == 0.0 and self.current_vy == -linear_speed and self.current_wz == 0.0:
                    self.stop()
                else:
                    self.current_vx, self.current_vy, self.current_wz = 0.0, -linear_speed, 0.0
                    self.ser.write(pack_cmd_frame(self.current_vx, self.current_vy, self.current_wz))
            elif k == 'q':
                if self.current_vx == 0.0 and self.current_vy == 0.0 and self.current_wz == angular_speed:
                    self.stop()
                else:
                    self.current_vx, self.current_vy, self.current_wz = 0.0, 0.0, angular_speed
                    self.ser.write(pack_cmd_frame(self.current_vx, self.current_vy, self.current_wz))
            elif k == 'e':
                if self.current_vx == 0.0 and self.current_vy == 0.0 and self.current_wz == -angular_speed:
                    self.stop()
                else:
                    self.current_vx, self.current_vy, self.current_wz = 0.0, 0.0, -angular_speed
                    self.ser.write(pack_cmd_frame(self.current_vx, self.current_vy, self.current_wz))
            elif k in ('x', ' '):
                self.stop()

        # 某些底盘带指令超时保护，手动模式下持续重发当前速度可避免自动刹停。
        if self.current_vx != 0.0 or self.current_vy != 0.0 or self.current_wz != 0.0:
            self.ser.write(pack_cmd_frame(self.current_vx, self.current_vy, self.current_wz))

    def print_odom(self):
        if not self.host_odom_ok:
            return
        self.get_logger().info(
            f'odom x={self.host_x:.3f} m, y={self.host_y:.3f} m, th={self.host_theta:.3f} rad, '
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
            self.parse_encoder_frame(frame)
            break

    def parse_encoder_frame(self, frame: bytes):
        # 新增IMU消息发布
        from sensor_msgs.msg import Imu
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        # 欧拉角转四元数
        imu_msg.orientation = rpy_to_quaternion(self.last_roll, self.last_pitch, self.host_theta)
        # 可补充角速度/加速度等（如有数据源）
        self.pub_imu.publish(imu_msg)
        if len(frame) != FRAME_LEN:
            return
        if frame[32] != EOF0 or frame[33] != EOF1:
            return
        if frame[2] != 39:
            return
        if frame[3] != MSG_ID_ENCODER:
            return

        calc_crc = crc16_ccitt_false(frame[2:42])
        rx_crc = frame[42] | (frame[43] << 8)
        if calc_crc != rx_crc:
            return

        seq, t_ms, dt_us, da, db, dc, dd = struct.unpack('<HIIiiii', frame[4:30])

        # 解出 IMU 姿态（mdeg -> deg）
        roll_mdeg, pitch_mdeg, yaw_mdeg = struct.unpack('<iii', frame[30:42])
        self.last_roll = roll_mdeg / 1000.0
        self.last_pitch = pitch_mdeg / 1000.0
        self.host_theta = yaw_mdeg / 1000.0
        self.get_logger().info(
            f"[IMU] roll={self.last_roll:.2f} pitch={self.last_pitch:.2f} yaw={self.host_theta:.2f}")

        if self.last_seq is not None:
            diff = (seq - self.last_seq) & 0xFFFF
            if diff == 0:
                return
            if diff > 1:
                self.get_logger().warn(f'seq lost: {self.last_seq} -> {seq}')
        self.last_seq = seq
        self.last_t_ms = t_ms

        if self.pub_delta is not None:
            msg_d = Int32MultiArray()
            msg_d.data = [da, db, dc, dd]
            self.pub_delta.publish(msg_d)

        if self.pub_meta is not None:
            msg_m = UInt32MultiArray()
            msg_m.data = [seq, t_ms, dt_us]
            self.pub_meta.publish(msg_m)

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

        wheel_circ = 2.0 * math.pi * self.wheel_radius
        v_wheels = []
        for d in dcounts:
            rev = float(d) / self.pulses_per_rev
            v_wheels.append((rev * wheel_circ) / dt)

        v_fl, v_fr, v_rl, v_rr = v_wheels
        L = 0.5 * (self.track_width + self.wheel_base)
        if L <= 1e-6:
            L = 1e-6

        self.vx_body = (v_fl + v_fr + v_rl + v_rr) * 0.25
        self.vy_body = (v_fl - v_fr - v_rl + v_rr) * 0.25
        self.omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * L)

        # 用 IMU yaw 作为航向，避免轮式里程计累积漂移
        theta_mid = self.host_theta + 0.5 * self.omega * dt
        c = math.cos(theta_mid)
        s = math.sin(theta_mid)
        vx_world = self.vx_body * c - self.vy_body * s
        vy_world = self.vx_body * s + self.vy_body * c

        self.host_x += vx_world * dt
        self.host_y += vy_world * dt

        self.host_odom_ok = True
        self.publish_odom()

    def publish_odom(self):
        if self.pub_odom is None:
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.host_x
        odom.pose.pose.position.y = self.host_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = rpy_to_quaternion(self.last_roll, self.last_pitch, self.host_theta)

        odom.twist.twist.linear.x = self.vx_body
        odom.twist.twist.linear.y = self.vy_body
        odom.twist.twist.angular.z = self.omega

        # Fill reasonable non-zero covariance to keep EKF numerically stable.
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36

        # pose: x, y, yaw
        odom.pose.covariance[0] = 2.5e-3
        odom.pose.covariance[7] = 2.5e-3
        odom.pose.covariance[35] = 4.0e-4
        # large uncertainty for unused z/roll/pitch in 2D
        odom.pose.covariance[14] = 1.0e6
        odom.pose.covariance[21] = 1.0e6
        odom.pose.covariance[28] = 1.0e6

        # twist: vx, vy, yaw_rate
        odom.twist.covariance[0] = 2.5e-3
        odom.twist.covariance[7] = 2.5e-3
        odom.twist.covariance[35] = 2.5e-3
        odom.twist.covariance[14] = 1.0e6
        odom.twist.covariance[21] = 1.0e6
        odom.twist.covariance[28] = 1.0e6

        self.pub_odom.publish(odom)

        # 打印调试：确认里程计数据确实发布了
        self.get_logger().info(
            f"[odom] x={self.host_x:.3f} y={self.host_y:.3f} th={self.host_theta:.3f} "
            f"vx={self.vx_body:.3f} vy={self.vy_body:.3f} w={self.omega:.3f} "
            f"roll={self.last_roll:.2f} pitch={self.last_pitch:.2f}"
        )

    # 自动模式回调
    def cb_auto(self, msg: Twist):
        if self.enable_keyboard_control:
            return
        self.current_vx = msg.linear.x
        self.current_vy = msg.linear.y
        self.current_wz = msg.angular.z
        self.ser.write(pack_cmd_frame(self.current_vx, self.current_vy, self.current_wz))

    # 发送里程计反馈
    def send_odom_feedback(self):
        flags = 1 if self.host_odom_ok else 0
        frame = pack_host_odom_frame(self.host_x, self.host_y, self.host_theta, flags)
        self.ser.write(frame)

    # 停止动作
    def stop(self):
        self.current_vx = self.current_vy = self.current_wz = 0.0
        self.ser.write(pack_cmd_frame(0.0, 0.0, 0.0))
        self.get_logger().info("小车已停止")

    # 主循环
    def loop(self):
        now = time.monotonic()

        if self.enable_keyboard_control:
            self.handle_keyboard_control(now)

        # 某些底盘有速度指令看门狗，非键盘模式也持续重发当前速度。
        if (not self.enable_keyboard_control) and (self.current_vx != 0.0 or self.current_vy != 0.0 or self.current_wz != 0.0):
            self.ser.write(pack_cmd_frame(self.current_vx, self.current_vy, self.current_wz))

    # 销毁节点
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.stop()
            self.ser.close()
        self.restore_keyboard_input()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CarControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()