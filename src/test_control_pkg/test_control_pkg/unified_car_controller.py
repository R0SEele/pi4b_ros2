import rclpy
  from rclpy.node import Node
  from sensor_msgs.msg import Imu
  from nav_msgs.msg import Odometry
  from geometry_msgs.msg import Twist, Quaternion, TransformStamped
  import serial
  import struct
  import threading
  import time
  import numpy as np
  from tf2_ros import TransformBroadcaster
  from collections import deque
  import crcmod

  class UnifiedCarController(Node):
      def __init__(self):
          super().__init__('unified_car_controller')

          # ==================== ROS2配置 ====================
          # 发布器
          self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
          self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
          self.tf_broadcaster = TransformBroadcaster(self)

          # 订阅器（接收速度命令）
          self.cmd_vel_sub = self.create_subscription(
              Twist,
              '/cmd_vel',
              self.cmd_vel_callback,
              10
          )

          # ==================== 串口配置 ====================
          # 根据实际连接修改
          self.serial_port = '/dev/ttyAMA0'  # 树莓派GPIO串口
          # self.serial_port = '/dev/ttyUSB0'  # USB转串口
          self.baudrate = 115200
          self.ser = None

          # ==================== 协议定义 ====================
          self.FRAME_HEAD = bytes([0xAA, 0x55])
          self.FRAME_TAIL = bytes([0x0D, 0x0A])

          # 消息ID定义
          self.MSG_ID_ENCODER = 0x11
          self.MSG_ID_IMU = 0x12
          self.MSG_ID_VELOCITY_CMD = 0x01  # 发送给STM32的控制命令

          # ==================== 数据缓冲区 ====================
          self.recv_buffer = bytearray()
          self.send_buffer = deque()

          # ==================== 状态变量 ====================
          self.last_encoder_time = None
          self.last_imu_time = None
          self.last_cmd_time = None

          # 编码器里程计状态
          self.encoder_counts = [0, 0, 0, 0]  # A, B, C, D
          self.encoder_seq = 0

          # IMU状态
          self.imu_data = {
              'gyro': [0.0, 0.0, 0.0],
              'accel': [0.0, 0.0, 0.0],
              'euler': [0.0, 0.0, 0.0],  # roll, pitch, yaw (弧度)
              'temperature': 0.0,
              'seq': 0
          }

          # 融合里程计状态
          self.position = np.array([0.0, 0.0, 0.0])  # x, y, theta (m, m, rad)
          self.velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, omega (m/s, m/s, rad/s)

          # CRC校验函数
          self.crc16_func = crcmod.predefined.mkCrcFun('crc-ccitt-false')

          # ==================== 初始化 ====================
          self.init_serial()

          # 启动线程
          self.running = True
          self.recv_thread = threading.Thread(target=self.receive_loop)
          self.send_thread = threading.Thread(target=self.send_loop)
          self.process_thread = threading.Thread(target=self.process_loop)

          self.recv_thread.start()
          self.send_thread.start()
          self.process_thread.start()

          self.get_logger().info('统一控制器节点已启动')

      # ==================== 串口相关方法 ====================
      def init_serial(self):
          """初始化串口"""
          try:
              self.ser = serial.Serial(
                  port=self.serial_port,
                  baudrate=self.baudrate,
                  bytesize=serial.EIGHTBITS,
                  parity=serial.PARITY_NONE,
                  stopbits=serial.STOPBITS_ONE,
                  timeout=0.01,  # 短超时，避免阻塞
                  write_timeout=0.1
              )
              # 清空缓冲区
              self.ser.reset_input_buffer()
              self.ser.reset_output_buffer()
              self.get_logger().info(f'串口 {self.serial_port} 打开成功')
          except Exception as e:
              self.get_logger().error(f'串口打开失败: {e}')
              self.ser = None

      def receive_loop(self):
          """接收线程主循环"""
          while self.running and self.ser and self.ser.is_open:
              try:
                  # 非阻塞读取
                  if self.ser.in_waiting > 0:
                      data = self.ser.read(self.ser.in_waiting)
                      self.recv_buffer.extend(data)

                      # 解析接收到的数据
                      self.parse_received_data()
              except serial.SerialException as e:
                  self.get_logger().error(f'串口读取错误: {e}')
                  time.sleep(0.1)
                  try:
                      self.ser.close()
                      time.sleep(1)
                      self.init_serial()
                  except:
                      pass
              except Exception as e:
                  self.get_logger().error(f'接收线程错误: {e}')
                  time.sleep(0.01)

      def send_loop(self):
          """发送线程主循环"""
          while self.running:
              try:
                  if self.ser and self.ser.is_open and self.send_buffer:
                      # 从队列获取待发送数据
                      data = self.send_buffer.popleft()
                      self.ser.write(data)
                      self.ser.flush()
                  else:
                      time.sleep(0.001)  # 短暂休眠，避免CPU占用过高
              except Exception as e:
                  self.get_logger().error(f'发送线程错误: {e}')
                  time.sleep(0.01)

      # ==================== 数据解析方法 ====================
      def parse_received_data(self):
          """解析接收缓冲区中的数据"""
          while len(self.recv_buffer) >= 4:  # 至少需要帧头+长度
              # 查找帧头
              try:
                  head_idx = self.recv_buffer.find(self.FRAME_HEAD)
                  if head_idx < 0:
                      # 没有找到帧头，清空缓冲区
                      self.recv_buffer.clear()
                      break

                  if head_idx > 0:
                      # 丢弃帧头前的无效数据
                      self.recv_buffer = self.recv_buffer[head_idx:]

                  # 检查是否有足够的数据获取长度字段
                  if len(self.recv_buffer) < 3:
                      break

                  payload_len = self.recv_buffer[2]
                  frame_len = payload_len + 6  # 完整帧长度

                  if len(self.recv_buffer) < frame_len:
                      break  # 等待完整帧

                  # 提取完整帧
                  frame = bytes(self.recv_buffer[:frame_len])

                  # 验证帧尾
                  if frame[-2:] != self.FRAME_TAIL:
                      self.get_logger().warn('帧尾验证失败')
                      self.recv_buffer.pop(0)  # 丢弃一个字节继续查找
                      continue

                  # 处理帧
                  self.process_frame(frame)

                  # 从缓冲区移除已处理的帧
                  self.recv_buffer = self.recv_buffer[frame_len:]

              except Exception as e:
                  self.get_logger().error(f'解析数据错误: {e}')
                  if len(self.recv_buffer) > 0:
                      self.recv_buffer.pop(0)  # 丢弃错误字节

      def process_frame(self, frame):
          """处理完整数据帧"""
          # 提取消息ID
          msg_id = frame[3]

          # 根据消息ID分发处理
          if msg_id == self.MSG_ID_ENCODER:
              self.process_encoder_frame(frame)
          elif msg_id == self.MSG_ID_IMU:
              self.process_imu_frame(frame)
          else:
              self.get_logger().warn(f'未知消息ID: 0x{msg_id:02X}')

      def process_encoder_frame(self, frame):
          """处理编码器数据帧 (ID=0x11)"""
          try:
              # 验证长度
              if frame[2] != 27:  # ROS2_TELEM_LEN
                  self.get_logger().warn(f'编码器帧长度错误: {frame[2]}')
                  return

              # 解析数据（小端字节序）
              seq, = struct.unpack('<H', frame[4:6])
              timestamp_ms, = struct.unpack('<I', frame[6:10])
              dt_us, = struct.unpack('<I', frame[10:14])

              dcount_a, = struct.unpack('<i', frame[14:18])
              dcount_b, = struct.unpack('<i', frame[18:22])
              dcount_c, = struct.unpack('<i', frame[22:26])
              dcount_d, = struct.unpack('<i', frame[26:30])

              # CRC校验（可选）
              crc_received, = struct.unpack('<H', frame[30:32])
              crc_calculated = self.crc16_func(frame[2:30])

              if crc_received != crc_calculated:
                  self.get_logger().warn('编码器帧CRC校验失败')
                  return

              # 更新编码器数据
              self.encoder_counts = [dcount_a, dcount_b, dcount_c, dcount_d]
              self.encoder_seq = seq

              # 记录接收时间
              self.last_encoder_time = time.time()

              # 发布编码器里程计（需要根据轮子参数计算）
              # self.publish_encoder_odometry(timestamp_ms, dt_us)

              self.get_logger().debug(f'编码器数据: seq={seq}, dt={dt_us}us, counts={self.encoder_counts}')

          except Exception as e:
              self.get_logger().error(f'解析编码器帧错误: {e}')

      def process_imu_frame(self, frame):
          """处理IMU数据帧 (ID=0x12)"""
          try:
              # 验证长度
              if frame[2] != 32:  # ROS2_IMU_PAYLOAD_LEN
                  self.get_logger().warn(f'IMU帧长度错误: {frame[2]}')
                  return

              # 解析数据
              seq, = struct.unpack('<H', frame[4:6])
              timestamp_ms, = struct.unpack('<I', frame[6:10])

              # 陀螺仪原始数据
              gyro_x, = struct.unpack('<h', frame[10:12])
              gyro_y, = struct.unpack('<h', frame[12:14])
              gyro_z, = struct.unpack('<h', frame[14:16])

              # 加速度计原始数据
              accel_x, = struct.unpack('<h', frame[16:18])
              accel_y, = struct.unpack('<h', frame[18:20])
              accel_z, = struct.unpack('<h', frame[20:22])

              # 温度数据
              temperature, = struct.unpack('<h', frame[22:24])

              # 欧拉角
              roll, = struct.unpack('<f', frame[24:28])
              pitch, = struct.unpack('<f', frame[28:32])
              yaw, = struct.unpack('<f', frame[32:36])

              # CRC校验（可选）
              crc_received, = struct.unpack('<H', frame[36:38])
              crc_calculated = self.crc16_func(frame[2:36])

              if crc_received != crc_calculated:
                  self.get_logger().warn('IMU帧CRC校验失败')
                  return

              # 更新IMU数据
              self.imu_data.update({
                  'gyro': [gyro_x, gyro_y, gyro_z],
                  'accel': [accel_x, accel_y, accel_z],
                  'euler': [roll, pitch, yaw],
                  'temperature': temperature,
                  'seq': seq
              })

              # 记录接收时间
              self.last_imu_time = time.time()

              # 发布IMU数据
              self.publish_imu_data(timestamp_ms)

              # 融合里程计
              self.fuse_odometry(timestamp_ms)

              self.get_logger().debug(f'IMU数据: seq={seq}, roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}')

          except Exception as e:
              self.get_logger().error(f'解析IMU帧错误: {e}')

      # ==================== ROS2发布方法 ====================
      def publish_imu_data(self, timestamp_ms):
          """发布ROS2 IMU消息"""
          imu_msg = Imu()
          imu_msg.header.stamp = self.get_clock().now().to_msg()
          imu_msg.header.frame_id = 'imu_link'

          # 角速度转换（需要根据MPU6050量程调整）
          # 假设量程 ±2000°/s, 灵敏度 16.4 LSB/(°/s)
          gyro_scale = 2000.0 / 32768.0 * (np.pi / 180.0)  # rad/s per LSB
          imu_msg.angular_velocity.x = self.imu_data['gyro'][0] * gyro_scale
          imu_msg.angular_velocity.y = self.imu_data['gyro'][1] * gyro_scale
          imu_msg.angular_velocity.z = self.imu_data['gyro'][2] * gyro_scale

          # 线加速度转换（假设量程 ±8g）
          accel_scale = 8.0 * 9.80665 / 32768.0  # m/s² per LSB
          imu_msg.linear_acceleration.x = self.imu_data['accel'][0] * accel_scale
          imu_msg.linear_acceleration.y = self.imu_data['accel'][1] * accel_scale
          imu_msg.linear_acceleration.z = self.imu_data['accel'][2] * accel_scale

          # 姿态四元数（从欧拉角转换）
          roll, pitch, yaw = self.imu_data['euler']
          cy = np.cos(yaw * 0.5)
          sy = np.sin(yaw * 0.5)
          cp = np.cos(pitch * 0.5)
          sp = np.sin(pitch * 0.5)
          cr = np.cos(roll * 0.5)
          sr = np.sin(roll * 0.5)

          imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
          imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
          imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
          imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

          # 发布
          self.imu_pub.publish(imu_msg)

      def fuse_odometry(self, timestamp_ms):
          """
          简单的IMU里程计融合
          实际应用需要：
          1. 结合编码器数据
          2. 使用EKF或互补滤波
          3. 考虑传感器标定
          """
          current_time = timestamp_ms / 1000.0

          # 这里只是示例，实际需要更复杂的融合算法
          # 简单的航向角积分
          gyro_z_rad = self.imu_data['gyro'][2] * 0.001  # 简化转换

          # 更新航向角
          self.position[2] += gyro_z_rad * 0.01  # 假设10ms周期

          # 发布里程计
          self.publish_fused_odometry(current_time)

      def publish_fused_odometry(self, current_time):
          """发布融合后的里程计"""
          odom_msg = Odometry()
          odom_msg.header.stamp = self.get_clock().now().to_msg()
          odom_msg.header.frame_id = 'odom'
          odom_msg.child_frame_id = 'base_link'

          # 位置（这里只是示例，实际需要从编码器计算）
          odom_msg.pose.pose.position.x = self.position[0]
          odom_msg.pose.pose.position.y = self.position[1]
          odom_msg.pose.pose.position.z = 0.0

          # 姿态
          cy = np.cos(self.position[2] * 0.5)
          sy = np.sin(self.position[2] * 0.5)
          odom_msg.pose.pose.orientation.w = cy
          odom_msg.pose.pose.orientation.x = 0.0
          odom_msg.pose.pose.orientation.y = 0.0
          odom_msg.pose.pose.orientation.z = sy

          # 协方差（示例值）
          odom_msg.pose.covariance = [0.01] + [0.0] * 5 + \
                                     [0.01] + [0.0] * 4 + \
                                     [0.01] + [0.0] * 3 + \
                                     [0.01] + [0.0] * 2 + \
                                     [0.01] + [0.0] * 1 + \
                                     [0.01]

          # 速度（示例值）
          odom_msg.twist.twist.linear.x = self.velocity[0]
          odom_msg.twist.twist.linear.y = self.velocity[1]
          odom_msg.twist.twist.angular.z = self.velocity[2]

          self.odom_pub.publish(odom_msg)

          # 发布TF
          self.publish_odom_tf(odom_msg)

      def publish_odom_tf(self, odom_msg):
          """发布TF变换"""
          t = TransformStamped()
          t.header.stamp = odom_msg.header.stamp
          t.header.frame_id = 'odom'
          t.child_frame_id = 'base_link'

          t.transform.translation.x = odom_msg.pose.pose.position.x
          t.transform.translation.y = odom_msg.pose.pose.position.y
          t.transform.translation.z = odom_msg.pose.pose.position.z

          t.transform.rotation = odom_msg.pose.pose.orientation

          self.tf_broadcaster.sendTransform(t)

      # ==================== 控制命令发送 ====================
      def cmd_vel_callback(self, msg):
          """接收ROS2速度命令，发送给STM32"""
          # 提取速度命令
          vx = msg.linear.x  # m/s
          vy = msg.linear.y  # m/s
          omega = msg.angular.z  # rad/s

          # 转换为STM32协议格式（mm/s, mrad/s）
          vx_mm_s = int(vx * 1000.0)
          vy_mm_s = int(vy * 1000.0)
          wz_mrad_s = int(omega * 1000.0)

          # 构建控制帧
          control_frame = self.build_velocity_frame(vx_mm_s, vy_mm_s, wz_mrad_s)

          # 添加到发送队列
          self.send_buffer.append(control_frame)

          # 记录最后命令时间
          self.last_cmd_time = time.time()

          self.get_logger().debug(f'发送速度命令: vx={vx:.2f}m/s, vy={vy:.2f}m/s, omega={omega:.2f}rad/s')

      def build_velocity_frame(self, vx_mm_s, vy_mm_s, wz_mrad_s):
          """
          构建速度控制帧（发送给STM32）
          协议格式（小端）：
          AA 55 06 01 VX VY WZ XOR 0D 0A
          """
          frame = bytearray()

          # 帧头
          frame.extend(self.FRAME_HEAD)

          # 长度和消息ID
          frame.append(6)  # payload长度
          frame.append(self.MSG_ID_VELOCITY_CMD)

          # 速度数据（小端）
          frame.extend(struct.pack('<h', vx_mm_s))
          frame.extend(struct.pack('<h', vy_mm_s))
          frame.extend(struct.pack('<h', wz_mrad_s))

          # XOR校验（从长度字段开始到WZ）
          checksum = 0
          for i in range(2, 8):  # 从长度字段到WZ结束
              checksum ^= frame[i]
          frame.append(checksum)

          # 帧尾
          frame.extend(self.FRAME_TAIL)

          return bytes(frame)

      # ==================== 处理线程 ====================
      def process_loop(self):
          """处理线程：监控连接状态、超时处理等"""
          while self.running:
              try:
                  current_time = time.time()

                  # 检查编码器数据超时
                  if (self.last_encoder_time is not None and
                      current_time - self.last_encoder_time > 0.5):  # 0.5秒超时
                      self.get_logger().warn('编码器数据超时')
                      self.last_encoder_time = None

                  # 检查IMU数据超时
                  if (self.last_imu_time is not None and
                      current_time - self.last_imu_time > 0.5):
                      self.get_logger().warn('IMU数据超时')
                      self.last_imu_time = None

                  # 定期发送心跳或状态请求（可选）
                  if current_time - self.get_clock().now().seconds_nanoseconds()[0] % 5 < 0.1:
                      self.send_status_request()

                  time.sleep(0.1)  # 100ms循环

              except Exception as e:
                  self.get_logger().error(f'处理线程错误: {e}')
                  time.sleep(0.1)

      def send_status_request(self):
          """发送状态请求帧（可选）"""
          # 可以发送请求帧获取STM32状态
          pass

      # ==================== 清理方法 ====================
      def destroy_node(self):
          """清理资源"""
          self.get_logger().info('正在关闭节点...')
          self.running = False

          # 等待线程结束
          if hasattr(self, 'recv_thread') and self.recv_thread.is_alive():
              self.recv_thread.join(timeout=1.0)

          if hasattr(self, 'send_thread') and self.send_thread.is_alive():
              self.send_thread.join(timeout=1.0)

          if hasattr(self, 'process_thread') and self.process_thread.is_alive():
              self.process_thread.join(timeout=1.0)

          # 关闭串口
          if self.ser and self.ser.is_open:
              self.ser.close()

          super().destroy_node()

  def main(args=None):
      rclpy.init(args=args)

      try:
          node = UnifiedCarController()
          rclpy.spin(node)
      except KeyboardInterrupt:
          node.get_logger().info('接收到Ctrl+C，正在关闭...')
      except Exception as e:
          node.get_logger().error(f'节点运行错误: {e}')
      finally:
          if 'node' in locals():
              node.destroy_node()
          rclpy.shutdown()

  if __name__ == '__main__':
      main()