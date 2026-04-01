#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

class GetRobotPose(Node):
    def __init__(self):
        super().__init__('get_robot_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 给 TF 一点时间填充 buffer
        self.create_timer(1.0, self.get_transform)
        
        # 打印一下已知的坐标系，帮助调试
        self.create_timer(5.0, self.list_frames)

    def list_frames(self):
        """列出所有已知的坐标系"""
        frames = self.tf_buffer.all_frames_as_string()
        self.get_logger().info(f"当前已知坐标系:\n{frames}")

    def get_transform(self):
        # 尝试从 map 到 base_link 或 base_footprint 的变换
        target_frames = ['base_link', 'base_footprint']
        
        for frame in target_frames:
            try:
                # 使用 Time(0) 获取最新可用的变换，并设置合理的超时
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    frame,
                    rclpy.time.Time(seconds=0),  # 获取最新变换
                    timeout=rclpy.duration.Duration(seconds=1.0)  # 等待1秒
                )
                
                # 打印位姿信息
                trans = transform.transform.translation
                rot = transform.transform.rotation
                
                self.get_logger().info(
                    f"\n🚀 成功获取到 '{frame}' 的位姿:\n"
                    f"  平移: x={trans.x:.3f}, y={trans.y:.3f}, z={trans.z:.3f}\n"
                    f"  旋转: x={rot.x:.3f}, y={rot.y:.3f}, z={rot.z:.3f}, w={rot.w:.3f}"
                )
                return  # 成功就返回
                
            except LookupException as e:
                self.get_logger().debug(f"'{frame}' 查找失败: {e}")
                continue
            except ConnectivityException as e:
                self.get_logger().debug(f"'{frame}' 连接问题: {e}")
                continue
            except ExtrapolationException as e:
                self.get_logger().debug(f"'{frame}' 外推异常: {e}")
                continue
            except Exception as e:
                self.get_logger().debug(f"'{frame}' 其他错误: {e}")
                continue
        
        # 如果都没找到，打印更详细的信息
        self.get_logger().warn(
            f"无法获取机器人位姿 (map→base_link/footprint)\n"
            f"请检查: \n"
            f"  1. 'map' 坐标系是否存在\n"
            f"  2. 'base_link' 或 'base_footprint' 是否存在\n"
            f"  3. 是否有从 map 到 base 的完整 TF 链"
        )

def main():
    rclpy.init()
    node = GetRobotPose()
    
    # 打印节点信息
    node.get_logger().info("机器人位姿监听节点已启动")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，正在关闭...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()