#!/usr/bin/env python3
"""
语音回复节点使用示例
演示如何向 /voice_responses 话题发布消息来触发语音播报
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class VoiceResponderExample(Node):
    def __init__(self):
        super().__init__('voice_responder_example')
        self.publisher_ = self.create_publisher(String, '/voice_responses', 10)
        self.get_logger().info("✅ 语音回复示例节点已启动")
        self.get_logger().info("📢 5秒后开始演示...")

        # 延迟演示
        self.timer = self.create_timer(5.0, self.run_demo)
        self.demo_step = 0

    def run_demo(self):
        """运行演示序列"""
        demo_messages = [
            "你好，欢迎使用语音回复系统",
            "正在前往景点1",
            "已到达景点1",
            "导航已暂停",
            "导航已继续",
            "已切换到全景点遍历模式",
            "导航完成，谢谢使用！",
        ]

        if self.demo_step < len(demo_messages):
            msg = String()
            msg.data = demo_messages[self.demo_step]
            self.get_logger().info(f"📤 发布：{msg.data}")
            self.publisher_.publish(msg)
            self.demo_step += 1
            # 每5秒演示一条
            self.timer = self.create_timer(5.0, self.run_demo)
        else:
            self.get_logger().info("✅ 演示完成")
            self.timer.cancel()


def main():
    rclpy.init()
    node = VoiceResponderExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
