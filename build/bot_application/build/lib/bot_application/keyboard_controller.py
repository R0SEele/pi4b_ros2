#!/usr/bin/env python3
"""
键盘控制节点 - 替代语音控制，通过按键发送指令
按键映射：
  Q - 重新规划路线
  W - 全路径遍历模式
  E - 全景点遍历模式
  R - 暂停运动
  T - 继续运动
  Y - 前往景点1
  U - 前往景点2
  I - 前往景点3
  A - 查询景点1人流密度
  S - 查询景点2人流密度
  D - 查询景点3人流密度
  P - 播报规划路线
  O - 询问优先景点
  H - 显示帮助
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import termios
import tty
import os
import subprocess
import asyncio
import shutil
import time
import edge_tts
from threading import Lock, Thread


# ================= 配置区域 =================
VOICE_NAME = "zh-CN-XiaoxiaoNeural"
VOICE_RATE = "+0%"
VOICE_VOLUME = "+0%"
DEBOUNCE_SECONDS = 0.3


# 按键映射配置
KEY_MAPPING = {
    'q': {
        'command': '重新规划路线',
        'voice': '已重新规划路线',
        'description': '重新规划路线'
    },
    'w': {
        'command': '全路径遍历模式',
        'voice': '已切换到全路径遍历模式',
        'description': '全路径遍历模式'
    },
    'e': {
        'command': '全景点遍历模式',
        'voice': '已切换到全景点遍历模式',
        'description': '全景点遍历模式'
    },
    'r': {
        'command': '暂停运动',
        'voice': '已暂停运动',
        'description': '暂停运动'
    },
    't': {
        'command': '继续运动',
        'voice': '已继续运动',
        'description': '继续运动'
    },
    'y': {
        'command': '前往景点1',
        'voice': '正在前往景点1',
        'description': '前往景点1'
    },
    'u': {
        'command': '前往景点2',
        'voice': '正在前往景点2',
        'description': '前往景点2'
    },
    'i': {
        'command': '前往景点3',
        'voice': '已重新规划路线，优先前往景点3,当前访问顺序为景点3，到景点2，到景点1',
        'description': '前往景点3'
    },
    'a': {
        'command': '查询景点1人流密度',
        'voice': '景点1当前人流密度较高，请谨慎前往',
        'description': '查询景点1人流密度'
    },
    's': {
        'command': '查询景点2人流密度',
        'voice': '景点2当前人流密度较高，请谨慎前往',
        'description': '查询景点2人流密度'
    },
    'd': {
        'command': '查询景点3人流密度',
        'voice': '景点3当前人流适中，无需排队',
        'description': '查询景点3人流密度'
    },
    'p': {
        'command': '播报规划路线',
        'voice': '已规划路线，景点访问顺序为1,2,3.',
        'description': '播报规划路线'
    },
    'o': {
        'command': '询问优先景点',
        'voice': '当前规划最优访问顺序为景点1，到景点2，到景点3,您可以告诉我您想最优先去的景点，我来重新规划。',
        'description': '询问优先景点'
    },
    'm': {
        'command': '优先考虑避开拥挤',
        'voice': '已提高避开拥挤的优先级，当前规划最优访问顺序为景点3,到景点2,到景点1',
        'description': '询问优先景点'
    },
    'n': {
        'command': '',
        'voice': '已到达景点1,这里风景优美，可以享受自然风光',
        'description': ''
    },
    'b': {
        'command': '',
        'voice': '已到达景点2,这里人文底蕴浓厚，是景区标志性打卡点',
        'description': ''
    },
    'v': {
        'command': '',
        'voice': '已到达景点3,这里环境清幽，适合休闲漫步与放松身心',
        'description': ''
    },
    'f': {
        'command': '',
        'voice': '景点已全部访问完毕，前往景区出入口',
        'description': ''
    },
}


speak_lock = Lock()
last_speak_time = 0


async def speak_async(text):
    """异步生成语音文件"""
    communicate = edge_tts.Communicate(
        text,
        VOICE_NAME,
        rate=VOICE_RATE,
        volume=VOICE_VOLUME
    )
    temp_file = "/tmp/tts_keyboard_temp.mp3"
    try:
        await communicate.save(temp_file)
        return temp_file
    except Exception as e:
        print(f"❌ TTS 生成失败：{e}")
        return None


def check_mpv_installed():
    """检查 mpv 是否安装"""
    return shutil.which("mpv") is not None


def play_audio(file_path):
    """播放音频文件 - 使用默认设备"""
    try:
        subprocess.run(
            ['mpv', '--vo=null', file_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=True,
            timeout=30
        )
        return True
    except Exception as e:
        print(f"   播放失败：{e}")
        return False


def speak_text(text):
    """播放指定文本（同步阻塞）"""
    global last_speak_time

    # 防抖检查
    current_time = time.time()
    if current_time - last_speak_time < DEBOUNCE_SECONDS:
        return False

    with speak_lock:
        print(f"🔊 播报：{text}")
        try:
            audio_file = asyncio.run(speak_async(text))
            if audio_file:
                play_audio(audio_file)
                os.remove(audio_file)
                last_speak_time = time.time()
                return True
        except Exception as e:
            print(f"❌ 播报失败: {e}")
        return False


def speak_text_async(text):
    """在后台线程中播放语音（非阻塞）"""
    thread = Thread(target=speak_text, args=(text,), daemon=True)
    thread.start()


def get_key(settings):
    """获取单个按键输入（非阻塞）"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_help():
    """打印帮助信息"""
    print("\n" + "="*50)
    print("⌨️  键盘控制说明")
    print("="*50)
    for key, mapping in sorted(KEY_MAPPING.items()):
        print(f"  {key.upper()} - {mapping['description']}")
    print("  H - 显示此帮助")
    print("  Ctrl+C - 退出")
    print("="*50 + "\n")


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        # 发布器 - 发布到语音指令话题
        self.publisher_ = self.create_publisher(String, '/voice_commands', 10)

        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("✅ 键盘控制节点已启动")
        print_help()

        if not check_mpv_installed():
            self.get_logger().warning("⚠️ 未检测到 mpv 播放器，语音播报可能无声")

    def publish_command(self, command_text):
        """发布指令到话题"""
        msg = String()
        msg.data = f"command={command_text}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"📤 已发布: {msg.data}")

    def run(self):
        """主循环"""
        try:
            while rclpy.ok():
                key = get_key(self.settings).lower()

                if key:
                    if key in KEY_MAPPING:
                        mapping = KEY_MAPPING[key]
                        command = mapping['command']
                        voice = mapping['voice']

                        self.get_logger().info(f"⌨️  按键: {key.upper()} - {mapping['description']}")

                        # 发布指令
                        self.publish_command(command)

                        # 播放语音回应
                        speak_text_async(voice)

                    elif key == 'h':
                        print_help()

                    elif key == '\x03':  # Ctrl+C
                        raise KeyboardInterrupt()

                # 处理 ROS 事件
                rclpy.spin_once(self, timeout_sec=0.01)

        except KeyboardInterrupt:
            self.get_logger().info("👋 正在退出...")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rclpy.init()
    node = KeyboardController()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
