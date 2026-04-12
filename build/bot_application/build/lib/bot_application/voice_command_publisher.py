#!/usr/bin/env python3
"""
语音指令发布节点
功能：
1. 使用 Vosk 进行中文语音识别
2. 识别到语音指令后发布到 /voice_commands 话题
3. 支持多种指令（为未来扩展预留）
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import vosk
import pyaudio
import json
import os
import subprocess
import time
import asyncio
from pypinyin import lazy_pinyin
from threading import Lock
import edge_tts


# ================= 配置区域 =================
# 模型路径 - 优先使用本地模型
MODEL_PATH = os.path.join(os.path.dirname(__file__), '../../../../vosk-model-small-cn-0.22')
# 如果本地没有则使用下载目录的模型
FALLBACK_MODEL_PATH = "/home/rose/pi4b_ros2/voice_control/vosk-model-small-cn-0.22"

TARGET_TEXT = "重新规划路线"
# 目标文本拼音首字母
TARGET_INITIALS = "".join([pinyin[0] for pinyin in lazy_pinyin(TARGET_TEXT)])
# 防抖时间（秒）
DEBOUNCE_SECONDS = 1
# 【语音配置】可更换音色
VOICE_NAME = "zh-CN-XiaoxiaoNeural"
VOICE_RATE = "+0%"
VOICE_VOLUME = "+0%"

# 【音频输出设备】
AUDIO_DEVICE = "alsa/hw:3,0"
# ===========================================

trigger_lock = Lock()
last_trigger_time = 0


async def speak_async(text):
    """Edge-TTS异步播报，带参数控制"""
    communicate = edge_tts.Communicate(
        text,
        VOICE_NAME,
        rate=VOICE_RATE,
        volume=VOICE_VOLUME
    )
    temp_file = "/tmp/tts_temp.mp3"
    await communicate.save(temp_file)
    return temp_file


def speak(text):
    """播报语音"""
    print(f"🔊 正在播报：{text}")
    temp_file = asyncio.run(speak_async(text))

    # 尝试播放，如果失败则跳过
    try:
        subprocess.run(
            ['mpv', '--vo=null', f'--audio-device={AUDIO_DEVICE}', temp_file],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10
        )
    except Exception as e:
        print(f"⚠️  语音播放失败: {e}")

    try:
        os.remove(temp_file)
    except:
        pass

    time.sleep(0.5)


def get_text_initials(text):
    """预处理文本：去空格+转首字母"""
    text_clean = text.replace(" ", "").strip()
    return "".join([pinyin[0] for pinyin in lazy_pinyin(text_clean)])


def contains_three_consecutive_match(text_initials, target_initials):
    """检查是否包含目标首字母的任意连续三个字符"""
    if len(target_initials) < 3:
        return target_initials in text_initials

    substrings = [target_initials[i:i+3] for i in range(len(target_initials) - 2)]
    for sub in substrings:
        if sub in text_initials:
            return True
    return False


class VoiceCommandPublisher(Node):
    def __init__(self):
        super().__init__('voice_command_publisher')

        # 创建发布器
        self.publisher_ = self.create_publisher(String, '/voice_commands', 10)

        # 确定模型路径
        self.model_path = self._find_model_path()
        if not self.model_path:
            self.get_logger().error("❌ 未找到语音识别模型")
            return

        # 初始化语音识别
        self._init_voice_recognition()

        # 启动语音识别线程
        self.running = True
        import threading
        self.voice_thread = threading.Thread(target=self._voice_recognition_loop, daemon=True)
        self.voice_thread.start()

        self.get_logger().info("✅ 语音指令发布节点已启动")
        self.get_logger().info(f"🎯 目标指令：{TARGET_TEXT}")

    def _find_model_path(self):
        """查找语音模型路径"""
        paths_to_check = [
            MODEL_PATH,
            FALLBACK_MODEL_PATH,
            "/home/vboxuser/pi4b_ros2-master/vosk-model-small-cn-0.22",
            "vosk-model-small-cn-0.22"
        ]

        for path in paths_to_check:
            expanded = os.path.expanduser(path)
            if os.path.exists(expanded):
                self.get_logger().info(f"📦 使用模型路径: {expanded}")
                return expanded

        return None

    def _init_voice_recognition(self):
        """初始化语音识别"""
        self.get_logger().info("🔄 正在加载语音模型...")
        try:
            self.model = vosk.Model(self.model_path)
            self.rec = vosk.KaldiRecognizer(self.model, 16000)
            self.rec.SetWords(True)
            self.get_logger().info("✅ 模型加载成功！")
        except Exception as e:
            self.get_logger().error(f"❌ 模型加载失败: {e}")
            return False

        # 初始化录音
        try:
            self.p = pyaudio.PyAudio()
            self.stream = self.p.open(format=pyaudio.paInt16,
                            channels=1,
                            rate=16000,
                            input=True,
                            frames_per_buffer=8000)
            self.stream.start_stream()
            return True
        except Exception as e:
            self.get_logger().error(f"❌ 录音初始化失败: {e}")
            return False

    def _voice_recognition_loop(self):
        """语音识别主循环"""
        global last_trigger_time

        self.get_logger().info("🎙️ 开始监听语音指令...")

        try:
            while rclpy.ok() and self.running:
                try:
                    data = self.stream.read(4000, exception_on_overflow=False)
                except Exception as e:
                    time.sleep(0.01)
                    continue

                if self.rec.AcceptWaveform(data):
                    result = json.loads(self.rec.Result())
                    text = result.get('text', '')
                    if not text:
                        continue

                    self.get_logger().info(f"🎤 识别到文本: {text}")

                    text_initials = get_text_initials(text)
                    self.get_logger().debug(f"🔧 首字母: {text_initials}")

                    if contains_three_consecutive_match(text_initials, TARGET_INITIALS):
                        current_time = time.time()
                        if current_time - last_trigger_time < DEBOUNCE_SECONDS:
                            self.get_logger().info(f"⏳ 防抖中")
                            continue

                        if trigger_lock.locked():
                            continue

                        with trigger_lock:
                            self.get_logger().info("🎯 匹配到目标指令!")

                            # 发布语音指令消息
                            msg = String()
                            msg.data = f"command={TARGET_TEXT}"
                            self.publisher_.publish(msg)
                            self.get_logger().info(f"📤 已发布指令: {msg.data}")

                            # 语音反馈
                            try:
                                speak("已重新规划路线！")
                            except Exception as e:
                                self.get_logger().warning(f"⚠️ 语音播报失败: {e}")

                            last_trigger_time = time.time()

        except Exception as e:
            self.get_logger().error(f"❌ 语音识别错误: {e}")

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'stream'):
            try:
                self.stream.stop_stream()
                self.stream.close()
            except:
                pass
        if hasattr(self, 'p'):
            try:
                self.p.terminate()
            except:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = VoiceCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 收到退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
