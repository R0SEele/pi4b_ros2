#!/usr/bin/env python3
"""
简单的语音指令发布节点 - 基于 voice.py 修改
功能：识别语音后发布到 /voice_commands 话题
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
import audioop
import re
import shutil
from pypinyin import lazy_pinyin
from threading import Lock
import edge_tts


# ================= 配置区域 =================
MODEL_PATH = "/home/rose/pi4b_ros2/voice_control/vosk-model-small-cn-0.22"
DEBOUNCE_SECONDS = 1
VOICE_NAME = "zh-CN-XiaoxiaoNeural"
VOICE_RATE = "+0%"
VOICE_VOLUME = "+0%"
AUDIO_DEVICE = "alsa/hw:3,0"

# 录音参数
SAMPLE_RATE_INPUT = 48000
SAMPLE_RATE_VOSK = 16000
CHUNK_INPUT = 4800
INPUT_DEVICE_INDEX = 1

# 景点列表（编号格式）
SPOTS = ["景点1", "景点2", "景点3"]

# ================ 指令与回复配置 ================
COMMANDS = [
    {
        "name": "重新规划路线",
        "keywords": ["重新规划路线", "重规划", "换路线"],
        "command_text": "重新规划路线",
        "needs_spot": False
    },
    {
        "name": "全路径遍历模式",
        "keywords": ["全路径遍历", "全路径模式", "遍历路径"],
        "command_text": "全路径遍历模式",
        "needs_spot": False
    },
    {
        "name": "全景点遍历模式",
        "keywords": ["全景点遍历", "全景点模式", "遍历景点"],
        "command_text": "全景点遍历模式",
        "needs_spot": False
    },
    {
        "name": "暂停运动",
        "keywords": ["暂停", "暂停运动", "停下"],
        "command_text": "暂停运动",
        "needs_spot": False
    },
    {
        "name": "继续运动",
        "keywords": ["继续", "继续运动", "开始"],
        "command_text": "继续运动",
        "needs_spot": False
    },
    {
        "name": "前往景点",
        "keywords": ["前往景点", "先去", "导航到", "优先前往", "去"],
        "command_text": None,  # 需要根据景点动态生成
        "needs_spot": True
    },
    {
        "name": "查询人流密度",
        "keywords": ["查询", "人流密度", "人多不多", "拥挤", "查看人流"],
        "command_text": None,  # 需要根据景点动态生成
        "needs_spot": True
    }
]

# 预计算拼音首字母（用于模糊匹配）
for cmd in COMMANDS:
    cmd["pinyin_initials"] = [
        "".join([p[0] for p in lazy_pinyin(kw)]) for kw in cmd["keywords"]
    ]
# ===========================================

trigger_lock = Lock()
last_trigger_time = 0
resample_state = None


async def speak_async(text):
    communicate = edge_tts.Communicate(text, VOICE_NAME, rate=VOICE_RATE, volume=VOICE_VOLUME)
    temp_file = "/tmp/tts_temp.mp3"
    await communicate.save(temp_file)
    return temp_file


def check_mpv_installed():
    return shutil.which("mpv") is not None


def play_audio(file_path):
    """播放音频"""
    try:
        subprocess.run(
            ['mpv', '--vo=null', f'--audio-device={AUDIO_DEVICE}', file_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=True,
            timeout=10
        )
        return True
    except Exception as e:
        print(f"   播放失败：{e}")
        return False


def speak(text, stream=None, p=None):
    """播报语音"""
    # 1. 关闭录音流（如果提供）
    local_stream = stream
    local_p = p
    if local_stream and local_stream.is_active():
        local_stream.stop_stream()
        local_stream.close()
    if local_p:
        local_p.terminate()

    # 2. 播报语音
    print(f"🔊 正在播报：{text}")
    try:
        temp_file = asyncio.run(speak_async(text))
        play_audio(temp_file)
        os.remove(temp_file)
    except Exception as e:
        print(f"❌ 播报失败: {e}")
    time.sleep(0.5)

    # 3. 重新初始化录音
    new_p = pyaudio.PyAudio()
    new_stream = new_p.open(format=pyaudio.paInt16,
                    channels=1,
                    rate=SAMPLE_RATE_INPUT,
                    input=True,
                    input_device_index=INPUT_DEVICE_INDEX,
                    frames_per_buffer=CHUNK_INPUT)
    new_stream.start_stream()
    print("🎙️ 录音已恢复，等待指令...")
    return new_stream, new_p


def clean_text(text):
    """去除空格、标点符号，保留中文字符和数字"""
    return re.sub(r'[^\u4e00-\u9fa50-9]', '', text)


def extract_spot(text):
    """优先精确匹配景点名，若失败则尝试拼音首字母匹配"""
    cleaned = clean_text(text)
    for spot in SPOTS:
        if spot in cleaned:
            return spot
    # 拼音首字母模糊匹配（应对识别错误）
    text_initials = "".join([p[0] for p in lazy_pinyin(cleaned)])
    for spot in SPOTS:
        spot_initials = "".join([p[0] for p in lazy_pinyin(spot)])
        if spot_initials in text_initials:
            return spot
    return None


def match_command(text):
    """两级匹配：1. 精确关键词 2. 拼音首字母模糊匹配"""
    cleaned = clean_text(text)
    if not cleaned:
        return None, None

    # 第一级：精确关键词匹配
    for cmd in COMMANDS:
        if any(clean_text(kw) in cleaned for kw in cmd["keywords"]):
            spot = extract_spot(text) if cmd.get("needs_spot") else None
            return cmd, spot

    # 第二级：拼音首字母模糊匹配（容忍同音字、识别错误）
    text_initials = "".join([p[0] for p in lazy_pinyin(cleaned)])
    for cmd in COMMANDS:
        for initials in cmd["pinyin_initials"]:
            if initials in text_initials:
                print(f"🔍 拼音模糊匹配成功：{initials} ∈ {text_initials}")
                spot = extract_spot(text) if cmd.get("needs_spot") else None
                return cmd, spot
    return None, None


class VoicePublisher(Node):
    def __init__(self):
        super().__init__('voice_publisher')
        self.publisher_ = self.create_publisher(String, '/voice_commands', 10)
        self.get_logger().info("✅ 语音发布节点已启动")

    def publish_command(self, command_text):
        msg = String()
        msg.data = f"command={command_text}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"📤 已发布: {msg.data}")


def main():
    global last_trigger_time, resample_state

    # 检查 mpv
    if not check_mpv_installed():
        print("⚠️ 警告：未检测到 mpv 播放器，语音播报可能无声。")
        print("   请安装：sudo apt install mpv")

    # 初始化 ROS2
    rclpy.init()
    node = VoicePublisher()

    # 加载模型
    model_path = os.path.expanduser(MODEL_PATH)
    if not os.path.exists(model_path):
        node.get_logger().error(f"❌ 模型路径不存在：{model_path}")
        return

    node.get_logger().info("🔄 正在加载语音模型...")
    try:
        model = vosk.Model(model_path)
        rec = vosk.KaldiRecognizer(model, SAMPLE_RATE_VOSK)
        rec.SetWords(True)
        node.get_logger().info("✅ 模型加载成功！")
    except Exception as e:
        node.get_logger().error(f"❌ 模型加载失败：{e}")
        return

    # 初始化录音
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16,
                    channels=1,
                    rate=SAMPLE_RATE_INPUT,
                    input=True,
                    input_device_index=INPUT_DEVICE_INDEX,
                    frames_per_buffer=CHUNK_INPUT)
    stream.start_stream()
    resample_state = None

    node.get_logger().info("🎙️ 开始监听语音指令...")
    node.get_logger().info(f"🎯 支持指令：")
    node.get_logger().info(f"   1. 重新规划路线")
    node.get_logger().info(f"   2. 全路径遍历模式")
    node.get_logger().info(f"   3. 全景点遍历模式")
    node.get_logger().info(f"   4. 暂停运动 / 继续运动")
    node.get_logger().info(f"   5. 前往景点1/2/3")
    node.get_logger().info(f"   6. 查询景点1/2/3人流密度")
    node.get_logger().info(f"💡 拼音模糊匹配已启用，可容忍一定识别错误")

    try:
        while rclpy.ok():
            try:
                data = stream.read(CHUNK_INPUT, exception_on_overflow=False)
            except Exception:
                continue

            # 重采样
            converted, resample_state = audioop.ratecv(
                data, 2, 1, SAMPLE_RATE_INPUT, SAMPLE_RATE_VOSK, resample_state
            )
            if not converted:
                continue

            if rec.AcceptWaveform(converted):
                result = json.loads(rec.Result())
                text = result.get('text', '')
                if not text:
                    continue

                node.get_logger().info(f"🎤 识别到文本：{text}")

                cmd, spot = match_command(text)
                if cmd is None:
                    node.get_logger().info("⏭️ 未匹配任何指令")
                    continue

                # 如果需要景点但未提取到，静默忽略
                if cmd.get("needs_spot") and spot is None:
                    node.get_logger().info("⏭️ 未指定景点，忽略")
                    continue

                current_time = time.time()
                if current_time - last_trigger_time < DEBOUNCE_SECONDS:
                    node.get_logger().info(f"⏳ 防抖中")
                    continue

                if trigger_lock.locked():
                    node.get_logger().info(f"🔒 触发器被锁定，跳过")
                    continue

                # 确定要发布的指令文本
                if cmd["name"] == "查询人流密度" and spot:
                    command_text = f"查询{spot}人流密度"
                elif cmd.get("needs_spot") and spot:
                    command_text = f"前往{spot}"
                else:
                    command_text = cmd["command_text"]

                # 确定回复文本
                if cmd["name"] == "查询人流密度" and spot:
                    if spot == "景点1" or spot == "景点2":
                        response_text = f"{spot}当前人流密度较高，请谨慎前往"
                    else:
                        response_text = f"{spot}当前人流适中，无需排队"
                elif cmd.get("needs_spot") and spot:
                    response_text = f"正在前往{spot}"
                elif command_text == "暂停运动":
                    response_text = "已暂停运动"
                elif command_text == "继续运动":
                    response_text = "已继续运动"
                elif command_text == "重新规划路线":
                    response_text = "已重新规划路线"
                elif command_text == "全路径遍历模式":
                    response_text = "已切换到全路径遍历模式"
                elif command_text == "全景点遍历模式":
                    response_text = "已切换到全景点遍历模式"
                else:
                    response_text = "已执行指令"

                with trigger_lock:
                    node.get_logger().info(f"🎯 匹配到指令：{command_text}")
                    # 发布 ROS2 话题
                    node.publish_command(command_text)
                    last_trigger_time = time.time()

                # 在锁外进行语音播报（避免长时间阻塞）
                node.get_logger().info(f"🔊 准备播报：{response_text}")
                try:
                    stream, p = speak(response_text, stream, p)
                    node.get_logger().info(f"✅ 播报完成")
                except Exception as e:
                    node.get_logger().error(f"❌ 播报失败: {e}")
                    # 即使播报失败也要重新初始化录音
                    try:
                        if stream and stream.is_active():
                            stream.stop_stream()
                            stream.close()
                        if p:
                            p.terminate()
                    except:
                        pass
                    # 重新初始化录音
                    p = pyaudio.PyAudio()
                    stream = p.open(format=pyaudio.paInt16,
                                    channels=1,
                                    rate=SAMPLE_RATE_INPUT,
                                    input=True,
                                    input_device_index=INPUT_DEVICE_INDEX,
                                    frames_per_buffer=CHUNK_INPUT)
                    stream.start_stream()

    except KeyboardInterrupt:
        node.get_logger().info("👋 正在退出...")
    finally:
        if 'stream' in locals():
            stream.stop_stream()
            stream.close()
        if 'p' in locals():
            p.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
