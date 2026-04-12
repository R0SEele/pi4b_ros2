#!/usr/bin/env python3
import asyncio
import websockets
import socket
from websockets.server import serve

async def handle_connection(websocket):
    """处理WebSocket连接"""
    print(f"✅ 客户端已连接: {websocket.remote_address}")
    
    try:
        # 发送欢迎消息
        await websocket.send("✅ 连接成功")
        
        # 持续接收消息
        async for message in websocket:
            print(f"📩 收到消息: {message}")
            response = f"✅ 树莓派已收到：{message}"
            await websocket.send(response)
            print(f"📤 已回复: {response}")
            
    except websockets.exceptions.ConnectionClosed:
        print(f"❌ 客户端断开连接")
    except Exception as e:
        print(f"⚠️ 错误: {e}")

async def main():
    local_ip = '10.163.62.222'
    
    print("="*50)
    print("树莓派 WebSocket 服务器")
    print("="*50)
    print(f"本机IP: {local_ip}")
    print(f"服务地址: ws://{local_ip}:8765")
    print("="*50)
    
    # 添加CORS支持和更宽松的设置
    async with serve(
        handle_connection,
        "0.0.0.0",
        8765,
        compression=None,
        ping_interval=30,
        ping_timeout=30,
        close_timeout=10,
        max_size=2**23,  # 8MB
        max_queue=32
    ) as server:
        print("✅ 服务器启动成功")
        print("等待连接...")
        print("按 Ctrl+C 退出")
        print("="*50)
        
        await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n👋 服务器已关闭")