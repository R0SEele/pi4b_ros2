#!/usr/bin/env python3
import os

# 解决 X11 共享内存问题
os.environ['SDL_VIDEO_X11_REQUIRE_SHM'] = '0'
os.environ['SDL_VIDEODRIVER'] = 'x11'

import pygame
import sys

# 你原有的代码继续...
import pygame
import time
import random

pygame.init()
screen = pygame.display.set_mode((800, 480))

# 加载两张图片：睁眼和闭眼
eye_open = pygame.image.load("eye_open.png")
eye_closed = pygame.image.load("eye_closed.png")

while True:
    # 显示睁眼，持续随机时间（2-5秒）
    screen.blit(eye_open, (0, 0))
    pygame.display.update()
    time.sleep(random.uniform(2, 5))
    
    # 眨眼：快速切换闭眼再睁开
    screen.blit(eye_closed, (0, 0))
    pygame.display.update()
    time.sleep(0.1)