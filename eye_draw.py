#!/usr/bin/env python3
import pygame
import sys
import random
import os

# 解决 X11 问题
os.environ['SDL_VIDEO_X11_REQUIRE_SHM'] = '0'
os.environ['SDL_VIDEODRIVER'] = 'x11'

def main():
    # 初始化
    pygame.init()
    
    # 设置窗口（全屏或窗口模式）
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("会眨眼的眼睛")
    
    # 颜色定义
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    BLUE = (100, 100, 255)
    
    clock = pygame.time.Clock()
    
    # 眼睛参数
    eye_center = (400, 300)
    eye_radius = 150
    pupil_radius = 60
    
    # 眨眼控制
    blink_counter = 0
    is_blinking = False
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
        
        # 清屏
        screen.fill((200, 200, 200))
        
        # 随机眨眼逻辑
        if not is_blinking:
            # 随机决定是否眨眼（每帧0.5%概率）
            if random.random() < 0.005:
                is_blinking = True
                blink_counter = 0
        else:
            blink_counter += 1
            # 眨眼持续5帧（约0.08秒）
            if blink_counter > 5:
                is_blinking = False
        
        # 画眼白（外圈）
        pygame.draw.circle(screen, WHITE, eye_center, eye_radius)
        pygame.draw.circle(screen, BLACK, eye_center, eye_radius, 3)
        
        if not is_blinking:
            # 睁眼状态 - 画瞳孔
            pygame.draw.circle(screen, BLACK, eye_center, pupil_radius)
            # 画高光
            highlight_pos = (eye_center[0] - 20, eye_center[1] - 20)
            pygame.draw.circle(screen, WHITE, highlight_pos, 15)
        else:
            # 眨眼状态 - 画眼皮线
            pygame.draw.line(screen, BLACK, 
                           (eye_center[0] - eye_radius, eye_center[1]),
                           (eye_center[0] + eye_radius, eye_center[1]), 5)
        
        # 更新显示
        pygame.display.flip()
        clock.tick(60)  # 60帧每秒
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
