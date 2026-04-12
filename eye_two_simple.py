#!/usr/bin/env python3
import pygame
import sys
import random
import os
import math

os.environ['SDL_VIDEO_X11_REQUIRE_SHM'] = '0'
os.environ['SDL_VIDEODRIVER'] = 'x11'

def main():
    pygame.init()
    
    # 获取屏幕信息
    screen_info = pygame.display.Info()
    screen_width = screen_info.current_w
    screen_height = screen_info.current_h
    
    # 创建窗口
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("两只眼睛")
    
    # 颜色
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    GRAY = (50, 50, 50)
    
    # 眼睛参数
    eye_radius = min(screen_width, screen_height) // 6
    pupil_radius = eye_radius // 2
    
    # 两只眼睛的位置
    spacing = eye_radius * 2.5
    left_eye = (screen_width // 2 - spacing, screen_height // 2)
    right_eye = (screen_width // 2 + spacing, screen_height // 2)
    
    # 眨眼控制
    blink_counter = 0
    is_blinking = False
    next_blink = random.randint(60, 180)
    
    clock = pygame.time.Clock()
    
    def draw_eye(center, blinking):
        # 眼白
        pygame.draw.circle(screen, WHITE, center, eye_radius)
        pygame.draw.circle(screen, BLACK, center, eye_radius, 3)
        
        if not blinking:
            # 瞳孔
            pygame.draw.circle(screen, BLACK, center, pupil_radius)
            # 高光
            highlight = (center[0] - pupil_radius//3, center[1] - pupil_radius//3)
            pygame.draw.circle(screen, WHITE, highlight, pupil_radius//4)
        else:
            # 眨眼
            pygame.draw.line(screen, GRAY,
                           (center[0] - eye_radius, center[1]),
                           (center[0] + eye_radius, center[1]), 
                           eye_radius)
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
        
        # 眨眼逻辑
        if not is_blinking:
            blink_counter += 1
            if blink_counter >= next_blink:
                is_blinking = True
                blink_counter = 0
        else:
            blink_counter += 1
            if blink_counter >= 5:
                is_blinking = False
                blink_counter = 0
                next_blink = random.randint(60, 180)
        
        # 清屏
        screen.fill(GRAY)
        
        # 画两只眼睛
        draw_eye(left_eye, is_blinking)
        draw_eye(right_eye, is_blinking)
        
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
