import cv2

video_path = '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/data/vedio/spot1.mp4'
cap = cv2.VideoCapture(video_path)

if cap.isOpened():
    print("视频文件已成功打开")
else:
    print("无法打开视频文件")