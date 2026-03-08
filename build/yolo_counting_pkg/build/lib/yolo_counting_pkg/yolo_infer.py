import os
import glob
import json
from ultralytics import YOLO

# 检测配置（可根据需求修改）
CONF_THRESHOLD = 0.5
DETECT_CLASSES = [0]  # 仅检测人
IMG_SIZE = 480        # 树莓派优化分辨率
SKIP_FRAMES = 2       # 跳帧检测

# 人数等级划分规则
LEVEL_RULES = {
    "C级": (0, 20),
    "B级": (20, 50),
    "A级": (50, float('inf'))
}

def get_all_video_files(source_dir):
    """获取文件夹中所有支持的视频文件"""
    SUPPORT_FORMATS = ['.mp4', '.avi', '.mov', '.mkv']
    files = []
    for fmt in SUPPORT_FORMATS:
        files.extend(glob.glob(os.path.join(source_dir, f"*{fmt}")))
        files.extend(glob.glob(os.path.join(source_dir, f"*{fmt.upper()}")))
    return sorted(list(set(files)))

def get_person_level(average_persons):
    """根据平均人数划分等级"""
    for level, (min_num, max_num) in LEVEL_RULES.items():
        if min_num <= average_persons < max_num:
            return level
    return "未知等级"

def process_video(model_path, video_path, result_save_root, weight_calc_path):
    """处理单个视频：检测+统计+等级划分+结果保存"""
    # 加载YOLO模型
    try:
        model = YOLO(model_path, task="detect")
    except Exception as e:
        print(f"模型加载失败：{e}")
        return None
    
    # 获取视频基础信息
    file_name = os.path.basename(video_path)
    file_name_no_ext = os.path.splitext(file_name)[0]
    
    # 初始化统计变量
    total_frames = 0
    total_persons = 0
    
    # 执行视频检测（流式处理，降低内存占用）
    results = model.predict(
        source=video_path,
        save=False, show=False, save_txt=False, save_conf=False, save_crop=False,
        conf=CONF_THRESHOLD, classes=DETECT_CLASSES, imgsz=IMG_SIZE,
        device="cpu", batch=1, verbose=False, stream=True, vid_stride=SKIP_FRAMES
    )
    
    # 遍历检测结果
    for r in results:
        total_frames += SKIP_FRAMES
        total_persons += len(r.boxes)
    
    # 修正总帧数（避免0帧）
    total_frames = max(total_frames, 1)
    # 计算平均人数
    average_persons = round(total_persons / total_frames, 2)
    # 划分等级
    person_level = get_person_level(average_persons)
    
    # 保存统计txt（原始格式+等级）
    stats_txt_path = os.path.join(result_save_root, f"{file_name_no_ext}_result.txt")
    with open(stats_txt_path, "w", encoding="utf-8") as f:
        f.write(f"TF: {total_frames}\n")
        f.write(f"TP: {total_persons}\n")
        f.write(f"AP: {average_persons}\n")
        f.write(f"Level: {person_level}\n")
    
    # 写入权重计算输入文件（JSON格式）
    result_data = {
        "video_name": file_name,
        "average_persons": average_persons,
        "level": person_level
    }
    with open(weight_calc_path, "a", encoding="utf-8") as f:
        json.dump(result_data, f, ensure_ascii=False)
        f.write("\n")
    
    # 返回检测结果
    return {
        "file_name": file_name,
        "total_frames": total_frames,
        "total_persons": total_persons,
        "average_persons": average_persons,
        "level": person_level
    }