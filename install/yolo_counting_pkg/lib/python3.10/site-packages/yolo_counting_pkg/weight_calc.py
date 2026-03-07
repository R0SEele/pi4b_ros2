import json

# 权重计算配置
LEVEL_WEIGHTS = {"C级": 0.2, "B级": 0.5, "A级": 0.8}
INPUT_FILE = "weight_calc_input.json"  # 和检测节点输出路径一致

def calculate_weight(average_persons, level):
    """自定义权重计算逻辑（可修改）"""
    base_weight = LEVEL_WEIGHTS.get(level, 0.1)
    final_weight = base_weight * (average_persons / 100)
    return round(final_weight, 3)

if __name__ == "__main__":
    # 读取检测结果
    if not os.path.exists(INPUT_FILE):
        print("❌ 未找到检测结果文件！")
        exit(1)
    
    print("=== 权重计算结果 ===\n")
    with open(INPUT_FILE, "r", encoding="utf-8") as f:
        for line in f.readlines():
            if line.strip():
                data = json.loads(line)
                video_name = data["video_name"]
                avg_persons = data["average_persons"]
                level = data["level"]
                
                # 计算权重
                weight = calculate_weight(avg_persons, level)
                print(f"视频：{video_name} | 平均人数：{avg_persons} | 等级：{level} | 权重：{weight}")