import onnxruntime as ort
import cv2
import numpy as np
import os
import glob
import warnings
warnings.filterwarnings('ignore')  # 屏蔽所有警告

# ===================== 树莓派专属优化配置 =====================
MODEL_PATH = "/home/rose/car_ws/src/yolo_counting_pkg/models/best.onnx"
VIDEO_DIR = "/home/rose/car_ws/src/yolo_counting_pkg/data/vedio"
RESULT_DIR = "/home/rose/car_ws/src/yolo_counting_pkg/results"
CONF_THRESHOLD = 0.15  # 树莓派专属低阈值（提升检出率）
NMS_IOU_THRESHOLD = 0.5  # 放宽NMS（减少漏检）
IMG_SIZE = 640
FRAME_INTERVAL = 100  # 每100帧检测一帧
BOX_COLOR = (0, 255, 0)  # 绿色框
BOX_THICKNESS = 2
FONT_SCALE = 0.5
MIN_BOX_SIZE = 8  # 过滤极小无效框（像素）
# ==============================================================

# 创建结果目录
os.makedirs(RESULT_DIR, exist_ok=True)

# ===================== 核心优化函数 =====================
def load_optimized_model(model_path):
    """树莓派专属模型加载（CPU优化）"""
    # ONNX Runtime 极致CPU优化
    sess_options = ort.SessionOptions()
    sess_options.log_severity_level = 3
    sess_options.intra_op_num_threads = 4  # 用满树莓派核心
    sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    
    providers = [
        ('CPUExecutionProvider', {
            'arena_extend_strategy': 'kNextPowerOfTwo',
            'cpu_mem_arena_enable': True,
            'denormal_as_zero': True,  # 提升数值稳定性
            'enable_mkldnn': False     # 树莓派禁用MKLDNN
        })
    ]
    
    try:
        ort_session = ort.InferenceSession(model_path, sess_options=sess_options, providers=providers)
        input_name = ort_session.get_inputs()[0].name
        output_name = ort_session.get_outputs()[0].name
        print("✅ 模型加载完成（树莓派CPU优化版）")
        return ort_session, input_name, output_name
    except Exception as e:
        print(f"❌ 模型加载失败：{str(e)}")
        exit(1)

def precise_preprocess(frame):
    """和电脑端完全一致的预处理（像素级校准）"""
    if frame is None or frame.size == 0:
        return np.zeros((1, 3, IMG_SIZE, IMG_SIZE), dtype=np.float32), 1.0, 0, 0, 0, 0
    
    # 保存原始尺寸
    orig_h, orig_w = frame.shape[:2]
    
    # 严格保持比例的Letterbox（和YOLOv8训练一致）
    ratio = min(IMG_SIZE / orig_w, IMG_SIZE / orig_h)
    new_w, new_h = int(orig_w * ratio), int(orig_h * ratio)
    
    # 高质量插值（树莓派默认插值差，手动指定）
    frame_resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_CUBIC)
    
    # 居中填充（黑色，和训练一致）
    pad_w = (IMG_SIZE - new_w) // 2
    pad_h = (IMG_SIZE - new_h) // 2
    frame_padded = np.full((IMG_SIZE, IMG_SIZE, 3), 114, dtype=np.uint8)
    frame_padded[pad_h:pad_h+new_h, pad_w:pad_w+new_w, :] = frame_resized
    
    # 归一化（高精度浮点）
    img = frame_padded.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    
    return img, ratio, pad_w, pad_h, orig_w, orig_h

def postprocess_with_nms(output, ratio, pad_w, pad_h, orig_w, orig_h):
    """树莓派专属后处理（NMS优化+精度补偿）"""
    boxes = []
    confidences = []
    
    # 适配输出维度，高精度处理
    predictions = output[0].T.astype(np.float32) if len(output.shape) == 3 else output.T.astype(np.float32)
    
    for pred in predictions:
        if len(pred) < 5:
            continue
        
        # 解析YOLOv8 xywh格式
        x_c, y_c, w, h = pred[:4]
        conf = pred[4]
        cls_scores = pred[5:] if len(pred) > 5 else np.array([1.0], dtype=np.float32)
        
        # 低阈值过滤（树莓派专属）
        if conf < CONF_THRESHOLD:
            continue
        
        # 单类别（head）判定
        cls_id = np.argmax(cls_scores) if len(cls_scores) > 0 else 0
        if cls_id != 0:
            continue
        
        # 坐标精准还原（补偿树莓派浮点误差）
        x_c = (x_c - pad_w) / ratio
        y_c = (y_c - pad_h) / ratio
        w = w / ratio
        h = h / ratio
        
        # xywh转x1y1x2y2
        x1 = int(round(x_c - w / 2))
        y1 = int(round(y_c - h / 2))
        x2 = int(round(x_c + w / 2))
        y2 = int(round(y_c + h / 2))
        
        # 严格边界检查
        x1 = max(0, min(x1, orig_w))
        y1 = max(0, min(y1, orig_h))
        x2 = max(0, min(x2, orig_w))
        y2 = max(0, min(y2, orig_h))
        
        # 过滤极小框（树莓派易出无效小框）
        if (x2 - x1) < MIN_BOX_SIZE or (y2 - y1) < MIN_BOX_SIZE:
            continue
        
        boxes.append([x1, y1, x2, y2])
        confidences.append(float(conf))
    
    # 优化版NMS（解决重复框，树莓派适配）
    final_boxes = []
    if len(boxes) > 0:
        # 转换为OpenCV兼容格式
        boxes_np = np.array(boxes, dtype=np.float32)
        confs_np = np.array(confidences, dtype=np.float32)
        
        # 执行NMS，放宽IOU阈值
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, 
            score_threshold=CONF_THRESHOLD,
            nms_threshold=NMS_IOU_THRESHOLD
        )
        
        # 兼容不同OpenCV版本的返回格式
        if isinstance(indices, (np.ndarray, list)) and len(indices) > 0:
            for i in indices.flatten() if len(indices.shape) > 1 else indices:
                i = int(i)
                x1, y1, x2, y2 = boxes[i]
                final_boxes.append((x1, y1, x2, y2, confidences[i]))
    
    head_count = len(final_boxes)
    return head_count, final_boxes

def draw_optimized_boxes(frame, boxes, save_path, frame_idx):
    """树莓派专属绘图（抗锯齿+清晰文本）"""
    frame_copy = frame.copy()
    
    for (x1, y1, x2, y2, conf) in boxes:
        # 抗锯齿绘制框（树莓派默认绘制模糊）
        cv2.rectangle(frame_copy, (x1, y1), (x2, y2), BOX_COLOR, BOX_THICKNESS, lineType=cv2.LINE_AA)
        
        # 清晰文本（避免模糊）
        text = f"head {conf:.2f}"
        text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, BOX_THICKNESS)
        text_x = x1
        text_y = y1 - 5 if y1 - 5 > text_size[1] else y1 + text_size[1] + 5
        
        # 绘制文本背景（提升可读性）
        cv2.rectangle(frame_copy, (text_x, text_y - text_size[1] - 2), 
                      (text_x + text_size[0], text_y + 2), BOX_COLOR, -1)
        cv2.putText(frame_copy, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                    FONT_SCALE, (255, 255, 255), BOX_THICKNESS, lineType=cv2.LINE_AA)
    
    # 保存高质量图片
    img_filename = f"{os.path.basename(save_path)}_frame_{frame_idx}.jpg"
    img_save_path = os.path.join(save_path, img_filename)
    cv2.imwrite(img_save_path, frame_copy, [cv2.IMWRITE_JPEG_QUALITY, 95])  # 高质量JPG
    
    return img_save_path

def process_video_optimized(video_path, ort_session, input_name, output_name):
    """树莓派专属视频处理（全流程优化）"""
    video_basename = os.path.splitext(os.path.basename(video_path))[0]
    video_result_dir = os.path.join(RESULT_DIR, video_basename)
    os.makedirs(video_result_dir, exist_ok=True)
    
    # 打开视频（树莓派专属参数）
    cap = cv2.VideoCapture(video_path)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 减少缓存，提升实时性
    if not cap.isOpened():
        print(f"❌ 无法打开视频：{video_basename}")
        return
    
    # 初始化统计
    total_heads = 0
    detected_frames = 0
    detected_details = []
    total_video_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"\n📌 处理视频：{video_basename}（总帧数：{total_video_frames}）")
    
    # 逐帧处理
    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        if frame_idx % FRAME_INTERVAL == 0:
            # 高精度预处理
            img, ratio, pad_w, pad_h, orig_w, orig_h = precise_preprocess(frame)
            
            # 推理（高精度）
            output = ort_session.run([output_name], {input_name: img})
            
            # 优化后处理
            frame_heads, boxes = postprocess_with_nms(
                output[0], ratio, pad_w, pad_h, orig_w, orig_h
            )
            
            # 保存高质量带框图片
            img_path = draw_optimized_boxes(frame, boxes, video_result_dir, frame_idx)
            
            # 更新统计
            total_heads += frame_heads
            detected_frames += 1
            detected_details.append({
                "frame_idx": frame_idx,
                "head_count": frame_heads,
                "box_count": len(boxes),
                "img_path": img_path
            })
        
        frame_idx += 1
    
    cap.release()
    
    # 计算统计值（避免除以0）
    avg_per_frame = total_heads / detected_frames if detected_frames > 0 else 0.0
    
    # 生成结果文件
    txt_path = os.path.join(video_result_dir, f"{video_basename}_result.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write(f"Frame Count：{total_video_frames}\n")
        f.write(f"Detected Frames：{detected_frames}\n")
        f.write(f"Total Detected People：{total_heads}\n")
        f.write(f"Average per Detected Frame：{avg_per_frame:.2f}\n")    
    # 终端输出
    print(f"\n✅ 视频「{video_basename}」处理完成：")
    print(f"   检测帧数：{detected_frames} | 总人数：{total_heads} | 平均：{avg_per_frame:.2f}")
    print(f"   结果文件：{txt_path}")

# ===================== 主函数 =====================
if __name__ == "__main__":
    print("🔍 加载YOLOv8模型（树莓派专属优化）...")
    ort_session, input_name, output_name = load_optimized_model(MODEL_PATH)
    
    # 获取视频文件
    video_formats = ['.mp4', '.avi', '.MOV', '.mkv']
    video_files = []
    for fmt in video_formats:
        video_files.extend(glob.glob(os.path.join(VIDEO_DIR, f"*{fmt}")))
        video_files.extend(glob.glob(os.path.join(VIDEO_DIR, f"*{fmt.upper()}")))
    video_files = list(set(video_files))
    
    if not video_files:
        print("❌ 未找到视频文件")
        exit(1)
    
    # 批量处理
    print(f"\n📋 共发现 {len(video_files)} 个视频文件")
    for idx, video_path in enumerate(video_files, 1):
        print(f"\n========================================")
        print(f"处理进度：{idx}/{len(video_files)}")
        print(f"========================================")
        process_video_optimized(video_path, ort_session, input_name, output_name)
    
    print(f"\n🎉 所有视频处理完成！结果目录：{RESULT_DIR}")
    