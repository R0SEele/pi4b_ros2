import rclpy
from rclpy.node import Node
import cv2
import onnxruntime as ort
import numpy as np
from std_msgs.msg import String
import os
import multiprocessing
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None


class YoloV8OnnxNode(Node):
    def __init__(self):
        super().__init__('yolov8_onnx_node')

        self.publisher_ = self.create_publisher(String, 'yolov8_detections', 10)

        self.declare_parameter('model_path', '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/models/best.onnx')
        self.declare_parameter('video_dir', '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/data/vedio')
        self.declare_parameter('results_dir', '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results')
        self.declare_parameter('detection_file', '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results/scenic_spot_detections.txt')
        self.declare_parameter('config_path', '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/config/params.yaml')
        self.declare_parameter('frame_interval', 50)
        self.declare_parameter('save_only_with_detections', True)
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('intra_op_num_threads', max(1, multiprocessing.cpu_count() - 1))

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.video_dir = self.get_parameter('video_dir').get_parameter_value().string_value
        self.results_dir = self.get_parameter('results_dir').get_parameter_value().string_value
        self.detection_file = self.get_parameter('detection_file').get_parameter_value().string_value
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.frame_interval = int(self.get_parameter('frame_interval').get_parameter_value().integer_value)
        self.save_only_with_detections = bool(self.get_parameter('save_only_with_detections').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.intra_op_num_threads = int(self.get_parameter('intra_op_num_threads').value)
        if self.frame_interval <= 0:
            self.frame_interval = 50
        if self.jpeg_quality < 1 or self.jpeg_quality > 100:
            self.jpeg_quality = 90
        if self.intra_op_num_threads <= 0:
            self.intra_op_num_threads = 1

        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        sess_options.intra_op_num_threads = self.intra_op_num_threads
        sess_options.inter_op_num_threads = 1
        self.ort_session = ort.InferenceSession(self.model_path, sess_options=sess_options)
        self.input_name = self.ort_session.get_inputs()[0].name

        self.conf_threshold = 0.35
        self.nms_threshold = 0.45
        self.person_class_id = 0

        os.makedirs(self.results_dir, exist_ok=True)
        os.makedirs(os.path.dirname(self.detection_file), exist_ok=True)

        self.scenic_map = self._load_scenic_map(self.config_path)
        self.level_rules = self._load_level_rules(self.config_path)
        self.get_logger().info(
            f"YOLO加速参数: frame_interval={self.frame_interval}, save_only_with_detections={self.save_only_with_detections}, "
            f"jpeg_quality={self.jpeg_quality}, intra_threads={self.intra_op_num_threads}"
        )

    def _load_scenic_map(self, config_path):
        scenic_map = {}
        if yaml is None:
            self.get_logger().warn('未安装PyYAML，无法读取景点配置，改用文件名顺序映射。')
            return scenic_map
        if not os.path.exists(config_path):
            self.get_logger().warn(f'配置文件不存在: {config_path}，改用文件名顺序映射。')
            return scenic_map

        with open(config_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}

        scenic_spots = cfg.get('scenic_spots', {})
        for raw_id, spot_cfg in scenic_spots.items():
            try:
                spot_id = int(raw_id)
            except (TypeError, ValueError):
                continue
            video_path = str(spot_cfg.get('video_path', ''))
            video_name = Path(video_path).stem
            if not video_name:
                continue
            scenic_map[video_name] = {
                'spot_id': spot_id,
                'distance': float(spot_cfg.get('distance', 1.0)),
            }
        return scenic_map

    def _load_level_rules(self, config_path):
        # 默认等级规则与params.yaml保持一致。
        default_rules = {
            'C': {'min': 0, 'max': 7, 'score': 1},
            'B': {'min': 8, 'max': 15, 'score': 2},
            'A': {'min': 15, 'max': float('inf'), 'score': 3},
        }
        if yaml is None or not os.path.exists(config_path):
            return default_rules
        with open(config_path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        raw_rules = cfg.get('person_levels', {})
        if not raw_rules:
            return default_rules

        rules = {}
        for level_name, info in raw_rules.items():
            if not isinstance(info, dict):
                continue
            rules[str(level_name)] = {
                'min': float(info.get('min', 0)),
                'max': float(info.get('max', float('inf'))),
                'score': int(info.get('score', 1)),
            }
        return rules or default_rules

    def _person_level(self, people_count):
        for level_name, rule in sorted(self.level_rules.items(), key=lambda x: x[1].get('score', 0)):
            min_v = float(rule.get('min', 0))
            max_v = float(rule.get('max', float('inf')))
            if people_count >= min_v and people_count <= max_v:
                return level_name

        best_level = max(self.level_rules.items(), key=lambda x: x[1].get('score', 0))[0]
        return best_level

    def preprocess(self, frame):
        resized = cv2.resize(frame, (640, 640))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        chw = rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
        return np.expand_dims(chw, axis=0)

    def postprocess(self, output, frame_shape):
        h, w = frame_shape[:2]

        pred = output[0]
        if pred.ndim != 2:
            return []

        # YOLOv8 ONNX 常见输出: [C, N]，转成 [N, C] 方便处理。
        if pred.shape[0] <= pred.shape[1]:
            pred = pred.T

        channels = pred.shape[1]
        if channels < 5:
            return []

        boxes = []
        confidences = []

        for det in pred:
            cx, cy, bw, bh = det[:4]

            if channels == 5:
                class_id = self.person_class_id
                confidence = float(det[4])
            else:
                class_scores = det[4:]
                class_id = int(np.argmax(class_scores))
                confidence = float(class_scores[class_id])

            if confidence < self.conf_threshold:
                continue
            if class_id != self.person_class_id:
                continue

            x1 = int((cx - bw / 2.0) * w / 640.0)
            y1 = int((cy - bh / 2.0) * h / 640.0)
            x2 = int((cx + bw / 2.0) * w / 640.0)
            y2 = int((cy + bh / 2.0) * h / 640.0)

            x1 = max(0, min(x1, w - 1))
            y1 = max(0, min(y1, h - 1))
            x2 = max(0, min(x2, w - 1))
            y2 = max(0, min(y2, h - 1))

            if x2 <= x1 or y2 <= y1:
                continue

            boxes.append([x1, y1, x2 - x1, y2 - y1])
            confidences.append(confidence)

        if not boxes:
            return []

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)
        if len(indices) == 0:
            return []

        detections = []
        for idx in np.array(indices).reshape(-1):
            x, y, bw, bh = boxes[idx]
            detections.append((x, y, x + bw, y + bh, confidences[idx]))

        return detections

    def process_single_video(self, video_path, out_dir):
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            self.get_logger().error(f'无法打开视频文件: {video_path}')
            return 0

        frame_count = 0
        max_people_count = 0

        while True:
            grabbed = cap.grab()
            if not grabbed:
                break

            frame_count += 1
            if frame_count % self.frame_interval != 0:
                continue

            ret, frame = cap.retrieve()
            if not ret:
                break

            input_tensor = self.preprocess(frame)
            outputs = self.ort_session.run(None, {self.input_name: input_tensor})
            detections = self.postprocess(outputs[0], frame.shape)

            people_count = len(detections)
            max_people_count = max(max_people_count, people_count)

            for x1, y1, x2, y2, confidence in detections:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f'Person: {confidence:.2f}',
                    (x1, max(0, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

            cv2.putText(
                frame,
                f'Frame: {frame_count} People: {people_count} Max: {max_people_count}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
            )

            should_save = (people_count > 0) if self.save_only_with_detections else True
            if should_save:
                out_file = os.path.join(out_dir, f'frame_{frame_count}.jpg')
                cv2.imwrite(out_file, frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])

        cap.release()
        self.get_logger().info(f'视频处理完成: {video_path}, 最大人数: {max_people_count}')
        return max_people_count

    def process_all_videos(self):
        exts = ('.mp4', '.avi', '.mov', '.mkv')
        if not os.path.isdir(self.video_dir):
            self.get_logger().error(f'视频目录不存在: {self.video_dir}')
            return

        videos = sorted(
            [
                os.path.join(self.video_dir, name)
                for name in os.listdir(self.video_dir)
                if os.path.isfile(os.path.join(self.video_dir, name)) and name.lower().endswith(exts)
            ]
        )

        if not videos:
            self.get_logger().error(f'在目录中未找到视频文件: {self.video_dir}')
            return

        results = []
        for idx, video_path in enumerate(videos, start=1):
            video_name = Path(video_path).stem
            per_video_dir = os.path.join(self.results_dir, f'{video_name}_result')
            os.makedirs(per_video_dir, exist_ok=True)

            max_people = self.process_single_video(video_path, per_video_dir)
            scenic_cfg = self.scenic_map.get(video_name, {'spot_id': idx, 'distance': 1.0})
            level = self._person_level(max_people)

            results.append(
                {
                    'spot_id': scenic_cfg['spot_id'],
                    'count': max_people,
                    'level': level,
                    'distance': scenic_cfg['distance'],
                    'video_name': video_name,
                }
            )

            msg = String()
            msg.data = (
                f"video={video_name}, spot={scenic_cfg['spot_id']}, count={max_people}, "
                f"level={level}, distance={scenic_cfg['distance']}"
            )
            self.publisher_.publish(msg)

        results.sort(key=lambda x: x['spot_id'])
        with open(self.detection_file, 'w', encoding='utf-8') as f:
            for item in results:
                f.write(
                    f"{item['spot_id']},{item['count']},{item['level']},{item['distance']},{item['video_name']}\n"
                )

        self.get_logger().info(f'检测结果已保存至: {self.detection_file}')

    def shutdown(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = YoloV8OnnxNode()
    try:
        node.process_all_videos()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()