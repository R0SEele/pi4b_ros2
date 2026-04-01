-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  
  -- 降低发布频率以减少CPU占用
  submap_publish_period_sec = 1.0,           -- 从0.3增加到1.0
  pose_publish_period_sec = 0.05,            -- 从5e-3降低到0.05 (20Hz)
  trajectory_publish_period_sec = 0.1,       -- 从30e-3增加到0.1 (10Hz)
  
  -- 降低采样率以减少处理数据量
  rangefinder_sampling_ratio = 0.5,          -- 从1.0降低到0.5
  odometry_sampling_ratio = 0.5,             -- 从1.0降低到0.5
  fixed_frame_pose_sampling_ratio = 0.5,
  imu_sampling_ratio = 0,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2        -- 从4降低到2，减少线程开销

-- 降低子图构建频率
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30   -- 增加此值减少子图构建频率（默认通常是30-50）
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2  -- 从1增加到2，降低匹配频率

-- 激光雷达范围设置
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 25.0               -- 从30降低到25，减少点云数量
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25.0

-- 禁用IMU
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 扫描匹配参数优化（降低精度换取性能）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- 增加搜索窗口减少失败重试
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 8.0  -- 降低权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.8

-- 降低Ceres优化频率和精度
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.5      -- 降低优化权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.3
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0   -- 从默认20降低
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.3

-- 运动滤波：降低对微小运动的响应
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)   -- 增加角度阈值 (默认0.05)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.15          -- 增加距离阈值 (默认0.1)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5              -- 增加时间阈值 (默认0.5)

-- 点云处理优化
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05          -- 增大体素滤波尺寸，减少点数（默认0.025）
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1  -- 降低地图分辨率（默认0.05）

-- 纯定位模式配置
POSE_GRAPH.optimize_every_n_nodes = 50                  -- 从20增加到50，降低优化频率
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 约束构建优化
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3      -- 添加约束采样率
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0  -- 限制约束搜索距离
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e4

-- 优化线程数
POSE_GRAPH.max_num_final_iterations = 100               -- 减少优化迭代次数

return options