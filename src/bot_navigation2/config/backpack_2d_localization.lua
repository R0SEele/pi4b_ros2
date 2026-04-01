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

include "backpack_2d.lua"

-- 限制保存的子图数量以降低内存和CPU
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,          -- 从3降低到2
}

-- 进一步降低全局定位的计算负载
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75  -- 提高阈值减少误匹配

return options