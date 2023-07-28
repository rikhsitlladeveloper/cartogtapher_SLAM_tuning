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
  tracking_frame = "laser",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1,
  landmarks_sampling_ratio = 1.,
}

-------------
-- General --
-------------

MAX_3D_RANGE = 20. -- default 60.
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 8

----------------
-- Local SLAM --
----------------

-- a bit more than the distance between lidar at least
TRAJECTORY_BUILDER_3D.min_range = 1. -- default 1.
TRAJECTORY_BUILDER_3D.max_range = MAX_3D_RANGE
-- Cartographer wants to have 1 rotation (per lidar) in each node
-- Configured for VLP16 in strongest return mode at 600RPM
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15 -- default 0.15

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2. -- default 2.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150 -- default 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 20. -- default 15.

TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4. -- default 4.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200 -- default 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = MAX_3D_RANGE

-- Scan matcher option2: computationally expensive
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false -- default false
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.5 -- default 0.15
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.) -- default math.rad(1.)
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1 -- default 1e-1
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1 -- default 1e-1

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1. -- hdl32 10., high resolution, default 1.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6. -- hdl32 18., low resolution, default 6.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5. -- default 5.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2 -- default 4e2
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false -- default false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false -- default false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50 -- default 12
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 1 -- default 1

-- TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5 -- default 0.5
-- TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.5 -- default 0.1
-- TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(1.0) -- default 0.004=math.rad(0.22)

imu_gravity_time_constant = 100. -- default 10.
rotational_histogram_size = 60 -- default 120

TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.1 -- default 0.1
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20. -- default 20.
-- submap discretization size
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45 -- default 0.45
-- NUMBER OF data points used for a single submap
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160 -- default 160
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55 -- default 0.55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49 -- default 0.49
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels = 2 -- default 2

-----------------
-- Global SLAM --
-----------------

POSE_GRAPH.optimize_every_n_nodes = 25 -- default 90

-- turn down once working to reduce execution time by reducing points used
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0 -- default 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15. -- default 15.
-- quality factor for loop closer matches between submaps higher less closures
POSE_GRAPH.constraint_builder.min_score = 0.55 -- default 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6 -- default 0.6
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4 -- default 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5 -- default 1e5
POSE_GRAPH.constraint_builder.log_matches = true -- default true

-- POSE_GRAPH.constraint_builder.linear_search_window = 7. -- default 7.
-- POSE_GRAPH.constraint_builder.angular_search_window = math.rad(30.) -- default math.rad(30.)
-- POSE_GRAPH.constraint_builder.branch_and_bound_depth = 7 -- default 7

-- POSE_GRAPH.constraint_builder.occupied_space_weight = 20. -- default 20.
-- POSE_GRAPH.constraint_builder.translation_weight = 10. -- default 10.
-- POSE_GRAPH.constraint_builder.rotation_weight = 1. -- default 1.
-- POSE_GRAPH.constraint_builder.ceres_solver_options.use_nonmonotonic_steps = true -- default true
-- POSE_GRAPH.constraint_builder.ceres_solver_options.max_num_iterations = 10 -- default 10
-- POSE_GRAPH.constraint_builder.ceres_solver_options.num_threads = 1 -- default 1

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 8 -- default 8
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth = 3 -- default 3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.85 -- default 0.77
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.65 -- default 0.55
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 2. -- default 5.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 2. -- default 1.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(15.) -- default math.rad(15.)

POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_0 = 5. -- default 5.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_1 = 30. -- default 30.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 100. -- default 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 10. -- default 1.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = true -- default false
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.use_nonmonotonic_steps = false -- default false
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 10 -- default 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 7 -- default 1

POSE_GRAPH.matcher_translation_weight = 5e2 -- default 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3 -- default 1.6e3

POSE_GRAPH.optimization_problem.huber_scale = 1e1 -- default 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3 -- default 1e3, hdl32 1e-6
POSE_GRAPH.optimization_problem.rotation_weight = 3e5 -- default 3e5, hdl32 1e-6
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5 -- default 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5 -- default 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5 -- default 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5 -- default 1e5
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1 -- default 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2 -- default 1e2
POSE_GRAPH.optimization_problem.log_solver_summary = false -- default false
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false -- default false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50 -- default 50
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 7 -- default 7

POSE_GRAPH.max_num_final_iterations = 200 -- default 200
POSE_GRAPH.global_sampling_ratio = 0.003 -- default 0.003
POSE_GRAPH.log_residual_histograms = true -- default true
POSE_GRAPH.global_constraint_search_after_n_seconds = 2. -- default 10.


return options
