-- burger_cartographer.lua
-- ======================
-- Cartographer 2D SLAM configuration for the TurtleBot3 Burger.
--
-- WHAT CARTOGRAPHER DOES:
--   Cartographer uses a scan-matcher to align each new LiDAR scan against
--   a running submap. When the robot revisits a place, a loop closure
--   optimisation corrects the accumulated drift in the entire trajectory.
--
-- KEY TUNING PARAMETERS FOR THE BURGER:
--   LDS-01 LiDAR:  range 0.12 m – 3.5 m,  360° scan, ~1.8° resolution
--   Wheel base:    0.16 m separation,  0.033 m wheel radius
--   Max speed:     0.22 m/s linear,  2.84 rad/s angular
--
-- REFERENCE:
--   https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder                    = MAP_BUILDER,
  trajectory_builder             = TRAJECTORY_BUILDER,

  -- Frame IDs — must match our URDF and TF tree exactly.
  map_frame                      = "map",
  tracking_frame                 = "base_footprint",
  published_frame                = "base_footprint",
  odom_frame                     = "odom",

  -- Use our EKF odometry as the initial pose estimate between scans.
  -- Cartographer fuses this with scan matching for better accuracy.
  provide_odom_frame             = false,
  use_odometry                   = true,

  -- No IMU — we already fuse IMU into odometry via the EKF (Step 6).
  -- Cartographer's IMU integration would double-count it.
  use_imu_data                   = false,

  -- LDS-01 is a single 2D LiDAR — no multi-echo, no 3D.
  num_laser_scans                = 1,
  num_multi_echo_laser_scans     = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds               = 0,

  -- How often to publish the map→odom transform (seconds).
  lookup_transform_timeout_sec   = 0.2,
  submap_publish_period_sec      = 0.3,
  pose_publish_period_sec        = 5e-3,   -- 200 Hz pose output
  trajectory_publish_period_sec  = 30e-3,  -- 33 Hz trajectory
  rangefinder_sampling_ratio     = 1.0,
  odometry_sampling_ratio        = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio             = 1.0,
  landmarks_sampling_ratio       = 1.0,
}

-- ── 2D trajectory builder ─────────────────────────────────────────────────
-- These settings control how each new scan is processed and matched.

TRAJECTORY_BUILDER_2D.use_imu_data               = false
TRAJECTORY_BUILDER_2D.min_range                  = 0.12  -- LDS-01 minimum (metres)
TRAJECTORY_BUILDER_2D.max_range                  = 3.5   -- LDS-01 maximum (metres)
TRAJECTORY_BUILDER_2D.missing_data_ray_length    = 3.0   -- fill missing rays with this

-- Voxel filter reduces the point cloud before scan matching.
-- 0.05 m matches the map resolution — no point keeping finer detail.
TRAJECTORY_BUILDER_2D.voxel_filter_size          = 0.05

-- Adaptive voxel filter: target 200 points per scan for the matcher.
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length              = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points          = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range               = 50.0

-- Loop closure voxel filter (coarser — used for submap matching).
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range  = 50.0

-- Real-time correlative scan matcher: fast initial alignment.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching          = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window  = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- Ceres scan matcher: refines the pose found by the correlative matcher.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight      = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight         = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight            = 40.

-- Motion filter: only process a scan if the robot moved enough.
-- This avoids wasting CPU when the robot is stationary.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters             = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians               = math.rad(5.)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds                = 5.

-- Submap size: 40 scans per submap is a good balance for indoor rooms.
TRAJECTORY_BUILDER_2D.submaps.num_range_data                        = 40
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution            = 0.05

-- ── Global map builder ────────────────────────────────────────────────────
-- These settings control loop closure and the global optimisation.

MAP_BUILDER.use_trajectory_builder_2d = true

-- Loop closure: search for matching submaps when revisiting a place.
POSE_GRAPH.optimize_every_n_nodes                                    = 20
POSE_GRAPH.constraint_builder.min_score                              = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score         = 0.7
POSE_GRAPH.constraint_builder.sampling_ratio                        = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance               = 15.0
POSE_GRAPH.global_sampling_ratio                                     = 0.003
POSE_GRAPH.log_residual_histograms                                   = false

return options
