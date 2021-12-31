/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include <chrono>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/local_trajectory_builder_options_2d.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// TODO(gaschler): Add test for this class similar to the 3D test.
//它有扫描匹配器(Scan Matiching)，以激光雷达的扫描数据和位姿估计为输入， 使用Ceres库完成扫描匹配，输出位姿的观测值。
//一方面反馈给位姿估计器用于修正估计值，另一方面提供给运动滤波器(Motion Filter)用于判定机器人是否产生了运动。 
//如果产生了运动，则将此时的点云数据插入到当前正在维护的子图中。同时输出插入的结果，包括时间、位姿、子图、扫描数据等信息。
class LocalTrajectoryBuilder2D {
 public:
  // 将点云插入到地图后的result
  struct InsertionResult {
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps; // 最多只有2个子图的指针
  };
  // 扫描匹配的result
  struct MatchingResult {
    common::Time time; //扫描匹配发生的时间(time)
    transform::Rigid3d local_pose; // 在局部地图坐标系下的位姿(local_pose)
    sensor::RangeData range_data_in_local; // 经过扫描匹配之后位姿校准之后的雷达数据
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result; //子图插入结果(insertion_result)
  };

  explicit LocalTrajectoryBuilder2D(
      const proto::LocalTrajectoryBuilderOptions2D& options,
      const std::vector<std::string>& expected_range_sensor_ids);
  ~LocalTrajectoryBuilder2D();

  LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D&) = delete;
  LocalTrajectoryBuilder2D& operator=(const LocalTrajectoryBuilder2D&) = delete;

  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'. Range data must be approximately horizontal
  // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  std::unique_ptr<MatchingResult> AddRangeData( // 用于处理雷达数据
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  void AddImuData(const sensor::ImuData& imu_data); // 用于处理IMU数据
  void AddOdometryData(const sensor::OdometryData& odometry_data); // 用于处理里程计数据

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData( // 函数AddAccumulatedRangeData会调用ScanMatch进行扫描匹配，并通过函数InsertIntoSubmap插入数据更新子图
      common::Time time, const sensor::RangeData& gravity_aligned_range_data,
      const transform::Rigid3d& gravity_alignment,
      const absl::optional<common::Duration>& sensor_duration);
  sensor::RangeData TransformToGravityAlignedFrameAndFilter( // 用于根据重力加速度的方向来估计激光扫描数据在水平面上的投影
      const transform::Rigid3f& transform_to_gravity_aligned_frame,
      const sensor::RangeData& range_data) const;
  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& range_data_in_local,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
  // observed pose, or nullptr on failure.
  std::unique_ptr<transform::Rigid2d> ScanMatch(
      common::Time time, const transform::Rigid2d& pose_prediction,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud);

  // Lazily constructs a PoseExtrapolator.
  void InitializeExtrapolator(common::Time time);

  const proto::LocalTrajectoryBuilderOptions2D options_; //	轨迹跟踪器的配置选项
  ActiveSubmaps2D active_submaps_; // 当前正在维护的子图

  MotionFilter motion_filter_; //运动滤波器，对位姿相关的数据进行降采样
  scan_matching::RealTimeCorrelativeScanMatcher2D //实时相关性分析的扫描匹配器，算法"Real-Time Correlative Scan Matching"的实现
      real_time_correlative_scan_matcher_;
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_; //使用Ceres库将扫描数据放置到地图中的扫描匹配器

  std::unique_ptr<PoseExtrapolator> extrapolator_; //位姿估计器，用一段时间内的位姿数据估计线速度和角速度，进而预测运动

  int num_accumulated_ = 0; //	累积数据的数量
  sensor::RangeData accumulated_range_data_; //累积的扫描数据

  absl::optional<std::chrono::steady_clock::time_point> last_wall_time_; //开始累积数据的时间，也是开始跟踪轨迹的时间
  absl::optional<double> last_thread_cpu_time_seconds_; 
  absl::optional<common::Time> last_sensor_time_;

  RangeDataCollator range_data_collator_; //累积数据收集器
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
