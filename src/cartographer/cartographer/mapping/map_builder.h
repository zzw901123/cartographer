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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"

namespace cartographer {
namespace mapping {

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
// 包含前端(TrajectoryBuilders,scan to submap) 与 后端(用于查找回环的PoseGraph) 的完整的SLAM

class MapBuilder : public MapBuilderInterface {
 public:
  explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder() override {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  // 最主要关心的是AddTrajectoryBuilder因为它创建了一个新的轨迹跟踪器，用于进行局部的SLAM。其次关心FinishTrajectory，它用于结束一段轨迹。
  // 创建一个新的轨迹跟踪器并返回该跟踪器的索引。
  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;

  void FinishTrajectory(int trajectory_id) override;

  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;

  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriterInterface *writer) override;

  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename) override;

  std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                               bool load_frozen_state) override;

  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       const bool load_frozen_state) override;

  // 返回一个PoseGraphInterface的接口指针
  mapping::PoseGraphInterface *pose_graph() override {
    return pose_graph_.get(); //unique_ptr的get函数可返回被管理对象的指针
  }

  // 返回系统中当前已有的trajectory_builder的数量
  int num_trajectory_builders() const override {
    return trajectory_builders_.size(); //向量的size即为TrajectoryBuilder的数量
  }

  // 根据trajectory_id返回一个TrajectoryBuilderInterface的指针。
  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const override {
    return trajectory_builders_.at(trajectory_id).get();
  }

  // 获取所有TrajectoryBuilder的配置项。
  const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const override {
    return all_trajectory_builder_options_;
  }

 private:
  const proto::MapBuilderOptions options_; // 用于记录运行配置，它使用了google的protobuf来处理结构化的数据
  common::ThreadPool thread_pool_; // 线程池

  std::unique_ptr<PoseGraph> pose_graph_; // 该对象用于在后台完成闭环检测，进行全局的地图优化  MapBuilder维护了一个PoseGraph的智能指针，该指针用来做Loop Closure

  std::unique_ptr<sensor::CollatorInterface> sensor_collator_; //应该是用来管理和收集传感器数据的
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_; //用于在前台构建子图。在系统运行的过程中，可能有不止一条轨迹，针对每一条轨迹Cartographer都建立了一个轨迹跟踪器 
                            //TrajectoryBuilder对应了机器人运行了一圈。这个向量列表就管理了整个图中的所有submap。
  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds> //记录了所有轨迹跟踪器的配置
      all_trajectory_builder_options_;
};

// 工厂函数
std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
    const proto::MapBuilderOptions& options);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
