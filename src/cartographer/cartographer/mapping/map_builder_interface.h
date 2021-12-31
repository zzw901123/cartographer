/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_

#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// This interface is used for both library and RPC implementations.
// Implementations wire up the complete SLAM stack.
class MapBuilderInterface {
 public:
  using LocalSlamResultCallback =
      TrajectoryBuilderInterface::LocalSlamResultCallback;

  using SensorId = TrajectoryBuilderInterface::SensorId;

  MapBuilderInterface() {}
  virtual ~MapBuilderInterface() {}

  MapBuilderInterface(const MapBuilderInterface&) = delete;
  MapBuilderInterface& operator=(const MapBuilderInterface&) = delete;

  // Creates a new trajectory builder and returns its index. 
  virtual int AddTrajectoryBuilder( 
      const std::set<SensorId>& expected_sensor_ids, //expected_sensor_ids中记录了用于建图的所有传感器名称和类型 
      const proto::TrajectoryBuilderOptions& trajectory_options, // 新建的轨迹跟踪器的配置
      LocalSlamResultCallback local_slam_result_callback) = 0; //local_slam_result_callback则是一个回调函数对象，用于响应局部地图构建完成的事件。

  // Creates a new trajectory and returns its index. Querying the trajectory 也是一个用于新建轨迹跟踪器的接口。
  // builder for it will return 'nullptr'.
  virtual int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds&
          options_with_sensor_ids_proto) = 0; // 记录了轨迹跟踪器的配置还包含传感器的配置

  // Returns the 'TrajectoryBuilderInterface' corresponding to the specified 获取一个索引为trajectory_id的轨迹跟踪器对象。
  // 'trajectory_id' or 'nullptr' if the trajectory has no corresponding
  // builder.
  virtual mapping::TrajectoryBuilderInterface* GetTrajectoryBuilder(
      int trajectory_id) const = 0; //索引为trajectory_id的轨迹跟踪器对象

  // Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished, 关闭trajectory_id对应的轨迹跟踪器。
  // i.e. no further sensor data is expected.
  virtual void FinishTrajectory(int trajectory_id) = 0; //FinishTrajectory用于关闭trajectory_id对应的轨迹跟踪器

  // Fills the SubmapQuery::Response corresponding to 'submap_id'. Returns an 将submap_id所对应的子图信息填充到proto流中。
  // error string on failure, or an empty string on success.
  virtual std::string SubmapToProto(const SubmapId& submap_id,
                                    proto::SubmapQuery::Response* response) = 0;

  // Serializes the current state to a proto stream. If 将系统状态转换为proto流，进行序列化。
  // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included in the serialized state.
  virtual void SerializeState(bool include_unfinished_submaps,
                              io::ProtoStreamWriterInterface* writer) = 0;

  // Serializes the current state to a proto stream file on the host system. If
  // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included in the serialized state.
  // Returns true if the file was successfully written.
  virtual bool SerializeStateToFile(bool include_unfinished_submaps,
                                    const std::string& filename) = 0;

  // Loads the SLAM state from a proto stream. Returns the remapping of new 从proto流中加载系统状态。
  // trajectory_ids.
  virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
  LoadState(io::ProtoStreamReaderInterface* reader, bool load_frozen_state) = 0;

  // Loads the SLAM state from a pbstream file. Returns the remapping of new 
  // trajectory_ids.
  virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
  LoadStateFromFile(const std::string& filename, bool load_frozen_state) = 0;

  virtual int num_trajectory_builders() const = 0; //获取当前轨迹跟踪器的数量。

  virtual mapping::PoseGraphInterface* pose_graph() = 0; // 获取用于实现闭环检测的PoseGraph对象。

  virtual const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>& // 获取所有的轨迹跟踪器的配置。
  GetAllTrajectoryBuilderOptions() const = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
