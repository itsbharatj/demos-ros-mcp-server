/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2024, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "velodyne_gazebo_plugins/VelodyneLidarSystem.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/msgs/pointcloud_packed.pb.h>
#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace velodyne_gazebo_plugins
{

/// \brief Private implementation class
class VelodyneLidarSystem::Implementation
{
public:
  /// \brief Gazebo transport node
  ignition::transport::Node gz_node;

  /// \brief ROS 2 node
  rclcpp::Node::SharedPtr ros_node{nullptr};

  /// \brief ROS 2 publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ros_pub{nullptr};

  /// \brief Gazebo topic name
  std::string gz_topic{"lidar"};

  /// \brief ROS 2 topic name
  std::string ros_topic{"/velodyne_points"};

  /// \brief Frame ID for the point cloud
  std::string frame_id{"velodyne"};

  /// \brief Minimum range to publish
  double min_range{0.0};

  /// \brief Maximum range to publish
  double max_range{std::numeric_limits<double>::infinity()};

  /// \brief Gaussian noise standard deviation
  double gaussian_noise{0.0};

  /// \brief Minimum intensity threshold
  double min_intensity{std::numeric_limits<double>::lowest()};

  /// \brief Mutex for thread safety
  std::mutex mutex;

  /// \brief Random number generator for Gaussian noise
  std::default_random_engine generator;

  /// \brief Whether the system has been initialized
  bool initialized{false};

  /// \brief Callback for Gazebo lidar messages
  void OnLidarMsg(const ignition::msgs::PointCloudPacked & _msg);

  /// \brief Generate Gaussian noise
  double GaussianNoise(double mean, double stddev);
};

//////////////////////////////////////////////////
double VelodyneLidarSystem::Implementation::GaussianNoise(
  double mean, double stddev)
{
  std::normal_distribution<double> distribution(mean, stddev);
  return distribution(generator);
}

//////////////////////////////////////////////////
void VelodyneLidarSystem::Implementation::OnLidarMsg(
  const ignition::msgs::PointCloudPacked & _msg)
{
  std::lock_guard<std::mutex> lock(mutex);

  if (!ros_node || !ros_pub) {
    return;
  }

  // Create ROS 2 PointCloud2 message
  auto ros_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  ros_msg->header.stamp.sec = _msg.header().stamp().sec();
  ros_msg->header.stamp.nanosec = _msg.header().stamp().nsec();
  ros_msg->header.frame_id = frame_id;

  // Set up point cloud fields (Velodyne format: x, y, z, intensity, ring)
  const uint32_t POINT_STEP = 32;
  ros_msg->height = 1;
  ros_msg->point_step = POINT_STEP;
  ros_msg->is_bigendian = false;
  ros_msg->is_dense = true;

  // Define fields
  ros_msg->fields.resize(5);
  
  ros_msg->fields[0].name = "x";
  ros_msg->fields[0].offset = 0;
  ros_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  ros_msg->fields[0].count = 1;
  
  ros_msg->fields[1].name = "y";
  ros_msg->fields[1].offset = 4;
  ros_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  ros_msg->fields[1].count = 1;
  
  ros_msg->fields[2].name = "z";
  ros_msg->fields[2].offset = 8;
  ros_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  ros_msg->fields[2].count = 1;
  
  ros_msg->fields[3].name = "intensity";
  ros_msg->fields[3].offset = 16;
  ros_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  ros_msg->fields[3].count = 1;
  
  ros_msg->fields[4].name = "ring";
  ros_msg->fields[4].offset = 20;
  ros_msg->fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
  ros_msg->fields[4].count = 1;

  // Reserve space for points
  size_t num_points = _msg.width() * _msg.height();
  ros_msg->data.resize(num_points * POINT_STEP);

  // Find field offsets in the input message
  int x_offset = -1, y_offset = -1, z_offset = -1;
  int intensity_offset = -1, ring_offset = -1;
  
  for (const auto & field : _msg.field()) {
    if (field.name() == "x") x_offset = field.offset();
    else if (field.name() == "y") y_offset = field.offset();
    else if (field.name() == "z") z_offset = field.offset();
    else if (field.name() == "intensity") intensity_offset = field.offset();
    else if (field.name() == "ring") ring_offset = field.offset();
  }

  // Process points
  const uint8_t * src_ptr = reinterpret_cast<const uint8_t *>(_msg.data().data());
  uint8_t * dst_ptr = ros_msg->data.data();
  size_t valid_points = 0;

  for (size_t i = 0; i < num_points; ++i) {
    const uint8_t * point_ptr = src_ptr + i * _msg.point_step();
    
    // Extract x, y, z
    float x = 0.0f, y = 0.0f, z = 0.0f;
    if (x_offset >= 0) {
      x = *reinterpret_cast<const float *>(point_ptr + x_offset);
    }
    if (y_offset >= 0) {
      y = *reinterpret_cast<const float *>(point_ptr + y_offset);
    }
    if (z_offset >= 0) {
      z = *reinterpret_cast<const float *>(point_ptr + z_offset);
    }

    // Calculate range
    double range = std::sqrt(x * x + y * y + z * z);

    // Extract intensity
    float intensity = 0.0f;
    if (intensity_offset >= 0) {
      intensity = *reinterpret_cast<const float *>(point_ptr + intensity_offset);
    }

    // Extract ring
    uint16_t ring = 0;
    if (ring_offset >= 0) {
      ring = *reinterpret_cast<const uint16_t *>(point_ptr + ring_offset);
    }

    // Apply filters
    if (range < min_range || range > max_range) {
      continue;
    }
    if (intensity < min_intensity) {
      continue;
    }

    // Add Gaussian noise to range if configured
    if (gaussian_noise > 0.0) {
      double noise = GaussianNoise(0.0, gaussian_noise);
      double scale = (range + noise) / range;
      x *= static_cast<float>(scale);
      y *= static_cast<float>(scale);
      z *= static_cast<float>(scale);
    }

    // Write point to output
    uint8_t * out_point = dst_ptr + valid_points * POINT_STEP;
    *reinterpret_cast<float *>(out_point + 0) = x;
    *reinterpret_cast<float *>(out_point + 4) = y;
    *reinterpret_cast<float *>(out_point + 8) = z;
    *reinterpret_cast<float *>(out_point + 16) = intensity;
    *reinterpret_cast<uint16_t *>(out_point + 20) = ring;
    
    valid_points++;
  }

  // Resize to actual number of valid points
  ros_msg->width = static_cast<uint32_t>(valid_points);
  ros_msg->row_step = ros_msg->width * POINT_STEP;
  ros_msg->data.resize(valid_points * POINT_STEP);

  // Publish
  ros_pub->publish(std::move(ros_msg));
}

//////////////////////////////////////////////////
VelodyneLidarSystem::VelodyneLidarSystem()
: impl_(std::make_unique<Implementation>())
{
}

//////////////////////////////////////////////////
VelodyneLidarSystem::~VelodyneLidarSystem() = default;

//////////////////////////////////////////////////
void VelodyneLidarSystem::Configure(
  const ignition::gazebo::Entity & /*_entity*/,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & /*_ecm*/,
  ignition::gazebo::EventManager & /*_eventMgr*/)
{
  // Read parameters from SDF
  if (_sdf->HasElement("ros_topic")) {
    impl_->ros_topic = _sdf->Get<std::string>("ros_topic");
  }
  
  if (_sdf->HasElement("gz_topic")) {
    impl_->gz_topic = _sdf->Get<std::string>("gz_topic");
  }
  
  if (_sdf->HasElement("frame_id")) {
    impl_->frame_id = _sdf->Get<std::string>("frame_id");
  }
  
  if (_sdf->HasElement("min_range")) {
    impl_->min_range = _sdf->Get<double>("min_range");
  }
  
  if (_sdf->HasElement("max_range")) {
    impl_->max_range = _sdf->Get<double>("max_range");
  }
  
  if (_sdf->HasElement("gaussian_noise")) {
    impl_->gaussian_noise = _sdf->Get<double>("gaussian_noise");
  }
  
  if (_sdf->HasElement("min_intensity")) {
    impl_->min_intensity = _sdf->Get<double>("min_intensity");
  }

  // Initialize ROS 2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  impl_->ros_node = std::make_shared<rclcpp::Node>("velodyne_lidar_system");
  impl_->ros_pub = impl_->ros_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    impl_->ros_topic, rclcpp::SensorDataQoS());

  // Subscribe to Gazebo lidar topic
  impl_->gz_node.Subscribe(
    impl_->gz_topic,
    &VelodyneLidarSystem::Implementation::OnLidarMsg,
    impl_.get());

  RCLCPP_INFO(
    impl_->ros_node->get_logger(),
    "VelodyneLidarSystem: Subscribing to Gazebo topic '%s', publishing to ROS topic '%s'",
    impl_->gz_topic.c_str(), impl_->ros_topic.c_str());

  impl_->initialized = true;
}

//////////////////////////////////////////////////
void VelodyneLidarSystem::PreUpdate(
  const ignition::gazebo::UpdateInfo & /*_info*/,
  ignition::gazebo::EntityComponentManager & /*_ecm*/)
{
  // Nothing to do in PreUpdate
}

//////////////////////////////////////////////////
void VelodyneLidarSystem::PostUpdate(
  const ignition::gazebo::UpdateInfo & /*_info*/,
  const ignition::gazebo::EntityComponentManager & /*_ecm*/)
{
  // Process ROS 2 callbacks
  if (impl_->ros_node) {
    rclcpp::spin_some(impl_->ros_node);
  }
}

}  // namespace velodyne_gazebo_plugins

// Register the plugin
IGNITION_ADD_PLUGIN(
  velodyne_gazebo_plugins::VelodyneLidarSystem,
  ignition::gazebo::System,
  velodyne_gazebo_plugins::VelodyneLidarSystem::ISystemConfigure,
  velodyne_gazebo_plugins::VelodyneLidarSystem::ISystemPreUpdate,
  velodyne_gazebo_plugins::VelodyneLidarSystem::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  velodyne_gazebo_plugins::VelodyneLidarSystem,
  "velodyne_gazebo_plugins::VelodyneLidarSystem")
