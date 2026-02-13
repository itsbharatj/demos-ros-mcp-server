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

#ifndef VELODYNE_GAZEBO_PLUGINS__VELODYNE_LIDAR_SYSTEM_HPP_
#define VELODYNE_GAZEBO_PLUGINS__VELODYNE_LIDAR_SYSTEM_HPP_

#include <memory>
#include <string>

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace velodyne_gazebo_plugins
{

/// \brief Gazebo Fortress system plugin for Velodyne lidar sensors.
/// 
/// This plugin subscribes to Gazebo lidar data and publishes it as ROS 2
/// PointCloud2 messages with the Velodyne-specific field layout (x, y, z,
/// intensity, ring).
///
/// Note: For most use cases, the built-in gpu_lidar sensor with ros_gz_bridge
/// is sufficient. This plugin provides additional customization options.
///
/// Plugin parameters (SDF):
/// - <ros_topic>: ROS 2 topic name for PointCloud2 output
/// - <gz_topic>: Gazebo topic name for lidar input
/// - <frame_id>: TF frame ID for the point cloud
/// - <min_range>: Minimum range to publish (meters)
/// - <max_range>: Maximum range to publish (meters)
/// - <gaussian_noise>: Gaussian noise standard deviation (meters)
/// - <min_intensity>: Minimum intensity threshold for filtering
class VelodyneLidarSystem
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate
{
public:
  /// \brief Constructor
  VelodyneLidarSystem();

  /// \brief Destructor
  ~VelodyneLidarSystem() override;

  /// \brief Configure the system
  /// \param[in] _entity Entity this plugin is attached to
  /// \param[in] _sdf SDF element containing plugin configuration
  /// \param[in] _ecm Entity component manager
  /// \param[in] _eventMgr Event manager
  void Configure(
    const ignition::gazebo::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager & _eventMgr) override;

  /// \brief Pre-update callback
  /// \param[in] _info Simulation update info
  /// \param[in] _ecm Entity component manager
  void PreUpdate(
    const ignition::gazebo::UpdateInfo & _info,
    ignition::gazebo::EntityComponentManager & _ecm) override;

  /// \brief Post-update callback
  /// \param[in] _info Simulation update info
  /// \param[in] _ecm Entity component manager (const)
  void PostUpdate(
    const ignition::gazebo::UpdateInfo & _info,
    const ignition::gazebo::EntityComponentManager & _ecm) override;

private:
  /// \brief Private implementation
  class Implementation;
  std::unique_ptr<Implementation> impl_;
};

}  // namespace velodyne_gazebo_plugins

#endif  // VELODYNE_GAZEBO_PLUGINS__VELODYNE_LIDAR_SYSTEM_HPP_
