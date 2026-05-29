// Copyright 2009, 2010, 2011, 2012, 2019 Austin Robot Technology, Jack O'Quin, Jesse Vera, Sebastian Pütz, Joshua Whitley  // NOLINT
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "velodyne_pointcloud/transform.hpp"

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "velodyne_pointcloud/organized_cloudXYZIRT.hpp"
#include "velodyne_pointcloud/pointcloudXYZIRT.hpp"
#include "velodyne_pointcloud/rawdata.hpp"

namespace velodyne_pointcloud
{

Transform::Transform(const rclcpp::NodeOptions & options)
: rclcpp::Node("velodyne_transform_node", options),
  diagnostics_(this)
{
  std::string calibration_file = this->declare_parameter("calibration", "");
  const auto model = this->declare_parameter("model", "64E");

  rcl_interfaces::msg::ParameterDescriptor min_range_desc;
  min_range_desc.name = "min_range";
  min_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_range_desc.description = "minimum range to publish";
  rcl_interfaces::msg::FloatingPointRange min_range_range;
  min_range_range.from_value = 0.1;
  min_range_range.to_value = 10.0;
  min_range_desc.floating_point_range.push_back(min_range_range);
  double min_range = this->declare_parameter("min_range", 0.9, min_range_desc);

  rcl_interfaces::msg::ParameterDescriptor max_range_desc;
  max_range_desc.name = "max_range";
  max_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_range_desc.description = "maximum range to publish";
  rcl_interfaces::msg::FloatingPointRange max_range_range;
  max_range_range.from_value = 0.1;
  max_range_range.to_value = 300.0;
  max_range_desc.floating_point_range.push_back(max_range_range);
  double max_range = this->declare_parameter("max_range", 130.0, max_range_desc);

  rcl_interfaces::msg::ParameterDescriptor view_direction_desc;
  view_direction_desc.name = "view_direction";
  view_direction_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  view_direction_desc.description = "angle defining the center of view";
  rcl_interfaces::msg::FloatingPointRange view_direction_range;
  view_direction_range.from_value = -M_PI;
  view_direction_range.to_value = M_PI;
  view_direction_desc.floating_point_range.push_back(view_direction_range);
  double view_direction = this->declare_parameter("view_direction", 0.0, view_direction_desc);

  rcl_interfaces::msg::ParameterDescriptor view_width_desc;
  view_width_desc.name = "view_width";
  view_width_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  view_width_desc.description = "angle defining the view width";
  rcl_interfaces::msg::FloatingPointRange view_width_range;
  view_width_range.from_value = 0.0;
  view_width_range.to_value = 2.0 * M_PI;
  view_width_desc.floating_point_range.push_back(view_width_range);
  double view_width = this->declare_parameter("view_width", 2.0 * M_PI, view_width_desc);

  std::string fixed_frame = this->declare_parameter("fixed_frame", "");
  std::string target_frame = this->declare_parameter("target_frame", "");
  bool organize_cloud = this->declare_parameter("organize_cloud", true);
  bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
  crop_radius_max_ = this->declare_parameter("crop_radius_max", 0.0);
  crop_min_z_ = this->declare_parameter(
    "crop_min_z", -std::numeric_limits<double>::infinity());
  crop_max_z_ = this->declare_parameter(
    "crop_max_z", std::numeric_limits<double>::infinity());

  RCLCPP_INFO(this->get_logger(), "correction angles: %s", calibration_file.c_str());

  data_ = std::make_unique<velodyne_rawdata::RawData>(calibration_file, model);

  if (organize_cloud) {
    container_ptr_ = std::make_unique<OrganizedCloudXYZIRT>(
      min_range, max_range, target_frame, fixed_frame, data_->numLasers(),
      data_->scansPerPacket(), this->get_clock());
  } else {
    container_ptr_ = std::make_unique<PointcloudXYZIRT>(
      min_range, max_range, target_frame, fixed_frame,
      data_->scansPerPacket(), this->get_clock());
  }

  // advertise output point cloud (before subscribing to input data)
  rclcpp::QoS io_qos = use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(10);
  output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", io_qos);

  velodyne_scan_ = this->create_subscription<velodyne_msgs::msg::VelodyneScan>(
    "velodyne_packets", io_qos, std::bind(&Transform::processScan, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(), "velodyne I/O QoS profile: %s",
    use_sensor_data_qos ? "sensor_data_qos (best_effort)" : "default (reliable)");
  RCLCPP_INFO(
    this->get_logger(),
    "crop filter: radius<=%.3f (<=0:disabled), z:[%.3f, %.3f]",
    crop_radius_max_, crop_min_z_, crop_max_z_);

  // Diagnostics
  diagnostics_.setHardwareID("Velodyne Transform");
  // Arbitrary frequencies since we don't know which RPM is used, and are only
  // concerned about monitoring the frequency.
  diag_min_freq_ = 2.0;
  diag_max_freq_ = 20.0;
  diag_topic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
    "velodyne_points", diagnostics_, diagnostic_updater::FrequencyStatusParam(
      &diag_min_freq_, &diag_max_freq_, 0.1, 10),
    diagnostic_updater::TimeStampStatusParam());

  data_->setParameters(min_range, max_range, view_direction, view_width);
  container_ptr_->configure(min_range, max_range, target_frame, fixed_frame);
}

/** @brief Callback for raw scan messages.
 *
 *  @pre TF message filter has already waited until the transform to
 *       the configured @c frame_id can succeed.
 */
void Transform::processScan(
  const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr scanMsg)
{
  if (output_->get_subscription_count() == 0 &&
    output_->get_intra_process_subscription_count() == 0)    // no one listening?
  {
    return;
  }

  container_ptr_->setup(scanMsg);

  // sufficient to calculate single transform for whole scan
  if (!container_ptr_->computeTransformToTarget(scanMsg->header.stamp)) {
    // target frame not available
    return;
  }

  // process each packet provided by the driver
  for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
    // calculate individual transform for each packet to account for ego
    // during one rotation of the velodyne sensor
    if (!container_ptr_->computeTransformToFixed(scanMsg->packets[i].stamp)) {
      // fixed frame not available
      return;
    }
    data_->unpack(scanMsg->packets[i], *container_ptr_, scanMsg->header.stamp);
  }

  const auto & cloud = container_ptr_->finishCloud();
  const bool enable_radius_crop = crop_radius_max_ > 0.0;
  const bool enable_z_crop =
    std::isfinite(crop_min_z_) || std::isfinite(crop_max_z_);
  if (!enable_radius_crop && !enable_z_crop) {
    output_->publish(cloud);
    diag_topic_->tick(scanMsg->header.stamp);
    return;
  }

  const size_t total_points = static_cast<size_t>(cloud.width) * cloud.height;
  const size_t point_step = static_cast<size_t>(cloud.point_step);
  sensor_msgs::msg::PointCloud2 filtered_cloud = cloud;
  filtered_cloud.height = 1;
  filtered_cloud.width = 0;
  filtered_cloud.data.clear();
  filtered_cloud.is_dense = true;

  const double radius2_max = crop_radius_max_ * crop_radius_max_;
  size_t kept_points = 0;
  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  for (const auto & field : cloud.fields) {
    if (field.name == "x") {
      x_offset = static_cast<int>(field.offset);
    } else if (field.name == "y") {
      y_offset = static_cast<int>(field.offset);
    } else if (field.name == "z") {
      z_offset = static_cast<int>(field.offset);
    }
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Crop filter skipped (missing x/y/z field)");
    output_->publish(cloud);
    diag_topic_->tick(scanMsg->header.stamp);
    return;
  }

  if (point_step == 0 ||
    (static_cast<size_t>(x_offset) + sizeof(float) > point_step) ||
    (static_cast<size_t>(y_offset) + sizeof(float) > point_step) ||
    (static_cast<size_t>(z_offset) + sizeof(float) > point_step))
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Crop filter skipped (invalid point field offsets)");
    output_->publish(cloud);
    diag_topic_->tick(scanMsg->header.stamp);
    return;
  }

  filtered_cloud.data.resize(total_points * point_step);
  uint8_t * out_ptr = filtered_cloud.data.data();

  for (size_t i = 0; i < total_points; ++i) {
    const uint8_t * point_ptr = &cloud.data[i * point_step];
    float x_f = 0.0F;
    float y_f = 0.0F;
    float z_f = 0.0F;
    std::memcpy(&x_f, point_ptr + x_offset, sizeof(float));
    std::memcpy(&y_f, point_ptr + y_offset, sizeof(float));
    std::memcpy(&z_f, point_ptr + z_offset, sizeof(float));

    const double x = static_cast<double>(x_f);
    const double y = static_cast<double>(y_f);
    const double z = static_cast<double>(z_f);

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      filtered_cloud.is_dense = false;
      continue;
    }
    if (enable_radius_crop && ((x * x + y * y) > radius2_max)) {
      continue;
    }
    if (z < crop_min_z_ || z > crop_max_z_) {
      continue;
    }

    std::memcpy(out_ptr, point_ptr, point_step);
    out_ptr += point_step;
    ++kept_points;
  }

  filtered_cloud.data.resize(kept_points * point_step);
  if (filtered_cloud.data.empty() && kept_points == 0) {
    filtered_cloud.is_dense = false;
  }

  if (filtered_cloud.data.size() != kept_points * point_step) {
    RCLCPP_WARN(
      this->get_logger(),
      "Crop filter skipped (unexpected filtered cloud size)");
    output_->publish(cloud);
    diag_topic_->tick(scanMsg->header.stamp);
    return;
  }

  filtered_cloud.width = static_cast<uint32_t>(kept_points);
  filtered_cloud.row_step = filtered_cloud.width * filtered_cloud.point_step;
  output_->publish(filtered_cloud);
  diag_topic_->tick(scanMsg->header.stamp);
}

}  // namespace velodyne_pointcloud

RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_pointcloud::Transform)
