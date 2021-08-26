// Copyright 2021 Tier IV, Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <vector>

#include "common/types.hpp"
#include "outlier_filter_nodes/ring_filter_node.hpp"

#include "pcl_conversions/pcl_conversions.h"


namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter_nodes
{

using float64_t = autoware::common::types::float64_t;
using float32_t = autoware::common::types::float32_t;

using RingFilter =
  autoware::perception::filters::outlier_filter::ring_filter::
  RingFilter;

RingFilterNode::RingFilterNode(const rclcpp::NodeOptions & options)
: FilterNodeBase("ring_filter_node", options),
  distance_ratio_(declare_parameter("distance_ratio").get<float64_t>()),
  object_length_threshold_(declare_parameter("object_length_threshold").get<float64_t>()),
  num_points_threshold_(declare_parameter("num_points_threshold").get<std::int64_t>())
{
  ring_filter_ = std::make_shared<RingFilter>(
    distance_ratio_,
    object_length_threshold_,
    num_points_threshold_
  );

  set_param_callback();
}

void RingFilterNode::filter(
  const sensor_msgs::msg::PointCloud2 &,
  sensor_msgs::msg::PointCloud2 & output)
{
  pcl::PointCloud<common::types::PointXYZIF> pcl_input;
  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  // pcl::fromROSMsg(input, pcl_input);

  // Perform filtering
  ring_filter_->filter(pcl_input, pcl_output);

  pcl::toROSMsg(pcl_output, output);
}

rcl_interfaces::msg::SetParametersResult RingFilterNode::get_node_parameters(
  const std::vector<rclcpp::Parameter> & p)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  {
    using namespace autoware::perception::filters::filter_node_base; //NOLINT

    if (get_param<float64_t>(p, "distance_ratio", distance_ratio_)) {
      result.successful = false;
      result.reason += "Failed to retrieve distance_ratio parameter. ";
      RCLCPP_DEBUG(get_logger(), "Setting new distance ratio to: %f.", distance_ratio_);
    }
    if (get_param<float64_t>(p, "object_length_threshold", object_length_threshold_)) {
      result.successful = false;
      result.reason += "Failed to retrieve object_length_threshold parameter. ";
      RCLCPP_DEBUG(
        get_logger(), "Setting new object length threshold to: %f.",
        object_length_threshold_);
    }
    if (get_param<std::int64_t>(p, "num_points_threshold", num_points_threshold_)) {
      result.successful = false;
      result.reason += "Failed to retrieve num_points_threshold parameter. ";
      RCLCPP_DEBUG(get_logger(), "Setting new points threshold to: %f.", num_points_threshold_);
    }
  }

  // Call update method in filter class object
  // ring_filter_->update_parameters(search_radius_, static_cast<int>(min_neighbors_));

  return result;
}

}  // namespace outlier_filter_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::outlier_filter_nodes::RingFilterNode)
