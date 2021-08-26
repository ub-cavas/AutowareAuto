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

/// \copyright Copyright 2021 Tier IV, Inc
/// \file
/// \brief This file defines the RingFilterNode class.

#ifndef OUTLIER_FILTER_NODES__RING_FILTER_NODE_HPP_
#define OUTLIER_FILTER_NODES__RING_FILTER_NODE_HPP_

#include <vector>
#include <memory>

#include "common/types.hpp"
#include "outlier_filter_nodes/visibility_control.hpp"

#include "filter_node_base/filter_node_base.hpp"
#include "outlier_filter/ring_filter.hpp"

#include "rclcpp/rclcpp.hpp"


namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter_nodes
{

/// \class RingFilterNode
/// \brief Node inheriting from FilterNodeBase and is the ROS2 node interface for the
//   RingFilter library
class OUTLIER_FILTER_NODES_PUBLIC RingFilterNode final : public filter_node_base::
  FilterNodeBase
{
public:
  /** \brief The default constructor for the RingFilterNode class
   * \param options An rclcpp::NodeOptions object to pass on to the FilterNodeBase class
   */
  explicit RingFilterNode(const rclcpp::NodeOptions & options);

protected:
  /** \brief Implementation of the FilterNodeBase class abstract filter method
   *
   * Converts the point cloud into the PCL::PointCloud<pcl::PointXYZ> type to be processed by the
   * RingFilter library. The method then returns the filter point cloud via the output argument.
   *
   * \param input The input point cloud dataset.
   * \param output The resultant filtered PointCloud2
   */
  OUTLIER_FILTER_NODES_PUBLIC void filter(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::msg::PointCloud2 & output) override;

  /** \brief Implementation of the FilterNodeBase class abstract get_node_parameters method
   *
   * This method is called when a parameter is set. From the resulting parameters it will get all of
   * the associated parameters and update the filter library class with the new values. The result
   * of retrieving the parameter is returned.
   *
   * \param p Vector of rclcpp::Parameters belonging to the node
   * \return rcl_interfaces::msg::SetParametersResult Result of retrieving the parameter
   */
  OUTLIER_FILTER_NODES_PUBLIC rcl_interfaces::msg::SetParametersResult get_node_parameters(
    const std::vector<rclcpp::Parameter> & p) override;

private:
  /** \brief Class object containing the RingFilter library functionality */
  std::shared_ptr<perception::filters::outlier_filter::ring_filter::
    RingFilter> ring_filter_;

  // variables go here
  common::types::float64_t distance_ratio_;

  common::types::float64_t object_length_threshold_;

  std::int64_t num_points_threshold_;
};
}  // namespace outlier_filter_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // OUTLIER_FILTER_NODES__RING_FILTER_NODE_HPP_
