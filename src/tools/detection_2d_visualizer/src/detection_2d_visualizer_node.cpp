// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <common/types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <detection_2d_visualizer/detection_2d_visualizer_node.hpp>
#include <detection_2d_visualizer/utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <time_utils/time_utils.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <utility>
#include <vector>

namespace autoware
{
namespace detection_2d_visualizer
{
Detection2dVisualizerNode::Detection2dVisualizerNode(const rclcpp::NodeOptions & options)
:  Node("ground_truth_visualizer", options),
  m_image_sub{this, "/simulator/main_camera"},
  m_roi_sub(this, "/rois"),
  m_cloud_sub(this, "/clouds"),
  m_image_pub{create_publisher<sensor_msgs::msg::Image>("/image_with_detections", rclcpp::QoS{20})},
  m_tf_listener{m_tf_buffer}
{
  auto initialize = [this](auto & sync_ptr) {
      sync_ptr->registerCallback(
        std::bind(
          &Detection2dVisualizerNode::process, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    };
  if (declare_parameter("sync_approximately", true)) {
    m_approximate_sync_ptr = std::make_unique<message_filters::Synchronizer<ApproximatePolicy>>(
      ApproximatePolicy(50), m_image_sub, m_roi_sub, m_cloud_sub);
    initialize(m_approximate_sync_ptr);
  } else {
    m_exact_sync_ptr = std::make_unique<message_filters::Synchronizer<ExactPolicy>>(
      ExactPolicy(50), m_image_sub, m_roi_sub, m_cloud_sub);
    initialize(m_exact_sync_ptr);
  }
}

void Detection2dVisualizerNode::process(
  sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg,
  autoware_auto_msgs::msg::ClassifiedRoiArray::ConstSharedPtr roi_msg,
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  static const cv::Scalar ground_truth_color{0, 255, 0};
  static const cv::Scalar projection_color{0, 0, 255};
  constexpr std::int32_t thickness = 5;
  cv_bridge::CvImagePtr cv_img_ptr;
  try {
    cv_img_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  for (const auto & rect : roi_msg->rois) {
    draw_shape(cv_img_ptr, rect.polygon, ground_truth_color, thickness);
  }

  struct Point
  {
    float x;
    float y;
    float z;
    alignas(double) std::uint8_t intensity;
    double timestamp;
  };

  struct CameraIntrinsics
  {
    float image_width{1920.0F};
    float image_height{1080.0F};
    float fx{1158.0337F};
    float fy{1158.0337F};

    Eigen::Matrix3f as_matrix()
    {
      return (Eigen::Matrix3f{} <<
             fx, 0.0F, image_width / 2.0F,
             0.0F, fy, image_height / 2.0F,
             0.0F, 0.0F, 1.0F).finished();
    }
  };


  geometry_msgs::msg::TransformStamped tf2_camera_from_cloud;
  try {
    tf2_camera_from_cloud = m_tf_buffer.lookupTransform(
      roi_msg->header.frame_id, cloud_msg->header.frame_id,
      time_utils::from_message(roi_msg->header.stamp));
  } catch (...) {
    std::cerr << "Cannot find appropriate transform" << std::endl;
    return;
  }

  CameraIntrinsics intrinsics{};
  const auto intrinsics_mat = intrinsics.as_matrix();

  const Eigen::Isometry3d tfd_camera_from_lidar = tf2::transformToEigen(tf2_camera_from_cloud);
  const Eigen::Isometry3f tf_camera_from_cloud = tfd_camera_from_lidar.cast<float>();

  std::cerr << cv_img_ptr->image.type() << std::endl;
  std::cerr << cloud_msg->width << std::endl;

  const point_cloud_msg_wrapper::PointCloud2View<Point> view{*cloud_msg};
  for (const auto & point : view) {
    const Eigen::Vector3f point_lidar{point.x, point.y, point.z};
    const Eigen::Vector3f point_camera = tf_camera_from_cloud * point_lidar;
    if (point_camera.z() < 0.1F) {
      continue;
    }
    const auto point_image = (intrinsics_mat * point_camera) / point_camera.z();
    if (point_image.x() < 0.0F || point_image.x() > intrinsics.image_width ||
      point_image.y() < 0.0F || point_image.y() > intrinsics.image_height)
    {
      continue;
    }
    const int col = static_cast<int>(std::round(point_image.x()));
    const int row = static_cast<int>(std::round(point_image.y()));
    auto & pixel = cv_img_ptr->image.at<cv::Vec3b>(row, col);
    pixel[0] = point.intensity;
    pixel[1] = 255;
    pixel[2] = 255;
  }

  m_image_pub->publish(*(cv_img_ptr->toImageMsg()));
}

}  // namespace detection_2d_visualizer
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::detection_2d_visualizer::Detection2dVisualizerNode)
