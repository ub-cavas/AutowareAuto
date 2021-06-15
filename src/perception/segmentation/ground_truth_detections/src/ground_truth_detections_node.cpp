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

#include "ground_truth_detections/ground_truth_detections_node.hpp"

#include <autoware_auto_msgs/msg/classified_roi.hpp>
namespace
{

// Make classification with class known with certainty. LGSVL only knows `car` and `pedestrian` up to now
autoware_auto_msgs::msg::ObjectClassification make_classification(
  const lgsvl_msgs::msg::Detection2D & detection)
{
  autoware_auto_msgs::msg::ObjectClassification obj_classification;
  obj_classification.probability = 1.0;
  if (detection.label == "car") {
    obj_classification.classification = autoware_auto_msgs::msg::ObjectClassification::CAR;
  } else if (detection.label == "pedestrian") {
    obj_classification.classification = autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN;
  } else {
    obj_classification.classification = autoware_auto_msgs::msg::ObjectClassification::UNKNOWN;
  }
  return obj_classification;
}

// Convert 2D bounding box from LGSVL format of center point, width, and height to a polygon with with four points
geometry_msgs::msg::Polygon make_polygon(const lgsvl_msgs::msg::Detection2D & detection)
{
  geometry_msgs::msg::Polygon polygon;
  auto & points = polygon.points;
  points.resize(4);
  const float width = detection.bbox.width;
  const float height = detection.bbox.height;

  // bbox coordinates given at center
  // lower left corner
  points[0].x = detection.bbox.x - 0.5F * width;
  points[0].y = detection.bbox.y - 0.5F * height;

  // lower right corner
  points[1] = points[0];
  points[1].x += width;

  // upper right corner
  points[2] = points[1];
  points[2].y += height;

  // upper left corner
  points[3] = points[2];
  points[3].x -= width;

  return polygon;
}

}

namespace autoware
{
namespace ground_truth_detections
{

GroundTruthDetectionsNode::GroundTruthDetectionsNode(const rclcpp::NodeOptions & options)
:  Node("ground_truth_detections", options)
{
}

void GroundTruthDetectionsNode::on_detection(const lgsvl_msgs::msg::Detection2DArray & msg)
{
  autoware_auto_msgs::msg::ClassifiedRoiArray roi_array;
  roi_array.header = msg.header;

  roi_array.rois.reserve(msg.detections.size());
  std::transform(
    msg.detections.begin(), msg.detections.end(), roi_array.rois.begin(),
    [](const lgsvl_msgs::msg::Detection2D & detection) {
      return autoware_auto_msgs::build<autoware_auto_msgs::msg::ClassifiedRoi>().classifications(
        {make_classification(detection)}).polygon(make_polygon(detection));
    });
  m_detection2d_pub->publish(roi_array);
}
}  // namespace ground_truth_detections
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ground_truth_detections::GroundTruthDetectionsNode)
