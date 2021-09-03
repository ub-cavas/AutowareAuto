#include <autoware_auto_msgs/msg/tracked_objects.hpp>
#include <common/types.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tracking/projection.hpp>
#include <tracking/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <time_utils/time_utils.hpp>
#include <tracking/projection_debugger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_eigen/tf2_eigen.h>

namespace autoware
{
namespace perception
{
namespace tracking
{
using common::types::float64_t;

ProjectionDebuggerNode::ProjectionDebuggerNode(const rclcpp::NodeOptions & options)
: Node("proj_debugger", options),
  m_tf_pub{create_publisher<tf2_msgs::msg::TFMessage>(
      "tf_static",
      rclcpp::QoS{10U}.transient_local())},
  m_track_pub{create_publisher<TrackedObjects>("tracks", 10U)}
{
  CameraIntrinsics camera_intrinsics;
  camera_intrinsics.width = 1920;
  camera_intrinsics.height = 1080;
  camera_intrinsics.fx = 1158.0337F;
  camera_intrinsics.fy = 1158.0337F;
  camera_intrinsics.ox = 0.F;
  camera_intrinsics.oy = 0.F;

  Eigen::Affine3d tf_ego_from_camera;
  tf_ego_from_camera.setIdentity();
  tf_ego_from_camera.translate(Eigen::Vector3d{-0.2, 0.0, 1.7});

  constexpr auto pi = static_cast<float64_t>(autoware::common::types::PI);
  tf_ego_from_camera.rotate(
    Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitX()));
  tf_ego_from_camera.rotate(
    Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitY()));

  m_camera_from_base_link = tf2::eigenToTransform(tf_ego_from_camera.inverse()).transform;
  m_camera_ptr = std::make_unique<CameraModel>(camera_intrinsics);

  make_objects();
  publish_tf();
  m_wall_timer = create_wall_timer(
    std::chrono::milliseconds{100},
    std::bind(&ProjectionDebuggerNode::timer_function, this));
}

void ProjectionDebuggerNode::timer_function()
{
  m_track_pub->publish(m_objects);
}

void ProjectionDebuggerNode::publish_tf()
{
  tf2_msgs::msg::TFMessage msg;
  TransformStamped tf;
  const auto eig_tf = tf2::transformToEigen(m_camera_from_base_link);
  tf = tf2::eigenToTransform(eig_tf.inverse());
  tf.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "camera";

  msg.transforms.push_back(tf);
  m_tf_pub->publish(msg);
}

ProjectionDebuggerNode::Point32
ProjectionDebuggerNode::make_pt(float32_t x, float32_t y, float32_t z)
{
  return Point32{}.set__x(x).set__y(y).set__z(z);
}

std::vector<geometry_msgs::msg::Point32> expand_shape_to_vector(
  const autoware_auto_msgs::msg::Shape & shape)
{
  std::vector<geometry_msgs::msg::Point32> result{shape.polygon.points};
  const auto num_corners = shape.polygon.points.size();
  for (auto i = 0U; i < num_corners; ++i) {
    auto pt = shape.polygon.points[i];
    result.push_back(pt.set__z(pt.z + shape.height));
  }
  return result;
}


ProjectionDebuggerNode::TrackedObject
ProjectionDebuggerNode::from_centroid(
  float32_t x, float32_t y, float32_t z,
  std::size_t ID)
{
  TrackedObject object;
  object.shape.emplace_back();
  constexpr auto half_width = 0.2F;
  constexpr auto half_height = 0.2F;

  object.shape.front().polygon.points.push_back(make_pt(x - half_width, y + half_height, z));
  object.shape.front().polygon.points.push_back(make_pt(x + half_width, y + half_height, z));
  object.shape.front().polygon.points.push_back(make_pt(x + half_width, y - half_height, z));
  object.shape.front().polygon.points.push_back(make_pt(x - half_width, y - half_height, z));
  object.shape.front().height = 0.4F;
  details::ShapeTransformer transformer{m_camera_from_base_link};
  if (m_camera_ptr->project(transformer(object.shape.front()))) {
    TrackedObject::_classification_type::value_type cls;
    cls.classification = TrackedObject::_classification_type::value_type::CAR;
    cls.probability = 1.0F;
    object.classification.push_back(cls);
  } else {
    TrackedObject::_classification_type::value_type cls;
    cls.classification = TrackedObject::_classification_type::value_type::UNKNOWN;
    cls.probability = 1.0F;
    object.classification.push_back(cls);
  }

  object.kinematics.centroid_position.set__x(500.).set__y(500.).set__z(500.);
  object.object_id = ID;

  return object;
}

void ProjectionDebuggerNode::make_objects()
{
  std::size_t counter = 0U;
  constexpr auto radius = 5.0F;
  const auto eps = std::numeric_limits<float32_t>::epsilon();
  for (auto angle = 0.0F;
    angle < 2.0f * common::types::PI + eps;
    angle += common::types::PI / 20.0F)
  {
    m_objects.objects.push_back(
      from_centroid(radius * std::cos(angle), 0.0F, radius * std::sin(angle), ++counter));
    m_objects.objects.push_back(
      from_centroid(0.0F, radius * std::cos(angle), radius * std::sin(angle), ++counter));
    m_objects.objects.push_back(
      from_centroid(radius * std::cos(angle), radius * std::sin(angle), 0.0F, ++counter));
  }

  m_objects.header.frame_id = "base_link";
  m_objects.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
}
}
}
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::perception::tracking::ProjectionDebuggerNode)
