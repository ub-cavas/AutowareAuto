// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "freespace_planner/freespace_planner.hpp"

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace planning
{
namespace freespace_planner
{

namespace
{

using motion::motion_common::to_quat;
using motion::motion_common::from_quat;
using autoware_auto_msgs::action::PlannerCostmap;


geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  geometry_msgs::msg::PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

autoware_auto_msgs::msg::Trajectory createTrajectory(
  const geometry_msgs::msg::PoseStamped & current_pose, const AstarWaypoints & astar_waypoints,
  const double & velocity)
{
  autoware_auto_msgs::msg::Trajectory trajectory;
  trajectory.header = astar_waypoints.header;

  // TODO trajectory can be only 100 points long - find solution
  // TODO make sure which fields must be filled regarding behavioral planner
  for (const auto & awp : astar_waypoints.waypoints) {
    autoware_auto_msgs::msg::TrajectoryPoint point;

    point.x = awp.pose.pose.position.x;
    point.y = awp.pose.pose.position.y;
    point.z = current_pose.pose.position.z;  // height = const
    point.heading = from_quat<geometry_msgs::msg::Quaternion>(awp.pose.pose.orientation);

    // switch sign by forward/backward
    point.longitudinal_velocity_mps = (awp.is_back ? -1 : 1) * (velocity / 3.6);  // velocity = const

    trajectory.points.push_back(point);
  }

  return trajectory;
}

}  // namespace

FreespacePlannerNode::FreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("freespace_planner", node_options)
{
  using std::placeholders::_1;
  using namespace std::literals::chrono_literals;

  // NodeParam
  {
    node_param_.waypoints_velocity = declare_parameter("waypoints_velocity", 5.0);
    node_param_.update_rate = declare_parameter("update_rate", 1.0);
    node_param_.th_arrived_distance_m = declare_parameter("th_arrived_distance_m", 1.0);
    node_param_.th_stopped_time_sec = declare_parameter("th_stopped_time_sec", 1.0);
    node_param_.th_stopped_velocity_mps = declare_parameter("th_stopped_velocity_mps", 0.01);
    node_param_.th_course_out_distance_m = declare_parameter("th_course_out_distance_m", 3.0);
    node_param_.replan_when_obstacle_found = declare_parameter("replan_when_obstacle_found", true);
    node_param_.replan_when_course_out = declare_parameter("replan_when_course_out", true);
  }

  // AstarParam
  {
    // base configs
    astar_param_.use_back = declare_parameter("use_back", true);
    astar_param_.only_behind_solutions = declare_parameter("only_behind_solutions", false);
    astar_param_.time_limit = declare_parameter("time_limit", 5000.0);

    // robot configs
    // TODO(Kenji Miyake): obtain from vehicle_info
    astar_param_.robot_shape.length = declare_parameter("robot_length", 4.5);
    astar_param_.robot_shape.width = declare_parameter("robot_width", 1.75);
    astar_param_.robot_shape.base2back = declare_parameter("robot_base2back", 1.0);
    astar_param_.minimum_turning_radius = declare_parameter("minimum_turning_radius", 0.5);
    astar_param_.maximum_turning_radius = declare_parameter("maximum_turning_radius", 6.0);
    astar_param_.turning_radius_size = declare_parameter("turning_radius_size", 11);
    astar_param_.maximum_turning_radius = std::max(
      astar_param_.maximum_turning_radius,
      astar_param_.minimum_turning_radius);
    astar_param_.turning_radius_size = std::max(astar_param_.turning_radius_size, 1);

    // search configs
    astar_param_.theta_size = declare_parameter("theta_size", 48);
    astar_param_.angle_goal_range = declare_parameter("angle_goal_range", 6.0);
    astar_param_.curve_weight = declare_parameter("curve_weight", 1.2);
    astar_param_.reverse_weight = declare_parameter("reverse_weight", 2.00);
    astar_param_.lateral_goal_range = declare_parameter("lateral_goal_range", 0.5);
    astar_param_.longitudinal_goal_range = declare_parameter("longitudinal_goal_range", 2.0);

    // costmap configs
    astar_param_.obstacle_threshold = declare_parameter("obstacle_threshold", 100);
    astar_param_.distance_heuristic_weight = declare_parameter("distance_heuristic_weight", 1.0);
  }

  // Publisher
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_debug_pub_ =
      create_publisher<autoware_auto_msgs::msg::Trajectory>("~/output/trajectory", qos);
  }

  // Action client
  {
    map_client_ = rclcpp_action::create_client<PlannerCostmapAction>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "generate_costmap");

    while (!map_client_->wait_for_action_server(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for map server.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for costmap generator action service...");
    }
  }

  // Action service
  {
    plan_trajectory_srv_ =rclcpp_action::create_server<PlanTrajectoryAction>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "/planning/plan_parking_trajectory",  // TODO: add namespace in launch instead of here
      [this](auto uuid, auto goal) {return this->handleGoal(uuid, goal);},
      [this](auto goal_handle) {return this->handleCancel(goal_handle);},
      [this](auto goal_handle) {return this->handleAccepted(goal_handle);});
  }

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);
  }
}

rclcpp_action::GoalResponse FreespacePlannerNode::handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const PlanTrajectoryAction::Goal> goal)
{
  (void) uuid;
  (void) goal;
  if (isPlanning())
  {
    RCLCPP_WARN(get_logger(), "Planner is already planning. Rejecting new goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Received new goal");
  startPlanning();
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FreespacePlannerNode::handleCancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Cancelling trajectory planning");
  stopPlanning();
  auto result = std::make_shared<PlanTrajectoryAction::Result>();
  result->result = PlanTrajectoryAction::Result::FAIL;
  goal_handle->canceled(result);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void FreespacePlannerNode::handleAccepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  RCLCPP_DEBUG(this->get_logger(), "Planning");

  planning_goal_handle_ = goal_handle;

  // acquire start and goal position from action request
  start_pose_.header = goal_handle->get_goal()->sub_route.header;
  start_pose_.pose.position = goal_handle->get_goal()->sub_route.start_point.position;
  start_pose_.pose.orientation =
    to_quat<geometry_msgs::msg::Quaternion>(
    goal_handle->get_goal()->sub_route.start_point.heading);

  goal_pose_.header = goal_handle->get_goal()->sub_route.header;
  goal_pose_.pose.position = goal_handle->get_goal()->sub_route.goal_point.position;
  goal_pose_.pose.orientation =
    to_quat<geometry_msgs::msg::Quaternion>(
    goal_handle->get_goal()->sub_route.goal_point.heading);

  // request costmap and plan trajectory
  auto action_goal = PlannerCostmapAction::Goal();
  action_goal.route = goal_handle->get_goal()->sub_route;

  auto send_goal_options = rclcpp_action::Client<PlannerCostmapAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
    &FreespacePlannerNode::goalResponseCallback,
    this,
    _1);
  send_goal_options.feedback_callback = std::bind(
    &FreespacePlannerNode::feedbackCallback,
    this,
    _1,
    _2);
  send_goal_options.result_callback = std::bind(
    &FreespacePlannerNode::resultCallback,
    this,
    _1);

  map_client_->async_send_goal(action_goal, send_goal_options);
  RCLCPP_INFO(get_logger(), "Costmap generator action goal sent.");
}

void FreespacePlannerNode::goalResponseCallback(
  std::shared_future<PlannerCostmapGoalHandle::SharedPtr> future)
{
  if (!future.get()) {
    RCLCPP_ERROR(get_logger(), "Costmap generator goal was rejected by server");
    auto result = std::make_shared<PlanTrajectoryAction::Result>();
    result->result = PlanTrajectoryAction::Result::FAIL;
    planning_goal_handle_->abort(result);
    stopPlanning();
    return;
  }
  RCLCPP_INFO(get_logger(), "Costmap generator action goal accepted.");
}

void FreespacePlannerNode::feedbackCallback(
  PlannerCostmapGoalHandle::SharedPtr goal_handle,
  const std::shared_ptr<const PlannerCostmapAction::Feedback> feedback)
{
  // feedback is not needed atm
  (void) goal_handle;
  (void) feedback;
}

void FreespacePlannerNode::resultCallback(const PlannerCostmapGoalHandle::WrappedResult & costmap_result)
{
  auto planning_result = std::make_shared<PlanTrajectoryAction::Result>();

  if (costmap_result.result->costmap.data.empty())
  {
    RCLCPP_ERROR(get_logger(), "Costmap generator failed to produce map");
    planning_result->result = PlanTrajectoryAction::Result::FAIL;
    planning_goal_handle_->abort(planning_result);
    return;
  }

  RCLCPP_INFO(get_logger(), "Costmap received. Planning started");

  occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap_result.result->costmap);

  planTrajectory();

  if (trajectory_.points.size() != 0) {
    RCLCPP_INFO(this->get_logger(), "Planning successfull");
    trajectory_debug_pub_->publish(trajectory_);
    planning_result->result = PlanTrajectoryAction::Result::SUCCESS;
    planning_result->trajectory = trajectory_;
    planning_goal_handle_->succeed(planning_result);
  } else {
    RCLCPP_INFO(this->get_logger(), "Planning unsuccessfull");
    planning_result->result = PlanTrajectoryAction::Result::FAIL;
    planning_goal_handle_->abort(planning_result);
  }

  stopPlanning();
}

void FreespacePlannerNode::planTrajectory()
{
  // reset inner state
  reset();

  // Extend robot shape
  RobotShape extended_robot_shape = astar_param_.robot_shape;
  constexpr double margin = 1.0;
  extended_robot_shape.length += margin;
  extended_robot_shape.width += margin;
  extended_robot_shape.base2back += margin / 2;

  // initialize vector for A* search, this runs only once
  astar_ = std::make_unique<AstarSearch>(astar_param_);
  astar_->setRobotShape(extended_robot_shape);
  astar_->initializeNodes(*occupancy_grid_.get());

  // Calculate poses in costmap frame
  // TODO check if behavioral planner pose is already in map frame (in avp or code analysis)
  const auto start_pose_in_costmap_frame = transformPose(
    start_pose_.pose,
    getTransform(occupancy_grid_->header.frame_id, start_pose_.header.frame_id));

  const auto goal_pose_in_costmap_frame = transformPose(
    goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

  // execute astar search
  const rclcpp::Time start = get_clock()->now();
  const bool result = astar_->makePlan(start_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  const rclcpp::Time end = get_clock()->now();

  RCLCPP_INFO(get_logger(), "Astar planning: %f [s]", (end - start).seconds());

  if (result) {
    RCLCPP_INFO(get_logger(), "Found goal!");
    trajectory_ =
      createTrajectory(start_pose_, astar_->getWaypoints(), node_param_.waypoints_velocity);
  } else {
    RCLCPP_INFO(get_logger(), "Can't find goal...");
    reset();
  }
}

bool FreespacePlannerNode::isPlanning() const
{
  return state_ == FreespacePlannerState::PLANNING;
}
void FreespacePlannerNode::startPlanning()
{
  state_ = FreespacePlannerState::PLANNING;
}
void FreespacePlannerNode::stopPlanning()
{
  state_ = FreespacePlannerState::IDLE;
}

void FreespacePlannerNode::reset()
{
  trajectory_ = autoware_auto_msgs::msg::Trajectory();
  // this->set_parameter(rclcpp::Parameter("is_completed", false));
}

geometry_msgs::msg::TransformStamped FreespacePlannerNode::getTransform(
  const std::string & from, const std::string & to)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

}  // namespace freespace_planner
}  // namespace planning
}  // namespace autoware
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning::freespace_planner::FreespacePlannerNode)
