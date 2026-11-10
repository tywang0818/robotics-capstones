#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/body_operations.h>
#include <jaka_planning_interfaces/msg/environment_model.hpp>
#include <jaka_planning_interfaces/srv/build_conservative_octomap.hpp>
#include <jaka_planning_interfaces/srv/validate_known_free_trajectory.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
using EnvironmentModel = jaka_planning_interfaces::msg::EnvironmentModel;
using ValidateTrajectory =
  jaka_planning_interfaces::srv::ValidateKnownFreeTrajectory;
using BuildConservativeOctomap =
  jaka_planning_interfaces::srv::BuildConservativeOctomap;

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);
  if (q.norm() < 1e-9) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = q.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}
}  // namespace

class KnownFreeValidator : public rclcpp::Node
{
public:
  static std::shared_ptr<KnownFreeValidator> create()
  {
    auto node = std::shared_ptr<KnownFreeValidator>(new KnownFreeValidator(PrivateTag{}));
    node->initialize();
    return node;
  }

private:
  struct PrivateTag {};

  explicit KnownFreeValidator(PrivateTag)
  : Node("known_free_validator")
  {
    declare_parameter<std::string>("environment_topic", "/known_environment");
    declare_parameter<std::string>("validation_service", "/validate_known_free_trajectory");
    declare_parameter<std::string>("conservative_map_service", "/build_conservative_octomap");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<bool>("initial_scan_free_cylinder_enabled", true);
    declare_parameter<double>("initial_scan_free_cylinder_center_x", 0.0);
    declare_parameter<double>("initial_scan_free_cylinder_center_y", 0.0);
    declare_parameter<double>("initial_scan_free_cylinder_radius", 0.25);
    declare_parameter<double>("initial_scan_free_cylinder_z_min", 0.0);
    declare_parameter<double>("initial_scan_free_cylinder_z_max", 1.26);
  }

  void initialize()
  {
    robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_) {
      throw std::runtime_error("Failed to load robot model from robot_description.");
    }

    rclcpp::QoS qos(1);
    qos.reliable().transient_local();
    environment_sub_ = create_subscription<EnvironmentModel>(
      get_parameter("environment_topic").as_string(), qos,
      std::bind(&KnownFreeValidator::environmentCallback, this, std::placeholders::_1));
    service_ = create_service<ValidateTrajectory>(
      get_parameter("validation_service").as_string(),
      std::bind(
        &KnownFreeValidator::validate, this, std::placeholders::_1,
        std::placeholders::_2));
    conservative_service_ = create_service<BuildConservativeOctomap>(
      get_parameter("conservative_map_service").as_string(),
      std::bind(
        &KnownFreeValidator::buildConservativeMap, this, std::placeholders::_1,
        std::placeholders::_2));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_states_topic").as_string(), 20,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        latest_joint_state_ = *message;
      });

    RCLCPP_INFO(
      get_logger(), "Ready: strict known-free validation service=%s",
      get_parameter("validation_service").as_string().c_str());
  }

  static double pointSegmentDistance(
    const Eigen::Vector3d & point, const Eigen::Vector3d & start,
    const Eigen::Vector3d & end)
  {
    const Eigen::Vector3d segment = end - start;
    const double length_squared = segment.squaredNorm();
    if (length_squared < 1e-12) return (point - start).norm();
    const double t = std::clamp((point - start).dot(segment) / length_squared, 0.0, 1.0);
    return (point - (start + t * segment)).norm();
  }

  bool currentRobotBodies(
    const std::string & planning_group, double padding,
    std::vector<bodies::BodyPtr> & robot_bodies,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> & robot_segments)
  {
    sensor_msgs::msg::JointState joints;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      joints = latest_joint_state_;
    }
    if (joints.name.empty() || joints.name.size() != joints.position.size()) return false;
    const auto * group = robot_model_->getJointModelGroup(planning_group);
    if (!group) return false;
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.setVariablePositions(joints.name, joints.position);
    state.updateLinkTransforms();
    std::vector<Eigen::Vector3d> link_origins;
    link_origins.push_back(
      state.getGlobalLinkTransform(robot_model_->getRootLinkName()).translation());
    for (const auto & link_name : group->getLinkModelNames()) {
      link_origins.push_back(state.getGlobalLinkTransform(link_name).translation());
    }
    for (std::size_t i = 1; i < link_origins.size(); ++i) {
      robot_segments.emplace_back(link_origins[i - 1], link_origins[i]);
    }
    for (const auto * link : robot_model_->getLinkModelsWithCollisionGeometry()) {
      const auto & shapes = link->getShapes();
      const auto & collision_origins = link->getCollisionOriginTransforms();
      if (shapes.size() != collision_origins.size()) return false;
      const Eigen::Isometry3d & link_pose = state.getGlobalLinkTransform(link);
      for (std::size_t i = 0; i < shapes.size(); ++i) {
        bodies::BodyPtr body(bodies::createBodyFromShape(shapes[i].get()));
        if (!body) continue;
        body->setPoseDirty(link_pose * collision_origins[i]);
        body->setPaddingDirty(padding);
        body->updateInternalData();
        robot_bodies.push_back(std::move(body));
      }
    }
    return !robot_bodies.empty() && !robot_segments.empty();
  }

  void buildConservativeMap(
    const BuildConservativeOctomap::Request::SharedPtr request,
    BuildConservativeOctomap::Response::SharedPtr response)
  {
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(
      request->observed.octomap.binary ?
      octomap_msgs::binaryMsgToMap(request->observed.octomap) :
      octomap_msgs::fullMsgToMap(request->observed.octomap));
    auto * observed = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (!observed) {
      response->message = "Observed environment is not a valid OcTree.";
      return;
    }
    if (!(request->min_x < request->max_x && request->min_y < request->max_y &&
      request->min_z < request->max_z))
    {
      response->message = "Conservative planning bounds are invalid.";
      return;
    }
    const double resolution = std::max(0.02, request->resolution);
    const double clearance = std::max(0.02, request->robot_clearance);
    const double seed_radius = std::max(0.0, request->initial_seed_radius);
    const bool initial_cylinder_enabled =
      get_parameter("initial_scan_free_cylinder_enabled").as_bool();
    const double initial_cylinder_center_x =
      get_parameter("initial_scan_free_cylinder_center_x").as_double();
    const double initial_cylinder_center_y =
      get_parameter("initial_scan_free_cylinder_center_y").as_double();
    const double initial_cylinder_radius = std::max(
      0.0, get_parameter("initial_scan_free_cylinder_radius").as_double());
    const double initial_cylinder_z_min =
      get_parameter("initial_scan_free_cylinder_z_min").as_double();
    const double initial_cylinder_z_max =
      get_parameter("initial_scan_free_cylinder_z_max").as_double();
    const double initial_cylinder_radius_sq =
      initial_cylinder_radius * initial_cylinder_radius;
    // Include half a voxel diagonal so a voxel overlapping the robot is
    // cleared even when its center lies just outside the collision geometry.
    const double robot_padding = clearance + std::sqrt(3.0) * resolution * 0.5;
    std::vector<bodies::BodyPtr> robot_bodies;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> current_segments;
    if (!currentRobotBodies(
        request->planning_group, robot_padding, robot_bodies, current_segments))
    {
      response->message = "Current joint state or robot collision geometry is unavailable.";
      return;
    }
    if (initial_seed_segments_.empty()) {
      initial_seed_segments_ = current_segments;
      RCLCPP_INFO(
        get_logger(), "Captured initial robot seed envelope with %zu segments.",
        initial_seed_segments_.size());
    }
    const Eigen::Isometry3d world_to_observed = poseToEigen(request->observed.origin).inverse();
    octomap::OcTree conservative(resolution);
    uint64_t initial_cylinder_carved_cells = 0;
    for (double x = request->min_x + resolution * 0.5; x < request->max_x; x += resolution) {
      for (double y = request->min_y + resolution * 0.5; y < request->max_y; y += resolution) {
        for (double z = request->min_z + resolution * 0.5; z < request->max_z; z += resolution) {
          const Eigen::Vector3d world(x, y, z);
          const double dx = x - initial_cylinder_center_x;
          const double dy = y - initial_cylinder_center_y;
          const bool in_initial_cylinder =
            initial_cylinder_enabled &&
            initial_cylinder_radius > 0.0 &&
            initial_cylinder_z_min < initial_cylinder_z_max &&
            z >= initial_cylinder_z_min &&
            z <= initial_cylinder_z_max &&
            (dx * dx + dy * dy) <= initial_cylinder_radius_sq;
          bool carved = false;
          for (const auto & body : robot_bodies) {
            if (body->containsPoint(world)) {
              carved = true;
              break;
            }
          }
          bool in_initial_seed = false;
          if (seed_radius > 0.0) {
            for (const auto & segment : initial_seed_segments_) {
              if (pointSegmentDistance(world, segment.first, segment.second) <= seed_radius) {
                in_initial_seed = true;
                break;
              }
            }
          }
          const Eigen::Vector3d map_point = world_to_observed * world;
          const auto * node = observed->search(map_point.x(), map_point.y(), map_point.z());
          bool occupied = true;
          if (in_initial_cylinder) {
            occupied = false;
            ++initial_cylinder_carved_cells;
          } else if (!node && carved) {
            occupied = false;
            ++response->robot_carved_cells;
          } else if (!node && in_initial_seed) {
            occupied = false;
            ++response->initial_seed_carved_cells;
          } else if (!node) {
            ++response->unknown_as_occupied_cells;
          } else if (observed->isNodeOccupied(node)) {
            ++response->observed_occupied_cells;
          } else {
            occupied = false;
            ++response->free_cells;
          }
          conservative.updateNode(octomap::point3d(x, y, z), occupied, true);
        }
      }
    }
    conservative.updateInnerOccupancy();
    conservative.prune();
    response->conservative.header = request->observed.header;
    response->conservative.origin.orientation.w = 1.0;
    if (!octomap_msgs::binaryMapToMsg(conservative, response->conservative.octomap)) {
      response->message = "Failed to serialize conservative OctoMap.";
      return;
    }
    response->conservative.octomap.header = request->observed.header;
    {
      std::lock_guard<std::mutex> lock(environment_mutex_);
      tree_ = std::make_unique<octomap::OcTree>(conservative);
      environment_version_ = request->environment_version;
      world_to_map_ = Eigen::Isometry3d::Identity();
    }
    response->success = true;
    response->message = "Unknown cells converted to occupied inside planning bounds.";
    RCLCPP_INFO(
      get_logger(),
      "Conservative map v%lu: free=%lu unknown_as_occupied=%lu "
      "observed_occupied=%lu robot_carved=%lu initial_seed_carved=%lu "
      "initial_cylinder_carved=%lu resolution=%.3f clearance=%.3f "
      "seed_radius=%.3f cylinder=[enabled=%s center=(%.3f,%.3f) "
      "radius=%.3f z=(%.3f,%.3f)].",
      static_cast<unsigned long>(request->environment_version),
      static_cast<unsigned long>(response->free_cells),
      static_cast<unsigned long>(response->unknown_as_occupied_cells),
      static_cast<unsigned long>(response->observed_occupied_cells),
      static_cast<unsigned long>(response->robot_carved_cells),
      static_cast<unsigned long>(response->initial_seed_carved_cells),
      static_cast<unsigned long>(initial_cylinder_carved_cells),
      resolution, robot_padding, seed_radius,
      initial_cylinder_enabled ? "true" : "false",
      initial_cylinder_center_x, initial_cylinder_center_y,
      initial_cylinder_radius, initial_cylinder_z_min, initial_cylinder_z_max);
  }

  void environmentCallback(const EnvironmentModel::SharedPtr message)
  {
    // Live snapshots may advance while MoveIt is planning task version A.
    // Do not overwrite A's conservative validation map with a newer raw map;
    // only buildConservativeMap() may replace the task-pinned tree.
    latest_observed_environment_version_ = message->version;
  }

  bool samplePoint(
    const Eigen::Vector3d & world_point, double clearance,
    ValidateTrajectory::Response & response) const
  {
    static const std::vector<Eigen::Vector3d> directions = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ()};
    for (const auto & direction : directions) {
      const Eigen::Vector3d map_point =
        world_to_map_ * (world_point + clearance * direction);
      const auto * node = tree_->search(
        map_point.x(), map_point.y(), map_point.z());
      ++response.checked_samples;
      if (!node) {
        ++response.unknown_samples;
        return false;
      }
      if (tree_->isNodeOccupied(node)) {
        ++response.occupied_samples;
        return false;
      }
    }
    return true;
  }

  bool validateState(
    moveit::core::RobotState & state,
    const moveit::core::JointModelGroup * group, double link_step,
    double clearance, ValidateTrajectory::Response & response) const
  {
    state.updateLinkTransforms();
    std::vector<Eigen::Vector3d> origins;
    for (const std::string & link_name : group->getLinkModelNames()) {
      origins.push_back(state.getGlobalLinkTransform(link_name).translation());
    }
    for (std::size_t i = 0; i < origins.size(); ++i) {
      if (!samplePoint(origins[i], clearance, response)) {
        return false;
      }
      if (i == 0) {
        continue;
      }
      const Eigen::Vector3d delta = origins[i] - origins[i - 1];
      const int samples = std::max(
        1, static_cast<int>(std::ceil(delta.norm() / link_step)));
      for (int sample = 1; sample < samples; ++sample) {
        if (!samplePoint(
            origins[i - 1] + delta * (static_cast<double>(sample) / samples),
            clearance, response))
        {
          return false;
        }
      }
    }
    return true;
  }

  void validate(
    const ValidateTrajectory::Request::SharedPtr request,
    ValidateTrajectory::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(environment_mutex_);
    if (!tree_) {
      response->reason = "No OctoMap is available to validate the trajectory.";
      return;
    }
    if (request->environment_version != environment_version_) {
      response->reason = "Validator environment version does not match the task.";
      return;
    }
    if (tree_->size() <= 1) {
      response->reason = "The OctoMap contains no known-free volume.";
      return;
    }

    const auto * group = robot_model_->getJointModelGroup(request->planning_group);
    if (!group) {
      response->reason = "Unknown planning group: " + request->planning_group;
      return;
    }
    const auto & joint_trajectory = request->trajectory.joint_trajectory;
    if (joint_trajectory.points.empty() || joint_trajectory.joint_names.empty()) {
      response->reason = "Planned trajectory is empty.";
      return;
    }

    const double interpolation_step = std::max(0.01, request->interpolation_step);
    const double link_step = std::max(0.005, request->link_sample_step);
    const double clearance = std::max(0.0, request->clearance);
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();

    std::vector<double> previous;
    for (const auto & point : joint_trajectory.points) {
      const std::vector<double> current(point.positions.begin(), point.positions.end());
      int interpolation_count = 1;
      if (!previous.empty()) {
        double max_delta = 0.0;
        for (std::size_t i = 0; i < std::min(previous.size(), current.size()); ++i) {
          max_delta = std::max(max_delta, std::abs(current[i] - previous[i]));
        }
        interpolation_count = std::max(
          1, static_cast<int>(std::ceil(max_delta / interpolation_step)));
      }
      for (int sample = 1; sample <= interpolation_count; ++sample) {
        std::vector<double> values = current;
        if (!previous.empty()) {
          const double t = static_cast<double>(sample) / interpolation_count;
          for (std::size_t i = 0; i < std::min(previous.size(), values.size()); ++i) {
            values[i] = previous[i] + t * (current[i] - previous[i]);
          }
        }
        state.setVariablePositions(joint_trajectory.joint_names, values);
        if (!validateState(state, group, link_step, clearance, *response)) {
          response->reason = response->unknown_samples > 0 ?
            "Trajectory enters unknown OctoMap space." :
            "Trajectory enters occupied OctoMap space.";
          return;
        }
      }
      previous = current;
    }

    response->valid = true;
    response->reason = "All sampled robot-link points are in known-free OctoMap space.";
  }

  std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  rclcpp::Subscription<EnvironmentModel>::SharedPtr environment_sub_;
  rclcpp::Service<ValidateTrajectory>::SharedPtr service_;
  rclcpp::Service<BuildConservativeOctomap>::SharedPtr conservative_service_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> initial_seed_segments_;
  mutable std::mutex environment_mutex_;
  std::unique_ptr<octomap::OcTree> tree_;
  uint64_t environment_version_{0};
  uint64_t latest_observed_environment_version_{0};
  Eigen::Isometry3d world_to_map_{Eigen::Isometry3d::Identity()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(KnownFreeValidator::create());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("known_free_validator"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
