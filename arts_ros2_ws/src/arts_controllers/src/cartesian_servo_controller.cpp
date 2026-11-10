// Copyright 2026 ARTS contributors
// SPDX-License-Identifier: Apache-2.0

#include "arts_controllers/cartesian_servo_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace
{
/// Reference interface names, in export order. Position first, then an xyzw quaternion.
constexpr std::array<const char *, 7> kReferenceInterfaceNames = {
  "target_pose/position.x",    "target_pose/position.y",    "target_pose/position.z",
  "target_pose/orientation.x", "target_pose/orientation.y", "target_pose/orientation.z",
  "target_pose/orientation.w"};

/// Name given to the frame that carries the live tool-tip offset. Deliberately prefixed
/// so it cannot collide with a frame that came from the URDF.
constexpr const char * kTipFrameName = "arts_tool_tip";
// The instrument wrist's frames (decision 17, 2026-11-03 amendment).
constexpr const char * kShaftFrameName = "arts_shaft_distal";  // A's frame E
constexpr const char * kRollRefFrameName = "arts_shaft_origin";  // fixed to the flange

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

namespace arts_controllers
{

CartesianServoController::CartesianServoController()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn CartesianServoController::on_init()
{
  try {
    param_listener_ =
      std::make_shared<arts_cartesian_servo_controller::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during parameter init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianServoController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(params_.joints.size());
  const char * interface = params_.command_interface == "velocity"
                              ? hardware_interface::HW_IF_VELOCITY
                              : hardware_interface::HW_IF_POSITION;
  for (const auto & joint : params_.joints) {
    config.names.push_back(joint + "/" + interface);
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianServoController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

bool CartesianServoController::buildModel(std::string & error)
{
  const std::string & urdf = get_robot_description();
  if (urdf.empty()) {
    error = "robot_description is empty; the controller manager was started without one";
    return false;
  }

  pinocchio::Model full_model;
  try {
    pinocchio::urdf::buildModelFromXML(urdf, full_model);
  } catch (const std::exception & e) {
    error = std::string("could not parse robot_description: ") + e.what();
    return false;
  }

  // The description may cover the whole patient cart, so reduce it to just this arm.
  // Locking freezes the other joints at the neutral configuration, which is harmless
  // here because nothing outside this arm's chain contributes to its kinematics -- the
  // arm's base is attached through fixed joints, which reduction preserves.
  std::vector<pinocchio::JointIndex> joints_to_lock;
  for (pinocchio::JointIndex jid = 1; jid < full_model.joints.size(); ++jid) {
    const std::string & name = full_model.names[jid];
    if (std::find(params_.joints.begin(), params_.joints.end(), name) == params_.joints.end()) {
      joints_to_lock.push_back(jid);
    }
  }

  const std::size_t found = full_model.joints.size() - 1 - joints_to_lock.size();
  if (found != params_.joints.size()) {
    error = "robot_description does not contain all configured joints (found " +
            std::to_string(found) + " of " + std::to_string(params_.joints.size()) +
            "); check the names and the tf_prefix";
    return false;
  }

  const Eigen::VectorXd neutral = pinocchio::neutral(full_model);
  pinocchio::buildReducedModel(full_model, joints_to_lock, neutral, model_);

  if (model_.nq != static_cast<int>(kNumJoints) || model_.nv != static_cast<int>(kNumJoints)) {
    // Ruckig is instantiated at a fixed DoF count to stay off the heap, so this is a
    // hard constraint rather than something to adapt to. A continuous joint parsed as
    // planar or a mimic joint surviving reduction would land here.
    error = "reduced model has nq=" + std::to_string(model_.nq) + ", nv=" +
            std::to_string(model_.nv) + "; expected " + std::to_string(kNumJoints) +
            " for both";
    return false;
  }

  if (!model_.existFrame(params_.base_frame)) {
    error = "base_frame '" + params_.base_frame + "' is not in the model";
    return false;
  }
  if (!model_.existFrame(params_.flange_frame)) {
    error = "flange_frame '" + params_.flange_frame + "' is not in the model";
    return false;
  }
  // Resolved once. getFrameId is a string-keyed linear scan of the frame list, which has
  // no business running inside a 500 Hz loop.
  base_frame_id_ = model_.getFrameId(params_.base_frame);
  flange_frame_id_ = model_.getFrameId(params_.flange_frame);

  // The tip frame hangs off the same joint as the flange. pinocchio expresses a frame's
  // placement relative to its PARENT JOINT, not its parent frame, so the offset has to be
  // composed onto the flange's own placement -- getting this wrong yields a tip that
  // tracks correctly only when the flange happens to sit at its joint origin.
  const pinocchio::Frame flange = model_.frames[flange_frame_id_];
  if (params_.wrist.enabled) {
    // Collaborator A's instrument chain (decision 17, 2026-11-03 amendment): the shaft line
    // shaft_offset from the flange along tool0 z, running along tool0 +y (a -90 deg turn
    // about x takes z to +y, as the da_vinci_x_wrist tip preset); roll about the shaft,
    // pitch about x pitch_distance down it, yaw about -y pitch_to_yaw further. At zero wrist
    // angles the tip frame equals the tip preset. Yaw is about -y: with +y the model turned the
    // jaws opposite to the instrument's own positive yaw (2026-11-05, FLIR frames at single-joint
    // poses: pitch and roll matched, yaw mirrored); the drive's joint sign is A's, kept.
    const pinocchio::SE3 shaft_origin(
      Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix(),
      Eigen::Vector3d(0.0, 0.0, params_.wrist.shaft_offset));
    const pinocchio::SE3 roll_placement = flange.placement * shaft_origin;
    auto limit = [](double v) {return Eigen::VectorXd::Constant(1, v);};
    const auto & lo = params_.wrist.lower_limits;
    const auto & hi = params_.wrist.upper_limits;
    const auto & vmax = params_.wrist.max_velocity;
    const pinocchio::JointIndex roll = model_.addJoint(
      flange.parentJoint, pinocchio::JointModelRZ(), roll_placement, params_.wrist.joints[0],
      limit(1.0), limit(vmax[0]), limit(lo[0]), limit(hi[0]));
    const pinocchio::JointIndex pitch = model_.addJoint(
      roll, pinocchio::JointModelRX(),
      pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, params_.wrist.pitch_distance)),
      params_.wrist.joints[1], limit(1.0), limit(vmax[1]), limit(lo[1]), limit(hi[1]));
    const pinocchio::JointIndex yaw = model_.addJoint(
      pitch, pinocchio::JointModelRevoluteUnaligned(0.0, -1.0, 0.0),
      pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, params_.wrist.pitch_to_yaw)),
      params_.wrist.joints[2], limit(1.0), limit(vmax[2]), limit(lo[2]), limit(hi[2]));
    roll_ref_frame_id_ = model_.addFrame(pinocchio::Frame(
      kRollRefFrameName, flange.parentJoint, flange_frame_id_, roll_placement,
      pinocchio::OP_FRAME));
    shaft_frame_id_ = model_.addFrame(pinocchio::Frame(
      kShaftFrameName, roll, roll_ref_frame_id_,
      pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, params_.wrist.pitch_distance)),
      pinocchio::OP_FRAME));
    tip_frame_id_ = model_.addFrame(pinocchio::Frame(
      kTipFrameName, yaw, shaft_frame_id_, pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
  } else {
    const pinocchio::SE3 initial_offset = pinocchio::SE3::Identity();
    pinocchio::Frame tip(
      kTipFrameName, flange.parentJoint, flange_frame_id_, flange.placement * initial_offset,
      pinocchio::OP_FRAME);
    tip_frame_id_ = model_.addFrame(tip);
    shaft_frame_id_ = tip_frame_id_;  // the RCM is taken at the tip without a wrist
  }

  // Data must be rebuilt after the model gains a frame.
  data_ = pinocchio::Data(model_);

  joint_q_index_.clear();
  joint_q_index_.reserve(params_.joints.size());
  for (const auto & name : params_.joints) {
    const pinocchio::JointIndex jid = model_.getJointId(name);
    joint_q_index_.push_back(model_.joints[jid].idx_q());
  }
  if (params_.wrist.enabled) {
    for (const auto & name : params_.wrist.joints) {
      joint_q_index_.push_back(model_.joints[model_.getJointId(name)].idx_q());
    }
  }

  return true;
}

controller_interface::CallbackReturn CartesianServoController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (params_.command_interface == "velocity" && params_.tracking.mode != "rate") {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "command_interface 'velocity' needs tracking.mode 'rate': position tracking plans to "
      "positions and would leave the velocity command without a position loop.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.joints.size() != kNumJoints) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "This controller is built for exactly %zu joints but %zu were configured. The joint count "
      "is fixed at compile time so that the trajectory generator's state stays on the stack.",
      kNumJoints, params_.joints.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  wrist_ = params_.wrist.enabled;
  if (wrist_) {
    if (params_.tracking.mode != "rate" || params_.tracking.solver != "qp") {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "wrist.enabled needs tracking.mode 'rate' and tracking.solver 'qp' (the IK path is "
        "6-joint).");
      return controller_interface::CallbackReturn::ERROR;
    }
    const auto & w = params_.wrist;
    if (w.joints.size() != kWristJoints || w.lower_limits.size() != kWristJoints ||
      w.upper_limits.size() != kWristJoints || w.max_velocity.size() != kWristJoints ||
      w.max_acceleration.size() != kWristJoints || w.max_jerk.size() != kWristJoints)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "wrist.* arrays must have 3 entries (roll, pitch, yaw)");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  n_joints_ = wrist_ ? kNumJoints + kWristJoints : kNumJoints;
  for (std::size_t i = 0; i < kMaxJoints; ++i) {
    if (i < kNumJoints) {
      v_max_[i] = params_.joint_limits.max_velocity[i];
      a_max_[i] = params_.joint_limits.max_acceleration[i];
      j_max_[i] = params_.joint_limits.max_jerk[i];
    } else if (i < n_joints_) {
      v_max_[i] = params_.wrist.max_velocity[i - kNumJoints];
      a_max_[i] = params_.wrist.max_acceleration[i - kNumJoints];
      j_max_[i] = params_.wrist.max_jerk[i - kNumJoints];
    } else {  // disabled Ruckig DoFs: any valid limits
      v_max_[i] = a_max_[i] = j_max_[i] = 1.0;
    }
  }

  std::string error;
  if (!buildModel(error)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Kinematic model setup failed: %s", error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Preallocate every IK intermediate; update() must not touch the heap.
  q_seed_ = Eigen::VectorXd::Zero(model_.nq);
  q_solution_ = Eigen::VectorXd::Zero(model_.nq);
  q_measured_ = Eigen::VectorXd::Zero(model_.nq);
  q_command_ = Eigen::VectorXd::Zero(model_.nq);
  q_written_ = Eigen::VectorXd::Zero(model_.nq);
  q_integrated_ = Eigen::VectorXd::Zero(model_.nq);
  dq_ = Eigen::VectorXd::Zero(model_.nv);
  jacobian_ = pinocchio::Data::Matrix6x::Zero(6, model_.nv);
  q_lower_ = model_.lowerPositionLimit;
  q_upper_ = model_.upperPositionLimit;

  otg_ = std::make_unique<ruckig::Ruckig<kMaxJoints>>(1.0 / static_cast<double>(get_update_rate()));

  for (std::size_t i = 0; i < kMaxJoints; ++i) {
    otg_input_.enabled[i] = i < n_joints_;
    otg_input_.max_velocity[i] = v_max_[i];
    otg_input_.max_acceleration[i] = a_max_[i];
    otg_input_.max_jerk[i] = j_max_[i];
    otg_input_.target_velocity[i] = 0.0;
    otg_input_.target_acceleration[i] = 0.0;
  }
  otg_input_.control_interface = ruckig::ControlInterface::Position;
  otg_input_.synchronization = ruckig::Synchronization::Time;

  // Initial offset from parameters, so a known tool can come up already calibrated.
  Eigen::Quaterniond q0(
    params_.tool_tip_offset[6], params_.tool_tip_offset[3], params_.tool_tip_offset[4],
    params_.tool_tip_offset[5]);
  if (q0.norm() < 1e-9) {
    RCLCPP_ERROR(get_node()->get_logger(), "tool_tip_offset quaternion has zero norm");
    return controller_interface::CallbackReturn::ERROR;
  }
  q0.normalize();
  const pinocchio::SE3 initial_offset(
    q0.toRotationMatrix(), Eigen::Vector3d(
                             params_.tool_tip_offset[0], params_.tool_tip_offset[1],
                             params_.tool_tip_offset[2]));
  applied_tool_tip_offset_ = initial_offset;
  tool_tip_offset_buffer_.writeFromNonRT(initial_offset);
  if (!wrist_) {  // with the wrist the tip is defined by the instrument chain
    model_.frames[tip_frame_id_].placement =
      model_.frames[flange_frame_id_].placement * initial_offset;
  }
  if (!params_.tip_frame.empty() && !wrist_) {
    tip_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(get_node());
    publishTipFrame(initial_offset);
  }

  // /arts/collision_hold: an absolute name, one flag for every arm (arts_safety
  // collision_guard). Latched, so a controller that starts while the guard holds holds too.
  collision_hold_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/arts/collision_hold", rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      collision_hold_.store(msg->data, std::memory_order_release);
    });

  // <arm>/tool_tip_target (decision 18): a relative name, so the arm's namespace places it
  // (the controller node lives in /<arm>). One message carries the pose, the optional
  // feedforward and the optional jaw; it is split here, off the control thread, into the
  // buffers the cycle already reads.
  tool_tip_target_sub_ = get_node()->create_subscription<arts_interfaces::msg::ToolTipTarget>(
    "tool_tip_target", rclcpp::SystemDefaultsQoS(),
    [this](const arts_interfaces::msg::ToolTipTarget::SharedPtr msg) {
      if (!msg->header.frame_id.empty() && msg->header.frame_id != params_.base_frame) {
        // Targets in other tf frames (world, the camera) need a tf lookup this controller
        // does not do yet; refusing is safer than reading them as base-frame numbers.
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "tool_tip_target in frame '%s' ignored: only '%s' (or empty) is supported.",
          msg->header.frame_id.c_str(), params_.base_frame.c_str());
        return;
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg->header;
      pose.pose = msg->pose;
      if (msg->velocity_valid) {
        geometry_msgs::msg::TwistStamped twist;
        twist.header = msg->header;
        twist.twist = msg->velocity;
        target_velocity_buffer_.writeFromNonRT(twist);
        velocity_seq_.fetch_add(1, std::memory_order_release);
      }
      target_pose_buffer_.writeFromNonRT(pose);
      // Released after the write so the control cycle cannot observe the new count
      // against the old pose. (The jaw has no drive yet; jaw_valid is accepted and unused.)
      reference_seq_.fetch_add(1, std::memory_order_release);
    });
  qd_target_ = Eigen::VectorXd::Zero(model_.nv);

  tool_tip_state_pub_ = get_node()->create_publisher<arts_interfaces::msg::ToolTipState>(
    "tool_tip_state", rclcpp::SystemDefaultsQoS());
  tool_tip_state_rt_ =
    std::make_unique<realtime_tools::RealtimePublisher<arts_interfaces::msg::ToolTipState>>(
      tool_tip_state_pub_);
  tool_tip_state_msg_.header.frame_id = params_.base_frame;
  tool_tip_state_msg_.jaw = kNaN;
  tool_tip_state_msg_.jaw_velocity = kNaN;

  joint_command_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "joint_command", rclcpp::SystemDefaultsQoS());
  joint_command_rt_ =
    std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      joint_command_pub_);
  joint_command_msg_.name = params_.joints;
  if (wrist_) {
    wrist_names_ = params_.wrist.joints;
    joint_command_msg_.name.insert(
      joint_command_msg_.name.end(), wrist_names_.begin(), wrist_names_.end());
  }
  joint_command_msg_.position.assign(n_joints_, 0.0);
  joint_command_msg_.velocity.assign(n_joints_, 0.0);

  if (wrist_) {
    // <arm>/tool_joint_states from the instrument drive; parsed here, off the control thread.
    wrist_state_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "tool_joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        WristState w;
        std::size_t found = 0;
        for (std::size_t k = 0; k < kWristJoints; ++k) {
          for (std::size_t m = 0; m < msg->name.size() && m < msg->position.size(); ++m) {
            if (msg->name[m] == wrist_names_[k] && std::isfinite(msg->position[m])) {
              w.q[k] = msg->position[m];
              ++found;
              break;
            }
          }
        }
        w.valid = found == kWristJoints;
        w.stamp_ns = get_node()->now().nanoseconds();
        wrist_state_buffer_.writeFromNonRT(w);
      });
    wrist_command_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      "tool_joint_target", rclcpp::SystemDefaultsQoS());
    wrist_command_rt_ =
      std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        wrist_command_pub_);
    wrist_command_msg_.name = wrist_names_;
    wrist_command_msg_.position.assign(kWristJoints, 0.0);
    wrist_command_msg_.velocity.assign(kWristJoints, 0.0);
  }

  // Task-layer QP (decision 17). Both problem shapes are built and initialized here, so a
  // cycle never allocates and switching the RCM is only choosing which one runs.
  try {
    task_qp_.configure(model_.nv, wrist_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Task QP setup failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  jacobian_base_ = pinocchio::Data::Matrix6x::Zero(6, model_.nv);
  jacobian_shaft_ = pinocchio::Data::Matrix6x::Zero(6, model_.nv);
  jacobian_shaft_base_ = pinocchio::Data::Matrix6x::Zero(6, model_.nv);
  jacobian_roll_ref_ = pinocchio::Data::Matrix6x::Zero(6, model_.nv);
  qd_lower_ = Eigen::VectorXd::Zero(model_.nv);
  qd_upper_ = Eigen::VectorXd::Zero(model_.nv);
  qd_qp_ = Eigen::VectorXd::Zero(model_.nv);
  q_posture_ = Eigen::VectorXd::Zero(model_.nq);
  shaft_point_jacobian_ = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, model_.nv);
  qd_rest_ = Eigen::VectorXd::Zero(model_.nv);
  const double w_ori = params_.task_qp.orientation_weight;
  task_weights_ << 1.0, 1.0, 1.0, w_ori, w_ori, w_ori;
  if (!params_.task_qp.joint_weights.empty()) {
    if (params_.task_qp.joint_weights.size() != n_joints_) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "task_qp.joint_weights has %zu entries, expected %zu (or none)",
        params_.task_qp.joint_weights.size(), n_joints_);
      return controller_interface::CallbackReturn::ERROR;
    }
    Eigen::VectorXd w = Eigen::VectorXd::Ones(model_.nv);
    for (std::size_t i = 0; i < n_joints_; ++i) {
      if (!(params_.task_qp.joint_weights[i] > 0.0)) {
        RCLCPP_ERROR(get_node()->get_logger(), "task_qp.joint_weights must all be > 0");
        return controller_interface::CallbackReturn::ERROR;
      }
      w[joint_q_index_[i]] = params_.task_qp.joint_weights[i];
    }
    task_qp_.setJointWeights(w);
  }
  shaft_axis_ = Eigen::Vector3d(
    params_.rcm.shaft_axis[0], params_.rcm.shaft_axis[1], params_.rcm.shaft_axis[2]);
  if (shaft_axis_.norm() < 1e-9) {
    RCLCPP_ERROR(get_node()->get_logger(), "rcm.shaft_axis has zero length");
    return controller_interface::CallbackReturn::ERROR;
  }
  shaft_axis_.normalize();
  if (wrist_) {
    shaft_axis_ = Eigen::Vector3d::UnitZ();  // frame E's z is the shaft by construction
  }
  if (params_.rcm.insertion_limits[0] >= params_.rcm.insertion_limits[1]) {
    RCLCPP_ERROR(get_node()->get_logger(), "rcm.insertion_limits must be [min, max], min < max");
    return controller_interface::CallbackReturn::ERROR;
  }
  rcm_point_.setConstant(kNaN);
  rcm_enabled_ = false;
  rcm_enabled_flag_.store(false);
  last_insertion_depth_ = params_.rcm.default_insertion_depth;

  rcm_state_pub_ = get_node()->create_publisher<arts_interfaces::msg::RcmState>(
    "rcm_state", rclcpp::SystemDefaultsQoS());
  rcm_state_rt_ =
    std::make_unique<realtime_tools::RealtimePublisher<arts_interfaces::msg::RcmState>>(
      rcm_state_pub_);
  rcm_state_msg_.header.frame_id = params_.base_frame;

  set_rcm_srv_ = get_node()->create_service<arts_interfaces::srv::SetRcm>(
    "set_rcm",  // relative: /<arm>/set_rcm
    [this](
      const std::shared_ptr<arts_interfaces::srv::SetRcm::Request> request,
      std::shared_ptr<arts_interfaces::srv::SetRcm::Response> response) {
      handleSetRcm(request, response);
    });

  set_tool_tip_offset_srv_ = get_node()->create_service<arts_interfaces::srv::SetToolTipOffset>(
    "set_tool_tip_offset",  // relative: /<arm>/set_tool_tip_offset (decision 18)
    [this](
      const std::shared_ptr<arts_interfaces::srv::SetToolTipOffset::Request> request,
      std::shared_ptr<arts_interfaces::srv::SetToolTipOffset::Response> response) {
      handleSetToolTipOffset(request, response);
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured for %zu joints, servoing '%s' relative to '%s' at %u Hz.", n_joints_,
    kTipFrameName, params_.base_frame.c_str(), get_update_rate());

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
CartesianServoController::on_export_reference_interfaces_list()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
  interfaces.reserve(kNumReferenceInterfaces);

  for (const auto * name : kReferenceInterfaceNames) {
    hardware_interface::InterfaceInfo info;
    info.name = name;
    info.data_type = "double";
    info.size = 1;
    info.enable_limits = false;
    // Empty initial value means NaN, which is what marks "nobody has written a reference
    // yet". The control loop treats NaN as hold-position rather than as a pose, so a
    // chained upstream that has not produced its first command cannot fling the arm at
    // the origin.
    info.initial_value = "";
    interfaces.push_back(
      std::make_shared<hardware_interface::CommandInterface>(
        hardware_interface::InterfaceDescription(get_node()->get_name(), info)));
  }
  return interfaces;
}

bool CartesianServoController::on_set_chained_mode(bool /*chained_mode*/)
{
  // Both modes are supported. The base class only calls update_reference_from_subscribers
  // when unchained, so the subscription simply stops being consulted; leaving it active
  // costs nothing and means switching back does not need a reconfigure.
  return true;
}

void CartesianServoController::seedFromMeasuredState()
{
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto value = state_interfaces_[i].get_optional<double>();
    q_measured_[joint_q_index_[i]] = value.value_or(0.0);
  }
  if (wrist_) {
    const WristState * w = wrist_state_buffer_.readFromRT();
    for (std::size_t k = 0; k < kWristJoints; ++k) {
      q_measured_[joint_q_index_[kNumJoints + k]] = (w != nullptr && w->valid) ? w->q[k] : 0.0;
    }
    shaftPoseAt(q_measured_);
    roll_ref_rotation_ =
      data_.oMf[base_frame_id_].actInv(data_.oMf[roll_ref_frame_id_]).rotation();
  }

  q_seed_ = q_measured_;
  q_solution_ = q_measured_;
  q_command_ = q_measured_;
  q_written_ = q_measured_;
  q_posture_ = q_measured_;

  for (std::size_t i = 0; i < n_joints_; ++i) {
    otg_input_.current_position[i] = q_measured_[joint_q_index_[i]];
    otg_input_.current_velocity[i] = 0.0;
    otg_input_.current_acceleration[i] = 0.0;
    otg_input_.target_position[i] = q_measured_[joint_q_index_[i]];
    otg_input_.target_velocity[i] = 0.0;
    otg_input_.target_acceleration[i] = 0.0;
    otg_output_.new_position[i] = q_measured_[joint_q_index_[i]];
    otg_output_.new_velocity[i] = 0.0;
    otg_output_.new_acceleration[i] = 0.0;
  }
  otg_->reset();

  seeded_ = true;
  ik_fault_ = false;
  consecutive_ik_misses_ = 0;
}

controller_interface::CallbackReturn CartesianServoController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (state_interfaces_.size() != kNumJoints || command_interfaces_.size() != kNumJoints) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu state and command interfaces, got %zu and %zu",
      kNumJoints, state_interfaces_.size(), command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Seeding from the measured state before the first cycle is what makes activation
  // bumpless. Without it the first command is whatever the previous run left behind.
  seedFromMeasuredState();

  // Leave the reference interfaces at NaN so that "no command yet" stays distinguishable
  // from "commanded to the current pose". They mean different things when the upstream
  // controller is late.
  for (auto & interface : ordered_exported_reference_interfaces_) {
    if (!interface->set_value(kNaN)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not initialize reference interfaces");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Discard anything published while the controller was inactive instead of acting on it
  // one cycle after activation. Note this runs inside the control loop, so it must not
  // allocate -- which rules out RealtimeBuffer::reset(), whose whole job is to new up a
  // fresh pair of buffers.
  consumed_reference_seq_ = reference_seq_.load(std::memory_order_acquire);
  last_reference_stamp_ = get_node()->now();
  consumed_velocity_seq_ = velocity_seq_.load(std::memory_order_acquire);
  ff_saturation_weight_ = 1.0;
  last_ff_velocity_.fill(0.0);
  ff_acceleration_.fill(0.0);
  ff_jerk_.fill(0.0);
  last_velocity_stamp_ = last_reference_stamp_;
  twist_ff_base_.setZero();

  // The RCM always starts OFF: whatever was inserted before a deactivation may not be any
  // more, so it has to be switched on again against the arm as it now is. A request queued
  // while inactive is dropped (its caller has already timed out).
  rcm_enabled_ = false;
  rcm_enabled_flag_.store(false);
  rcm_fault_ = false;
  qp_failed_ = false;
  switch_hold_ = false;
  consumed_rcm_request_seq_ = rcm_request_seq_.load(std::memory_order_acquire);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianServoController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // A velocity command left in the interface would keep the joints moving.
  if (params_.command_interface == "velocity") {
    for (auto & interface : command_interfaces_) {
      (void)interface.set_value(0.0);
    }
  }
  if (wrist_) {
    publishWristCommand(get_node()->now(), true);
  }
  seeded_ = false;
  have_last_tip_pose_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianServoController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  const std::uint64_t seq = reference_seq_.load(std::memory_order_acquire);
  if (seq == consumed_reference_seq_) {
    // Nothing new. Leaving the exported interfaces untouched is what lets the reference
    // go stale, and going stale is the point -- a reference that refreshed itself every
    // cycle from a latched buffer could never trip the timeout below.
    return controller_interface::return_type::OK;
  }
  consumed_reference_seq_ = seq;

  const auto * msg = target_pose_buffer_.readFromRT();
  const auto & p = msg->pose.position;
  const auto & o = msg->pose.orientation;
  const std::array<double, kNumReferenceInterfaces> values = {p.x, p.y, p.z, o.x, o.y, o.z, o.w};

  for (std::size_t i = 0; i < kNumReferenceInterfaces; ++i) {
    if (!ordered_exported_reference_interfaces_[i]->set_value(values[i])) {
      return controller_interface::return_type::ERROR;
    }
  }
  last_reference_stamp_ = time;
  return controller_interface::return_type::OK;
}

pinocchio::SE3 CartesianServoController::tipPoseAt(const Eigen::VectorXd & q)
{
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[base_frame_id_].actInv(data_.oMf[tip_frame_id_]);
}

double CartesianServoController::cartesianJointRateBound(
  const Eigen::VectorXd & from, const Eigen::VectorXd & to)
{
  // The Cartesian speed limit has to be imposed on Ruckig's VELOCITY LIMITS rather than on
  // its target, and this is the conversion. Moving from `from` toward `to` traces a
  // direction in joint space; the Jacobian says how much tip speed a unit of joint speed
  // along that direction buys; dividing the allowed tip speed by that ratio gives the
  // joint speed the limit allows.
  //
  // Doing it this way is what lets Ruckig keep planning real jerk-limited profiles. The
  // obvious alternative -- clamp the pose handed to IK to one cycle's worth of Cartesian
  // travel -- reads better but does not work, for reasons recorded at the IK call site.
  pinocchio::difference(model_, from, to, dq_);
  const double dq_norm = dq_.norm();

  joint_rate_share_.fill(0.0);
  if (dq_norm < 1e-12) {
    return std::numeric_limits<double>::infinity();
  }
  for (std::size_t i = 0; i < n_joints_; ++i) {
    joint_rate_share_[i] = std::abs(dq_[joint_q_index_[i]]) / dq_norm;
  }

  // LOCAL_WORLD_ALIGNED rather than LOCAL only so the linear and angular blocks separate
  // the way the two limits are written. Only the norms are used and those are invariant
  // under the frame's orientation, so the choice is presentational.
  pinocchio::computeFrameJacobian(
    model_, data_, from, tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);
  tip_twist_.noalias() = jacobian_ * (dq_ / dq_norm);

  double bound = std::numeric_limits<double>::infinity();
  const double linear_per_rate = tip_twist_.head<3>().norm();
  const double angular_per_rate = tip_twist_.tail<3>().norm();
  if (linear_per_rate > 1e-9) {
    bound = std::min(bound, params_.cartesian.max_linear_velocity / linear_per_rate);
  }
  if (angular_per_rate > 1e-9) {
    bound = std::min(bound, params_.cartesian.max_angular_velocity / angular_per_rate);
  }
  return bound;
}

bool CartesianServoController::solveIk(
  const pinocchio::SE3 & target, const Eigen::VectorXd & seed, Eigen::VectorXd & solution)
{
  solution = seed;

  for (int iteration = 0; iteration < params_.ik.max_iterations; ++iteration) {
    pinocchio::forwardKinematics(model_, data_, solution);
    pinocchio::updateFramePlacements(model_, data_);

    const pinocchio::SE3 base_to_tip = data_.oMf[base_frame_id_].actInv(data_.oMf[tip_frame_id_]);
    error_ = pinocchio::log6(base_to_tip.actInv(target)).toVector();

    if (error_.head<3>().norm() < params_.ik.position_tolerance &&
        error_.tail<3>().norm() < params_.ik.orientation_tolerance) {
      return true;
    }

    pinocchio::computeFrameJacobian(
      model_, data_, solution, tip_frame_id_, pinocchio::LOCAL, jacobian_);

    jjt_.noalias() = jacobian_ * jacobian_.transpose();
    jjt_.diagonal().array() += params_.ik.damping;
    // Damped least squares rather than the plain pseudoinverse the v1.4 reference used.
    // Undamped is fine until the arm nears a singularity, at which point the solution
    // magnitude blows up and the joint command with it.
    const Eigen::Matrix<double, 6, 1> lambda = jjt_.ldlt().solve(error_);
    dq_.noalias() = jacobian_.transpose() * lambda;

    // Via a scratch vector because `solution` is very often q_solution_ itself at the call
    // site, and integrate() reads its first argument while writing its last.
    pinocchio::integrate(model_, solution, dq_, q_integrated_);
    solution = q_integrated_;

    for (int i = 0; i < model_.nq; ++i) {
      solution[i] = std::clamp(solution[i], q_lower_[i], q_upper_[i]);
    }
  }

  // Ran out of iterations. Report the residual so the caller can decide; a near miss on
  // a fast-moving target is normal and self-corrects next cycle.
  return error_.head<3>().norm() < params_.ik.position_tolerance &&
         error_.tail<3>().norm() < params_.ik.orientation_tolerance;
}

controller_interface::return_type CartesianServoController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (dt <= 0.0) {
    return controller_interface::return_type::OK;
  }

  // 1. Adopt a new tool-tip offset if one arrived. Writing a frame placement is twelve
  //    doubles; the v1.4 equivalent rebuilt four solvers and leaked the old ones.
  const pinocchio::SE3 * requested_offset = tool_tip_offset_buffer_.readFromRT();
  if (!wrist_ && requested_offset != nullptr &&
    !requested_offset->isApprox(applied_tool_tip_offset_))
  {
    applied_tool_tip_offset_ = *requested_offset;
    model_.frames[tip_frame_id_].placement =
      model_.frames[flange_frame_id_].placement * applied_tool_tip_offset_;
  }

  // 2. Read the arm. A dropped read is treated as a fault rather than as a zero, because
  //    a zero here would look like a perfectly plausible joint angle.
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto value = state_interfaces_[i].get_optional<double>();
    if (!value.has_value() || !std::isfinite(*value)) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "No finite position for joint '%s'; holding.", params_.joints[i].c_str());
      return controller_interface::return_type::OK;
    }
    q_measured_[joint_q_index_[i]] = *value;
  }
  // The wrist's newest sample (the drive's 333 Hz against this 500 Hz loop: a sample may be
  // used twice). Stale or missing: hold the arm and zero the wrist (below).
  wrist_stale_ = wrist_ && !readWrist(time);

  if (!seeded_) {
    seedFromMeasuredState();
  }

  // Velocity commands close no position loop by themselves: Ruckig and the rate law start
  // every cycle from the MEASURED joints, so drift between commanded and actual motion is
  // seen and corrected instead of integrated.
  const bool velocity_out = params_.command_interface == "velocity";
  if (velocity_out) {
    for (std::size_t i = 0; i < n_joints_; ++i) {
      otg_input_.current_position[i] = q_measured_[joint_q_index_[i]];
    }
    q_command_ = q_measured_;
  }

  // 2b. RCM: adopt a pending set_rcm request, then measure the shaft against the point.
  //     The measured deviation is both what rcm_state reports and the fault check.
  processRcmRequest();
  if (rcm_point_.allFinite()) {
    measured_rcm_ = rcmGeometry(shaftPoseAt(q_measured_), rcm_point_);
    if (rcm_enabled_ && !rcm_fault_ && measured_rcm_.deviation > params_.rcm.fault_deviation) {
      rcm_fault_ = true;
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "RCM deviation %.2f mm exceeds %.2f mm: holding until the next set_rcm call. Something "
        "other than this controller moved the shaft.",
        1000.0 * measured_rcm_.deviation, 1000.0 * params_.rcm.fault_deviation);
    }
  }
  qp_failed_ = false;
  rcm_rows_fresh_ = false;

  // 3. Collect the reference. NaN on any component means nobody has commanded yet.
  bool reference_valid = true;
  std::array<double, kNumReferenceInterfaces> reference{};
  for (std::size_t i = 0; i < kNumReferenceInterfaces; ++i) {
    const auto value = ordered_exported_reference_interfaces_[i]->get_optional<double>();
    if (!value.has_value() || !std::isfinite(*value)) {
      reference_valid = false;
      break;
    }
    reference[i] = *value;
  }

  // A stale reference is as dangerous as a missing one: the v1.4 reference had no timeout
  // at all, so a dead upstream publisher left the last target latched indefinitely.
  if (reference_valid && params_.reference_timeout > 0.0 && !is_in_chained_mode()) {
    if ((time - last_reference_stamp_).seconds() > params_.reference_timeout) {
      reference_valid = false;
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Reference older than %.3f s; holding position.", params_.reference_timeout);
    }
  }
  // After an RCM switch, hold until the source has sent something newer (SetRcm.srv).
  if (switch_hold_) {
    if (!is_in_chained_mode() &&
      reference_seq_.load(std::memory_order_acquire) == switch_reference_seq_)
    {
      reference_valid = false;
    } else {
      switch_hold_ = false;
    }
  }
  if (rcm_fault_) {
    reference_valid = false;
  }
  if (collision_hold_.load(std::memory_order_acquire)) {
    reference_valid = false;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Collision guard hold (/arts/collision_hold); holding position.");
  }
  if (wrist_stale_) {
    reference_valid = false;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "No fresh wrist state on tool_joint_states (older than %.0f ms); holding.",
      1000.0 * params_.wrist.state_timeout);
  }
  const bool rate_mode = params_.tracking.mode == "rate";
  // The task-layer QP runs whenever the RCM is on, and without it when configured to.
  const bool use_qp = rate_mode && (rcm_enabled_ || params_.tracking.solver == "qp");

  if (reference_valid && !ik_fault_) {
    Eigen::Quaterniond orientation(reference[6], reference[3], reference[4], reference[5]);
    const double norm = orientation.norm();
    if (norm < 1e-9) {
      reference_valid = false;
    } else {
      // Normalizing rather than trusting the sender. The v1.4 reference built a rotation
      // straight from an unnormalized quaternion and carried a code comment admitting the
      // result was wrong in that case.
      orientation.coeffs() /= norm;
      const pinocchio::SE3 target(
        orientation.toRotationMatrix(),
        Eigen::Vector3d(reference[0], reference[1], reference[2]));
      target_position_ = target.translation();
      target_pose_ = target;

      // 4. Solve IK straight to the reference pose, warm-started from the joint
      //    configuration the arm is actually being commanded to -- Ruckig's last output --
      //    rather than from a pose the controller carries forward on its own. Anchoring to
      //    the achieved command is what keeps a lagging arm from accumulating a setpoint it
      //    later has to chase at the joint velocity limit, which at the UR5e's reach is
      //    something like 850 mm/s at the tool.
      //
      // THE TARGET IS THE FULL REFERENCE, NOT A POSE CLAMPED TO ONE CYCLE'S TRAVEL.
      // Clamping it is the obvious way to express a Cartesian speed limit and it is a trap.
      // Ruckig's Position interface plans to ARRIVE at its target, so a target permanently
      // one cycle ahead asks it to accelerate and stop inside a tenth of a millimetre;
      // under a sane acceleration limit the profile that fits is very slow, and the arm
      // crawled at 3 mm/s against a 60 mm/s limit (mock hardware, 2026-08-12).
      //
      // Handing Ruckig a target velocity to compensate -- the trick MoveIt's ruckig_filter
      // uses -- is worse, and the failure is instructive. Ruckig reads the pair as "be AT
      // that point TRAVELLING at that speed", and reaching a point 0.0005 rad away at
      // 0.5 rad/s from rest is infeasible under the acceleration limit, so its minimum-time
      // answer is to reverse first and gain runway. The next cycle anchors the target to
      // the command that just moved backwards, so it reverses again. Commanded +50 mm in x,
      // the arm travelled 800 mm in -x at very nearly the speed limit, and the IK residual
      // was zero throughout, which is what made it look like a kinematics problem.
      //
      // So Ruckig gets a stationary goal, which is what it is for, and the Cartesian speed
      // limit is imposed below as a bound on its velocity limits instead.
      for (std::size_t i = 0; i < n_joints_; ++i) {
        if (!velocity_out) {
          q_command_[joint_q_index_[i]] = otg_output_.new_position[i];
        }
      }

      q_seed_ = q_command_;
      if (use_qp) {
        // The QP works on the Cartesian error directly; no IK solution to chase. Kept equal
        // to the command so the position-mode rate bound below has nothing to say.
        q_solution_ = q_command_;
        consecutive_ik_misses_ = 0;
      } else if (!solveIk(target, q_seed_, q_solution_)) {
        const double residual = error_.head<3>().norm();
        // Not a fault. Persistent misses mean the arm is against a joint limit or a
        // singularity and has stopped at the closest pose it can reach, which is the
        // behaviour we want and recovers by itself once a reachable pose is commanded. An
        // earlier revision latched a fault needing deactivate/reactivate on any single
        // cycle missing by more than 0.1 mm; on the first bringup that fired one cycle
        // after activation. An arm that has to be manually re-armed the first time an
        // operator pushes toward a workspace boundary is not safer, it is unusable.
        //
        // A miss also leaves the best effort the solver reached in q_solution_, which for
        // an out-of-reach target is the closest pose on the boundary. That is exactly what
        // should be handed to Ruckig, so there is nothing to substitute here.
        ++consecutive_ik_misses_;
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "IK has been short of tolerance for %u cycles (residual %.3f mm). The arm is "
          "most likely against a joint limit or a singularity; it is holding at the "
          "closest pose it can reach and will resume when a reachable pose is commanded.",
          consecutive_ik_misses_, 1000.0 * residual);
      } else {
        consecutive_ik_misses_ = 0;
      }

      // A non-finite solution is different in kind: it cannot be clamped back to anything
      // meaningful and must never reach the joints.
      if (!q_solution_.allFinite()) {
        ik_fault_ = true;
        q_solution_ = q_measured_;
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "IK produced a non-finite solution; holding position. Clear by deactivating and "
          "reactivating the controller.");
      }
    }
  }

  // 5. Smooth in joint space. Ruckig runs even when holding, so that a hold is itself a
  //    jerk-limited stop rather than an instant one.
  const bool tracking = reference_valid && !ik_fault_;

  // 4b. Rate mode feedforward. Only the unchained subscription path carries it; a chained
  //     upstream gets V_ff = 0 (pure proportional following) until it exports a twist.
  if (rate_mode && !is_in_chained_mode()) {
    const std::uint64_t vseq = velocity_seq_.load(std::memory_order_acquire);
    if (vseq != consumed_velocity_seq_) {
      consumed_velocity_seq_ = vseq;
      const auto * v = target_velocity_buffer_.readFromRT();
      twist_ff_base_ << v->twist.linear.x, v->twist.linear.y, v->twist.linear.z,
        v->twist.angular.x, v->twist.angular.y, v->twist.angular.z;
      if (!twist_ff_base_.allFinite()) {
        twist_ff_base_.setZero();
      }
      last_velocity_stamp_ = time;
    }
    if ((time - last_velocity_stamp_).seconds() > params_.reference_timeout) {
      twist_ff_base_.setZero();
    }
  }

  // Turn the Cartesian speed limit into a joint-rate bound for this cycle's direction of
  // travel, then split it across the joints in proportion to how far each has to move, so
  // that all six hit their limit together and the profile keeps the direction IK chose.
  const double rate_bound =
    tracking ? cartesianJointRateBound(q_command_, q_solution_)
             : std::numeric_limits<double>::infinity();

  // Rate mode: the joint velocity Ruckig is driven to. Feedforward J+ V_ff (damped, at the
  // commanded configuration, twist rotated from base_frame into the tip's LOCAL frame the
  // Jacobian uses) plus gain times the joint-space error to the IK solution. Then scaled as a
  // whole so the tip stays under the Cartesian caps; the governor below still backs it up.
  if (tracking && rate_mode) {
    pinocchio::forwardKinematics(model_, data_, q_command_);
    pinocchio::updateFramePlacements(model_, data_);
    const pinocchio::SE3 base_to_tip = data_.oMf[base_frame_id_].actInv(data_.oMf[tip_frame_id_]);
    // Fade the feedforward when the arm cannot follow it. When a commanded motion asks for
    // more acceleration or jerk than the joint limits allow, Ruckig clips the velocity
    // unevenly and full feedforward then walks the arm off to one side: on mock a 10 Hz shake
    // drifted it to 1.7x the commanded amplitude, a 4 Hz one at the old limits to 2.9x
    // (2026-10-30). Three signals, the weight is the smallest:
    //  (1) feasibility of the feedforward itself: the joint acceleration and jerk it would
    //      need (finite differences, ~10 ms smoothing) against the limits, full at <= 70 %,
    //      none at >= 100 %. A 4 Hz, 15 mm shake needs ~60 %; 10 Hz far more.
    //  (2) Ruckig's output acceleration within 90 % of a joint's limit: drops the weight over
    //      ~50 ms, recovers over ~0.5 s. (Checking Ruckig's output JERK instead fired on
    //      followable shakes too, which ride the jerk limit, and cut 4 Hz to 0.37.)
    //  (3) a backstop on the tip's distance to its target (feedforward_fade), so a large
    //      drift can never be driven further. A 2-5 mm backstop throttled a followable
    //      4 Hz shake, whose normal lag error is ~7 mm.
    bool saturated = false;
    for (std::size_t i = 0; i < n_joints_; ++i) {
      saturated = saturated || std::abs(otg_output_.new_acceleration[i]) > 0.9 * a_max_[i];
    }
    ff_saturation_weight_ = std::clamp(
      ff_saturation_weight_ + (saturated ? -dt / 0.05 : dt / 0.5), 0.0, 1.0);
    const double tracking_error = (target_position_ - base_to_tip.translation()).norm();
    const double fade_lo = params_.tracking.feedforward_fade[0];
    const double fade_hi = params_.tracking.feedforward_fade[1];
    double ff_weight = std::min(
      ff_saturation_weight_,
      std::clamp((fade_hi - tracking_error) / std::max(fade_hi - fade_lo, 1e-9), 0.0, 1.0));
    tip_twist_.head<3>().noalias() = base_to_tip.rotation().transpose() * twist_ff_base_.head<3>();
    tip_twist_.tail<3>().noalias() = base_to_tip.rotation().transpose() * twist_ff_base_.tail<3>();
    pinocchio::computeFrameJacobian(
      model_, data_, q_command_, tip_frame_id_, pinocchio::LOCAL, jacobian_);
    jjt_.noalias() = jacobian_ * jacobian_.transpose();
    jjt_.diagonal().array() += params_.ik.damping;
    const Eigen::Matrix<double, 6, 1> lambda = jjt_.ldlt().solve(tip_twist_);
    qd_target_.noalias() = jacobian_.transpose() * lambda;  // unweighted joint feedforward

    constexpr double kSmooth = 0.01;  // s
    const double alpha = dt / (kSmooth + dt);
    double need = 0.0;
    for (std::size_t i = 0; i < n_joints_; ++i) {
      const int q = joint_q_index_[i];
      const double acc = (qd_target_[q] - last_ff_velocity_[i]) / dt;
      last_ff_velocity_[i] = qd_target_[q];
      const double acc_f = ff_acceleration_[i] + alpha * (acc - ff_acceleration_[i]);
      const double jerk = (acc_f - ff_acceleration_[i]) / dt;
      ff_acceleration_[i] = acc_f;
      ff_jerk_[i] += alpha * (jerk - ff_jerk_[i]);
      need = std::max({need, std::abs(ff_acceleration_[i]) / a_max_[i],
                       std::abs(ff_jerk_[i]) / j_max_[i]});
    }
    ff_weight = std::min(ff_weight, std::clamp((1.0 - need) / 0.3, 0.0, 1.0));
    if (use_qp) {
      // The joint feedforward above served only to judge feasibility; the QP recomputes
      // the whole joint velocity from the weighted tool twist and the Cartesian error.
      if (solveTaskQp(base_to_tip, ff_weight)) {
        qd_target_ = qd_qp_;
      } else {
        qp_failed_ = true;
        qd_target_.setZero();
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Task QP failed (%s, proxsuite status %d after %ld iterations); holding.",
          rcm_enabled_ ? "RCM on" : "RCM off", task_qp_.lastStatus(), task_qp_.lastIterations());
      }
    } else {
      qd_target_ *= ff_weight;
      pinocchio::difference(model_, q_command_, q_solution_, dq_);
      qd_target_ += params_.tracking.gain * dq_;
    }

    // The QP has already capped the task velocity and bounded every joint (velocity,
    // acceleration, jerk, position); scaling or zeroing its answer here would break the RCM.
  }
  if (tracking && rate_mode && !use_qp) {
    const Eigen::Matrix<double, 6, 1> tip_rate = jacobian_ * qd_target_;
    double scale = 1.0;
    const double lin = tip_rate.head<3>().norm();
    const double ang = tip_rate.tail<3>().norm();
    if (lin > params_.cartesian.max_linear_velocity) {
      scale = std::min(scale, params_.cartesian.max_linear_velocity / lin);
    }
    if (ang > params_.cartesian.max_angular_velocity) {
      scale = std::min(scale, params_.cartesian.max_angular_velocity / ang);
    }
    for (std::size_t i = 0; i < n_joints_; ++i) {
      scale = std::min(scale, v_max_[i] / std::max(std::abs(qd_target_[joint_q_index_[i]]), 1e-12));
    }
    qd_target_ *= scale;
    // A joint at its position limit is not driven further out: the velocity interface
    // does not know about position limits.
    for (std::size_t i = 0; i < n_joints_; ++i) {
      const int q = joint_q_index_[i];
      if ((q_command_[q] >= q_upper_[q] && qd_target_[q] > 0.0) ||
          (q_command_[q] <= q_lower_[q] && qd_target_[q] < 0.0)) {
        qd_target_[q] = 0.0;
      }
    }
  }
  // In rate mode a hold is a jerk-limited stop (target velocity zero) where the arm is,
  // not a return to the last commanded position, which would brake past it and come back.
  otg_input_.control_interface =
    rate_mode ? ruckig::ControlInterface::Velocity : ruckig::ControlInterface::Position;
  if (rate_mode && !tracking) {
    qd_target_.setZero();
  }

  for (std::size_t i = 0; i < n_joints_; ++i) {
    const int q = joint_q_index_[i];
    otg_input_.target_position[i] = tracking ? q_solution_[q] : otg_output_.new_position[i];
    // Ruckig plans to rest at the target, which is correct here: the target is a real goal
    // rather than a setpoint one cycle ahead, and it is replaced every cycle anyway.
    otg_input_.target_velocity[i] = rate_mode ? qd_target_[q] : 0.0;

    // The floor keeps a joint that barely moves this cycle from being handed a zero or
    // negative velocity limit, which Ruckig rejects outright -- taking the whole
    // trajectory generator down over a joint that was not going anywhere.
    const double cartesian_bound = std::max(rate_bound * joint_rate_share_[i], 1e-6);
    // Rate mode has already scaled its target velocity to the Cartesian and joint caps.
    otg_input_.max_velocity[i] = rate_mode ? v_max_[i] : std::min(v_max_[i], cartesian_bound);
    otg_input_.max_acceleration[i] = a_max_[i];
    otg_input_.max_jerk[i] = j_max_[i];
  }

  // With the RCM on and no QP this cycle (holding), the rows are rebuilt at the current
  // state so the projection below still has them.
  if (rate_mode && rcm_enabled_ && !rcm_rows_fresh_) {
    const pinocchio::SE3 shaft = shaftPoseAt(q_command_);
    computeJacobianBase();
    buildRcmRows(shaft);
  }

  const ruckig::Result result = otg_->update(otg_input_, otg_output_);
  if (result >= 0 && rate_mode && rcm_enabled_) {
    projectOntoRcm(dt);
  }
  if (result < 0) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Trajectory generation failed (%d); holding.", static_cast<int>(result));
  } else {
    // 5b. Cartesian speed governor -- the last stage, and the only one that can actually
    //     guarantee the tool-space speed limit.
    //
    //     The bound above is computed from the Jacobian at the START of the cycle and is
    //     exact only for an infinitesimal step. That is enough almost everywhere and not
    //     enough near a singularity, which is where the two disagree most: there the
    //     Cartesian bound goes to infinity, the joint limits take over, Ruckig accelerates
    //     into them, and then -- because acceleration and jerk limits mean a velocity
    //     vector cannot turn instantly -- it carries that joint speed as the arm rotates
    //     out of the singularity. The same 1 rad/s that moved the tool a tenth of a
    //     millimetre now moves it most of a metre per second. Measured on mock hardware on
    //     2026-08-12: 0.51 mm written in a cycle that allowed 0.36 mm.
    //
    //     Scaling position, velocity and acceleration by the same factor keeps the state
    //     Ruckig is handed back self-consistent, so the next cycle plans from where the
    //     arm actually is rather than re-accelerating into the same violation. It does
    //     cost strict jerk-limiting in the cycles where it engages, which is the right
    //     trade: a governed profile is slightly less smooth, an ungoverned one is fast.
    for (std::size_t i = 0; i < n_joints_; ++i) {
      q_written_[joint_q_index_[i]] = otg_output_.new_position[i];
    }
    const double written_travel =
      (tipPoseAt(q_written_).translation() - tipPoseAt(q_command_).translation()).norm();
    const double allowed_travel = params_.cartesian.max_linear_velocity * dt;

    if (written_travel > allowed_travel && written_travel > 0.0) {
      const double scale = allowed_travel / written_travel;
      for (std::size_t i = 0; i < n_joints_; ++i) {
        const int q = joint_q_index_[i];
        otg_output_.new_position[i] =
          q_command_[q] + scale * (otg_output_.new_position[i] - q_command_[q]);
        otg_output_.new_velocity[i] *= scale;
        otg_output_.new_acceleration[i] *= scale;
      }
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Cartesian speed governor engaged: %.1f mm/s scaled back to the %.1f mm/s limit. "
        "Sustained engagement means the arm is working near a singularity.",
        1000.0 * written_travel / dt, 1000.0 * params_.cartesian.max_linear_velocity);
    }

    otg_output_.pass_to_input(otg_input_);
  }

  // 6. Write. A failed write is reported rather than ignored -- silently dropping a
  //    command on a surgical arm is the kind of thing that has to be visible.
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const double command =
      velocity_out ? otg_output_.new_velocity[i] : otg_output_.new_position[i];
    if (!command_interfaces_[i].set_value(command)) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Could not write %s command for joint '%s'.", params_.command_interface.c_str(),
        params_.joints[i].c_str());
      return controller_interface::return_type::ERROR;
    }
    joint_command_msg_.position[i] = otg_output_.new_position[i];
    joint_command_msg_.velocity[i] = otg_output_.new_velocity[i];
  }
  for (std::size_t i = kNumJoints; i < n_joints_; ++i) {
    joint_command_msg_.position[i] = otg_output_.new_position[i];
    joint_command_msg_.velocity[i] = otg_output_.new_velocity[i];
  }
  if (wrist_) {
    publishWristCommand(time, wrist_stale_ || result < 0);
  }

  // 7. Publish (decision 18). try_publish skips a cycle rather than block when the
  //    publisher thread is still busy with the last message.
  joint_command_msg_.header.stamp = time;
  joint_command_rt_->try_publish(joint_command_msg_);
  publishToolTipState(time, dt);
  publishRcmState(time);

  return controller_interface::return_type::OK;
}

void CartesianServoController::publishTipFrame(const pinocchio::SE3 & offset)
{
  if (!tip_tf_broadcaster_) {
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = get_node()->now();
  t.header.frame_id = params_.flange_frame;
  t.child_frame_id = params_.tip_frame;
  t.transform.translation.x = offset.translation().x();
  t.transform.translation.y = offset.translation().y();
  t.transform.translation.z = offset.translation().z();
  const Eigen::Quaterniond quat(offset.rotation());
  t.transform.rotation.x = quat.x();
  t.transform.rotation.y = quat.y();
  t.transform.rotation.z = quat.z();
  t.transform.rotation.w = quat.w();
  tip_tf_broadcaster_->sendTransform(t);
}

void CartesianServoController::publishToolTipState(const rclcpp::Time & time, double dt)
{
  // FK of the MEASURED joints: this is what the arm is doing, not what was asked. The twist
  // is the per-cycle difference (linear in base_frame, angular as the rotation vector of
  // the relative rotation), which is what the joint positions alone allow.
  const pinocchio::SE3 tip = tipPoseAt(q_measured_);
  auto & m = tool_tip_state_msg_;
  m.header.stamp = time;
  const Eigen::Quaterniond quat(tip.rotation());
  m.pose.position.x = tip.translation().x();
  m.pose.position.y = tip.translation().y();
  m.pose.position.z = tip.translation().z();
  m.pose.orientation.x = quat.x();
  m.pose.orientation.y = quat.y();
  m.pose.orientation.z = quat.z();
  m.pose.orientation.w = quat.w();
  if (have_last_tip_pose_) {
    const Eigen::Vector3d v = (tip.translation() - last_tip_pose_.translation()) / dt;
    const Eigen::Vector3d w =
      pinocchio::log3(tip.rotation() * last_tip_pose_.rotation().transpose()) / dt;
    m.velocity.linear.x = v.x();
    m.velocity.linear.y = v.y();
    m.velocity.linear.z = v.z();
    m.velocity.angular.x = w.x();
    m.velocity.angular.y = w.y();
    m.velocity.angular.z = w.z();
  }
  last_tip_pose_ = tip;
  have_last_tip_pose_ = true;
  tool_tip_state_rt_->try_publish(m);
}

CartesianServoController::RcmGeometry CartesianServoController::rcmGeometry(
  const pinocchio::SE3 & tip, const Eigen::Vector3d & point) const
{
  RcmGeometry g;
  g.axis = tip.rotation() * shaft_axis_;
  g.depth = g.axis.dot(tip.translation() - point);
  // Foot of the point on the INFINITE shaft line: the open laparoscopic platform's
  // definition of the RCM deviation, so the published number compares directly.
  const Eigen::Vector3d foot = tip.translation() - g.depth * g.axis;
  g.perpendicular = point - foot;
  g.deviation = g.perpendicular.norm();
  return g;
}

void CartesianServoController::computeJacobianBase()
{
  // Tip Jacobian with linear and angular velocity in base_frame axes. LOCAL_WORLD_ALIGNED
  // gives the tip-origin velocity in the model root's axes; base_frame is fixed to the
  // root, so rotating both blocks is all it takes. jacobian_ keeps the LOCAL_WORLD_ALIGNED
  // copy for buildRcmRows().
  pinocchio::computeFrameJacobian(
    model_, data_, q_command_, tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);
  r_base_t_ = data_.oMf[base_frame_id_].rotation().transpose();
  jacobian_base_.topRows<3>().noalias() = r_base_t_ * jacobian_.topRows<3>();
  jacobian_base_.bottomRows<3>().noalias() = r_base_t_ * jacobian_.bottomRows<3>();
}

void CartesianServoController::buildRcmRows(const pinocchio::SE3 & tip)
{
  // `tip` is the frame the shaft is measured at: the tip without the wrist, frame E with it
  // (whose Jacobians are computed here; computeJacobianBase() has set r_base_t_).
  if (wrist_) {
    pinocchio::computeFrameJacobian(
      model_, data_, q_command_, shaft_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_shaft_);
    jacobian_shaft_base_.topRows<3>().noalias() = r_base_t_ * jacobian_shaft_.topRows<3>();
    jacobian_shaft_base_.bottomRows<3>().noalias() = r_base_t_ * jacobian_shaft_.bottomRows<3>();
  }
  const pinocchio::Data::Matrix6x & j_lwa = wrist_ ? jacobian_shaft_ : jacobian_;
  const pinocchio::Data::Matrix6x & j_base = wrist_ ? jacobian_shaft_base_ : jacobian_base_;
  // RCM rows. The shaft's material point at the foot of the RCM point moves at
  // v_c = v_tip + w x (foot - tip) = (J_lin - [foot - tip]x J_ang) qd. Its components
  // normal to the shaft must remove `gain` x the deviation: n_i^T v_c = gain n_i^T perp.
  // (The deviation vector's rate is minus the normal part of v_c, to first order.)
  // Expects computeJacobianBase() at the same state, before any roll projection.
  const RcmGeometry g = rcmGeometry(tip, rcm_point_);
  const Eigen::Vector3d & s = g.axis;
  const Eigen::Vector3d r = -g.depth * s;  // foot - tip
  Eigen::Matrix3d r_cross;
  r_cross << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
  // Two unit normals to s, built from whichever base axis is least aligned with it.
  Eigen::Index least = 0;
  s.cwiseAbs().minCoeff(&least);
  const Eigen::Vector3d n1 = s.cross(Eigen::Vector3d::Unit(least)).normalized();
  const Eigen::Vector3d n2 = s.cross(n1);
  auto & rows = task_qp_.rcmRows();
  shaft_point_jacobian_.noalias() =
    j_base.topRows<3>() - r_cross * (r_base_t_ * j_lwa.bottomRows<3>());
  rows.a.row(0).noalias() = n1.transpose() * shaft_point_jacobian_;
  rows.a.row(1).noalias() = n2.transpose() * shaft_point_jacobian_;
  rows.b << params_.rcm.gain * n1.dot(g.perpendicular), params_.rcm.gain * n2.dot(g.perpendicular);
  // Insertion depth: d' = s^T v_tip to first order (s' is normal to s, and tip - point is
  // along s while on the RCM). Kept inside [min, max] by an approach-rate inequality.
  rows.c.noalias() = s.transpose() * j_base.topRows<3>();
  rows.c_lo = params_.rcm.insertion_gain * (params_.rcm.insertion_limits[0] - g.depth);
  rows.c_hi = params_.rcm.insertion_gain * (params_.rcm.insertion_limits[1] - g.depth);
  rcm_axis_ = s;
  rcm_rows_fresh_ = true;
}

void CartesianServoController::projectOntoRcm(double dt)
{
  // Ruckig smooths each joint on its own and cannot see the RCM: on mock its jerk-limited
  // lag between the QP's joint velocity and the one it sent held the shaft 0.22 mm (median)
  // off the point during a 1 Hz, 15 mm Lissajous (2026-11-01). Its output is therefore
  // projected back onto the RCM rows -- the smallest joint-velocity change that satisfies
  // them exactly, v - A^T (A A^T)^-1 (A v - b) -- after which the same motion held 0.002 mm.
  // The correction is the size of Ruckig's lag in the two normal directions only, so it
  // leaves Ruckig's limits all but untouched. Putting the limits inside the QP as boxes
  // instead was tried first: one-step jerk boxes overshoot (1.32x on a 2 Hz shake), because
  // they cannot plan the braking Ruckig plans.
  const auto & rows = task_qp_.rcmRows();
  for (std::size_t i = 0; i < n_joints_; ++i) {
    dq_[joint_q_index_[i]] = otg_output_.new_velocity[i];
  }
  const Eigen::Matrix<double, TaskQp::kRcmRows, 1> residual = rows.a * dq_ - rows.b;
  const Eigen::Matrix<double, TaskQp::kRcmRows, TaskQp::kRcmRows> aat =
    rows.a * rows.a.transpose();
  const Eigen::Matrix<double, TaskQp::kRcmRows, 1> lambda = aat.ldlt().solve(residual);
  if (!lambda.allFinite()) {
    return;  // A has lost rank (shaft through a singular pose); leave Ruckig's output.
  }
  dq_.noalias() -= rows.a.transpose() * lambda;
  for (std::size_t i = 0; i < n_joints_; ++i) {
    const double v = dq_[joint_q_index_[i]];
    otg_output_.new_position[i] += (v - otg_output_.new_velocity[i]) * dt;
    otg_output_.new_velocity[i] = v;
  }
}

bool CartesianServoController::solveTaskQp(const pinocchio::SE3 & tip, double ff_weight)
{
  // With the wrist the RCM is taken at frame E, not at the tip (decision 17, 2026-11-03).
  const pinocchio::SE3 shaft = wrist_ ? shaftPoseAt(q_command_) : tip;
  computeJacobianBase();
  if (rcm_enabled_) {
    buildRcmRows(shaft);  // before the roll projection below changes the angular rows
  }
  if (wrist_) {
    buildRollRow();
  }

  // Decision 17's law in task space: V_ff (weighted) + K e. The rotation error is taken in
  // base axes, R_target R_tip^T, so it pairs with the base-axes angular rows.
  const double k = params_.tracking.gain;
  const Eigen::Matrix3d r_error = target_pose_.rotation() * tip.rotation().transpose();
  task_velocity_.head<3>() =
    ff_weight * twist_ff_base_.head<3>() + k * (target_pose_.translation() - tip.translation());
  task_velocity_.tail<3>() = ff_weight * twist_ff_base_.tail<3>() + k * pinocchio::log3(r_error);

  // With the wrist, orientation is always the full task (decision 17, 2026-11-03).
  if (rcm_enabled_ && !wrist_ && params_.rcm.orientation_task == "none") {
    // Tip position only. The angular rows go to zero, so the roll about the shaft -- the one
    // DoF the RCM and the tip position leave -- is resolved by the posture term and held.
    task_velocity_.tail<3>().setZero();
    jacobian_base_.bottomRows<3>().setZero();
  } else if (rcm_enabled_ && !wrist_ && params_.rcm.orientation_task == "roll") {
    // Only the roll about the shaft is a task; the shaft direction is the constraint's.
    // The roll error is the TWIST of the error rotation about s (swing-twist split), which
    // stays the roll even when the shaft has pivoted far from the target orientation.
    const Eigen::Vector3d & s = rcm_axis_;
    Eigen::Quaterniond qe(r_error);
    if (qe.w() < 0.0) {
      qe.coeffs() *= -1.0;
    }
    const double twist = 2.0 * std::atan2(qe.vec().dot(s), qe.w());
    task_velocity_.tail<3>() = s * (ff_weight * s.dot(twist_ff_base_.tail<3>()) + k * twist);
    jacobian_base_.bottomRows<3>() = (s * s.transpose()) * jacobian_base_.bottomRows<3>();
  }

  // The speed caps act on the TASK: the tip's linear speed, and the commanded rotation
  // (only the roll, under the RCM). Not on the shaft's pivoting, which the RCM implies from
  // the tip motion (tip speed / depth): capping the tool's whole angular rate, as the IK
  // path does, throttled every lateral move under the RCM.
  const double lin = task_velocity_.head<3>().norm();
  if (lin > params_.cartesian.max_linear_velocity) {
    task_velocity_.head<3>() *= params_.cartesian.max_linear_velocity / lin;
  }
  const double ang = task_velocity_.tail<3>().norm();
  if (ang > params_.cartesian.max_angular_velocity) {
    task_velocity_.tail<3>() *= params_.cartesian.max_angular_velocity / ang;
  }

  // Joint velocity and position limits as boxes; acceleration and jerk stay with Ruckig,
  // downstream (see projectOntoRcm). The damping pulls toward the posture reference.
  const double k_lim = params_.task_qp.joint_limit_gain;
  for (std::size_t i = 0; i < n_joints_; ++i) {
    const int q = joint_q_index_[i];
    const double v_max = v_max_[i];
    qd_lower_[q] = std::max(-v_max, k_lim * (q_lower_[q] - q_command_[q]));
    qd_upper_[q] = std::min(v_max, k_lim * (q_upper_[q] - q_command_[q]));
    if (qd_lower_[q] > qd_upper_[q]) {  // only outside both limits at once: not physical
      qd_lower_[q] = qd_upper_[q] = 0.0;
    }
  }
  pinocchio::difference(model_, q_command_, q_posture_, qd_rest_);
  qd_rest_ *= params_.task_qp.posture_gain;
  const TaskQp::RcmRows * rows = rcm_enabled_ ? &task_qp_.rcmRows() : nullptr;
  // One cold retry: a warm start from an unlucky previous iterate has been seen to report a
  // false PRIMAL_INFEASIBLE on a plainly feasible problem (mock, 2026-11-01).
  for (const bool cold : {false, true}) {
    if (task_qp_.solve(
        jacobian_base_, task_velocity_, task_weights_, params_.task_qp.damping, qd_rest_,
        qd_lower_, qd_upper_, rows, qd_qp_, cold, wrist_ ? &task_qp_.rollRow() : nullptr))
    {
      return true;
    }
  }
  return false;
}

void CartesianServoController::processRcmRequest()
{
  const std::uint64_t seq = rcm_request_seq_.load(std::memory_order_acquire);
  if (seq == consumed_rcm_request_seq_) {
    return;
  }
  consumed_rcm_request_seq_ = seq;
  const RcmRequest request = *rcm_request_buffer_.readFromRT();
  RcmReply & reply = rcm_reply_;
  reply = RcmReply{};

  // The frame the RCM is measured at every cycle (step 2b): frame E with the wrist, the tip
  // without. Placing the point from the TIP with the wrist put it on the jaw's axis, off the
  // shaft by depth * sin(wrist bend), and the per-cycle check latched a fault in the same
  // cycle (real arm, 2026-11-07: 12.45 mm at ~8 deg). The depth is then from frame E.
  const pinocchio::SE3 tip = wrist_ ? shaftPoseAt(q_measured_) : tipPoseAt(q_measured_);
  if (!request.enable) {
    rcm_enabled_ = false;
    reply.success = true;
    std::snprintf(reply.message.data(), reply.message.size(), "RCM off");
  } else if (params_.tracking.mode != "rate") {
    std::snprintf(
      reply.message.data(), reply.message.size(),
      "the RCM needs tracking.mode rate (the QP produces joint velocities)");
  } else {
    // No depth given: reuse the last engaged RCM's, so off -> move -> on relocates the
    // point onto the new shaft at the same depth.
    const double depth =
      request.insertion_depth > 0.0 ? request.insertion_depth : last_insertion_depth_;
    const Eigen::Vector3d point = request.from_current_shaft ?
      Eigen::Vector3d(tip.translation() - depth * (tip.rotation() * shaft_axis_)) :
      request.point;
    const RcmGeometry g = rcmGeometry(tip, point);
    reply.point = point;
    reply.deviation = g.deviation;
    reply.insertion_depth = g.depth;
    if (g.deviation > params_.rcm.engage_max_deviation) {
      std::snprintf(
        reply.message.data(), reply.message.size(),
        "refused: the shaft passes %.2f mm from the point (at most %.2f mm to engage)",
        1000.0 * g.deviation, 1000.0 * params_.rcm.engage_max_deviation);
    } else if (g.depth < params_.rcm.insertion_limits[0] || g.depth > params_.rcm.insertion_limits[1]) {
      std::snprintf(
        reply.message.data(), reply.message.size(),
        "refused: tip %.1f mm beyond the point, outside rcm.insertion_limits [%.1f, %.1f] mm",
        1000.0 * g.depth, 1000.0 * params_.rcm.insertion_limits[0],
        1000.0 * params_.rcm.insertion_limits[1]);
    } else {
      rcm_point_ = point;
      rcm_enabled_ = true;
      measured_rcm_ = g;
      last_insertion_depth_ = g.depth;
      reply.success = true;
      std::snprintf(
        reply.message.data(), reply.message.size(),
        "RCM on: deviation %.3f mm, tip %.1f mm beyond the point", 1000.0 * g.deviation,
        1000.0 * g.depth);
    }
  }
  if (reply.success) {
    // Any accepted switch clears a fault latch and holds until the source sends a target
    // made for the new mode.
    rcm_fault_ = false;
    switch_hold_ = true;
    q_posture_ = q_measured_;
    if (wrist_) {
      shaftPoseAt(q_measured_);
      roll_ref_rotation_ =
        data_.oMf[base_frame_id_].actInv(data_.oMf[roll_ref_frame_id_]).rotation();
    }
    switch_reference_seq_ = reference_seq_.load(std::memory_order_acquire);
  }
  rcm_enabled_flag_.store(rcm_enabled_, std::memory_order_release);
  rcm_reply_seq_.store(seq, std::memory_order_release);
}

void CartesianServoController::publishRcmState(const rclcpp::Time & time)
{
  auto & m = rcm_state_msg_;
  m.header.stamp = time;
  m.enabled = rcm_enabled_;
  m.point.x = rcm_point_.x();
  m.point.y = rcm_point_.y();
  m.point.z = rcm_point_.z();
  const bool have_point = rcm_point_.allFinite();
  m.deviation = have_point ? measured_rcm_.deviation : kNaN;
  m.insertion_depth = have_point ? measured_rcm_.depth : kNaN;
  m.constraint_ok = !(rcm_enabled_ && (qp_failed_ || rcm_fault_));
  rcm_state_rt_->try_publish(m);
}

void CartesianServoController::handleSetRcm(
  const std::shared_ptr<arts_interfaces::srv::SetRcm::Request> request,
  std::shared_ptr<arts_interfaces::srv::SetRcm::Response> response)
{
  const std::lock_guard<std::mutex> lock(rcm_service_mutex_);
  RcmRequest r;
  r.enable = request->enable;
  r.from_current_shaft = request->from_current_shaft;
  r.insertion_depth = request->insertion_depth;
  r.point = Eigen::Vector3d(request->point.point.x, request->point.point.y, request->point.point.z);
  if (r.enable && !r.from_current_shaft) {
    const std::string & frame = request->point.header.frame_id;
    if (!frame.empty() && frame != params_.base_frame) {
      response->success = false;
      response->message = "point must be in '" + params_.base_frame + "' (or empty); transform "
        "it first (the controller does no tf lookups)";
      return;
    }
    if (!r.point.allFinite()) {
      response->success = false;
      response->message = "point contains non-finite values";
      return;
    }
  }
  if (r.enable && r.from_current_shaft && !std::isfinite(r.insertion_depth)) {
    response->success = false;
    response->message = "insertion_depth must be finite (metres; <= 0 reuses the last depth)";
    return;
  }

  rcm_request_buffer_.writeFromNonRT(r);
  const std::uint64_t seq = rcm_request_seq_.fetch_add(1, std::memory_order_acq_rel) + 1;
  // The control cycle answers within one period; a second covers a busy machine. No answer
  // means the controller is not running cycles (inactive).
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (rcm_reply_seq_.load(std::memory_order_acquire) != seq) {
    if (std::chrono::steady_clock::now() > deadline) {
      response->success = false;
      response->message = "no answer from the control cycle within 1 s; is the controller active?";
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  response->success = rcm_reply_.success;
  response->message = rcm_reply_.message.data();
  response->applied_point.x = rcm_reply_.point.x();
  response->applied_point.y = rcm_reply_.point.y();
  response->applied_point.z = rcm_reply_.point.z();
  response->deviation = rcm_reply_.deviation;
  response->insertion_depth = rcm_reply_.insertion_depth;
  RCLCPP_INFO(get_node()->get_logger(), "set_rcm: %s", response->message.c_str());
}

void CartesianServoController::handleSetToolTipOffset(
  const std::shared_ptr<arts_interfaces::srv::SetToolTipOffset::Request> request,
  std::shared_ptr<arts_interfaces::srv::SetToolTipOffset::Response> response)
{
  if (wrist_) {
    response->success = false;
    response->message = "refused: with the instrument wrist the tip is the instrument chain's "
      "(wrist.* geometry), not an offset";
    return;
  }
  if (rcm_enabled_flag_.load(std::memory_order_acquire)) {
    // The offset defines the shaft line; changing it under an active RCM would move the
    // constraint, not the arm.
    response->success = false;
    response->message = "refused while the RCM is on; turn it off first (set_rcm)";
    return;
  }
  const auto & p = request->offset.position;
  const auto & o = request->offset.orientation;

  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(o.x) ||
      !std::isfinite(o.y) || !std::isfinite(o.z) || !std::isfinite(o.w)) {
    response->success = false;
    response->message = "offset contains non-finite values";
    return;
  }

  const Eigen::Vector3d translation(p.x, p.y, p.z);
  if (translation.norm() > params_.max_tool_tip_offset) {
    response->success = false;
    response->message =
      "offset translation " + std::to_string(translation.norm()) + " m exceeds the " +
      std::to_string(params_.max_tool_tip_offset) +
      " m limit. Note the units are METRES; the v1.4 equivalent took millimetres.";
    return;
  }

  Eigen::Quaterniond rotation(o.w, o.x, o.y, o.z);
  if (rotation.norm() < 1e-9) {
    response->success = false;
    response->message = "offset quaternion has zero norm";
    return;
  }
  rotation.normalize();

  const pinocchio::SE3 offset(rotation.toRotationMatrix(), translation);
  tool_tip_offset_buffer_.writeFromNonRT(offset);
  // The cycle applies it at its next start, one period after this frame moves.
  publishTipFrame(offset);

  response->success = true;
  response->message = "offset queued; applied at the start of the next control cycle";
  response->applied_offset.position.x = translation.x();
  response->applied_offset.position.y = translation.y();
  response->applied_offset.position.z = translation.z();
  response->applied_offset.orientation.x = rotation.x();
  response->applied_offset.orientation.y = rotation.y();
  response->applied_offset.orientation.z = rotation.z();
  response->applied_offset.orientation.w = rotation.w();

  RCLCPP_INFO(
    get_node()->get_logger(), "Tool-tip offset set to [%.4f %.4f %.4f] m.", translation.x(),
    translation.y(), translation.z());
}

bool CartesianServoController::readWrist(const rclcpp::Time & time)
{
  const WristState * w = wrist_state_buffer_.readFromRT();
  if (w == nullptr || !w->valid) {
    return false;
  }
  if (static_cast<double>(time.nanoseconds() - w->stamp_ns) * 1e-9 > params_.wrist.state_timeout) {
    return false;
  }
  for (std::size_t k = 0; k < kWristJoints; ++k) {
    q_measured_[joint_q_index_[kNumJoints + k]] = w->q[k];
  }
  return true;
}

void CartesianServoController::publishWristCommand(const rclcpp::Time & time, bool zero)
{
  if (!wrist_command_rt_) {
    return;
  }
  for (std::size_t k = 0; k < kWristJoints; ++k) {
    wrist_command_msg_.position[k] = otg_output_.new_position[kNumJoints + k];
    wrist_command_msg_.velocity[k] = zero ? 0.0 : otg_output_.new_velocity[kNumJoints + k];
  }
  wrist_command_msg_.header.stamp = time;
  wrist_command_rt_->try_publish(wrist_command_msg_);
}

pinocchio::SE3 CartesianServoController::shaftPoseAt(const Eigen::VectorXd & q)
{
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[base_frame_id_].actInv(data_.oMf[shaft_frame_id_]);
}

void CartesianServoController::buildRollRow()
{
  // Collaborator A's roll row (decision 17, 2026-11-03 amendment): the UR's angular velocity
  // about the shaft, taken in a frame fixed to the flange BEFORE the instrument roll (so the
  // instrument joints contribute nothing), is driven to remove the shaft's roll away from its
  // reference: s^T w_UR = -roll_gain * twist. All rotation about the shaft is then the
  // instrument roll's. Expects data_ placements and r_base_t_ at q_command_ (solveTaskQp).
  pinocchio::computeFrameJacobian(
    model_, data_, q_command_, roll_ref_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED,
    jacobian_roll_ref_);
  const Eigen::Matrix3d r_f =
    data_.oMf[base_frame_id_].actInv(data_.oMf[roll_ref_frame_id_]).rotation();
  const Eigen::Vector3d s = r_f.col(2);  // the shaft, base axes
  // s^T (R_base^T w_world) = (R_base s)^T w_world; r_base_t_ is R_base^T.
  const Eigen::Vector3d s_world = r_base_t_.transpose() * s;
  auto & row = task_qp_.rollRow();
  row.a.noalias() = s_world.transpose() * jacobian_roll_ref_.bottomRows<3>();
  Eigen::Quaterniond qe(r_f * roll_ref_rotation_.transpose());
  if (qe.w() < 0.0) {
    qe.coeffs() *= -1.0;
  }
  const double twist = 2.0 * std::atan2(qe.vec().dot(s), qe.w());
  row.b = -params_.wrist.roll_gain * twist;
}

}  // namespace arts_controllers

PLUGINLIB_EXPORT_CLASS(
  arts_controllers::CartesianServoController, controller_interface::ChainableControllerInterface)
