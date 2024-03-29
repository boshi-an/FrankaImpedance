// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_update_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool CartesianImpedanceUpdateController::init(hardware_interface::RobotHW* robot_hw,
                                              ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_params_ = node_handle.subscribe(
      "compliance_param", 20, &CartesianImpedanceUpdateController::complianceParamCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceUpdateController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  ee_report = node_handle.advertise<geometry_msgs::PoseStamped>("ee_pose", 1000);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceUpdateController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceUpdateController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceUpdateController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceUpdateController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceUpdateController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceUpdateController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceUpdateController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceUpdateController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // dynamic_reconfigure_compliance_param_node_ =
  //     ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  // dynamic_server_compliance_param_ = std::make_unique<
  //     dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(
  //     dynamic_reconfigure_compliance_param_node_);

  // dynamic_server_compliance_param_->setCallback(
  //     boost::bind(&CartesianImpedanceUpdateController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  // cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  // cartesian_damping_target_.setIdentity();
  // cartesian_damping_target_ = cartesian_damping_target_ * 30;
  cartesian_stiffness_.setZero();
  // cartesian_stiffness_target_.setIdentity();
  // cartesian_stiffness_target_ = cartesian_stiffness_target_ * 200;
  // cartesian_stiffness_.setIdentity();
  // cartesian_stiffness_ = cartesian_stiffness_ * 20;
  // cartesian_damping_.coeff() << 0.0;

  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << 130.0 * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << 20.0 * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(130.0) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(20.0) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = 0.2;

  return true;
}

void CartesianImpedanceUpdateController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void CartesianImpedanceUpdateController::update(const ros::Time& /*time*/,
                                                const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  geometry_msgs::PoseStamped msg;
  msg.pose.position.x = position(0);
  msg.pose.position.y = position(1);
  msg.pose.position.z = position(2);
  msg.pose.orientation.x = orientation.x();
  msg.pose.orientation.y = orientation.y();
  msg.pose.orientation.z = orientation.z();
  msg.pose.orientation.w = orientation.w();
  ee_report.publish(msg);
  ros::spinOnce();

  // printf("Current x:%.2lf y:%.2lf z:%.2lf qw:%.2lf qx:%.2lf qy:%.2lf qz: %.2lf\n",
  //       position(0), position(1), position(2), orientation.w(), orientation.x(), orientation.y(), orientation.z());
  // printf("Desired x:%.2lf y:%.2lf z:%.2lf qw:%.2lf qx:%.2lf qy:%.2lf qz: %.2lf\n",
  //       position_d_(0), position_d_(1), position_d_(2), orientation_d_.w(), orientation_d_.x(), orientation_d_.y(), orientation_d_.z());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  // printf("tau_task: %8lf; tau_null: %8lf; coriolis: %8lf\n", tau_task(0), tau_nullspace(0), coriolis(0));
  // printf("stiffness: %lf\n", cartesian_stiffness_(0));
  // printf("stiffness_target: %lf\n", cartesian_stiffness_target_(0));
  // printf("cartesian_stiffness_: %8lf; cartesian_damping_: %8lf\n", cartesian_stiffness_(0), cartesian_damping_(0));

  for (size_t i = 0; i < 7; ++i) {
    tau_task(i, 0) = std::min(std::max(tau_task(i, 0), -10.), 10.);
  }
  
  // for (int i=0; i<7; i++)
  //   printf("%.2lf ", tau_d[i]);
  // printf("\n");
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  

  
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceUpdateController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceUpdateController::complianceParamCallback(
    const std_msgs::Float64MultiArray& msg) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << msg.data[0] * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << msg.data[1] * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(msg.data[0]) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(msg.data[1]) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = msg.data[2];

  printf("Parameters updated to:\n");
  printf("\tTranslational Stiffness: %.4lf\n", msg.data[0]);
  printf("\tRotational Stiffness: %.4lf\n", msg.data[1]);
  printf("\tNullspace Stiffness: %.4lf\n", msg.data[2]);
}

// void CartesianImpedanceUpdateController::complianceParamCallback(
//     franka_example_controllers::compliance_paramConfig& config,
//     uint32_t /*level*/) {
//   cartesian_stiffness_target_.setIdentity();
//   cartesian_stiffness_target_.topLeftCorner(3, 3)
//       << config.translational_stiffness * Eigen::Matrix3d::Identity();
//   cartesian_stiffness_target_.bottomRightCorner(3, 3)
//       << config.rotational_stiffness * Eigen::Matrix3d::Identity();
//   cartesian_damping_target_.setIdentity();
//   // Damping ratio = 1
//   cartesian_damping_target_.topLeftCorner(3, 3)
//       << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
//   cartesian_damping_target_.bottomRightCorner(3, 3)
//       << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
//   nullspace_stiffness_target_ = config.nullspace_stiffness;
// }

void CartesianImpedanceUpdateController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  printf("Calling callback\n");
  printf("XYZ=%.2lf,%.2lf,%.2lf\n", msg->pose.position.x, msg->pose.position.y,
         msg->pose.position.z);
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceUpdateController,
                       controller_interface::ControllerBase)
