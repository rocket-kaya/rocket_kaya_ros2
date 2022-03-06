// Copyright 2021 Red Rocket Computing, LLC.
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

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include <rocket_kaya_controller/rocket_kaya_controller.hpp>
#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <tf2/LinearMath/Quaternion.h>

constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
constexpr size_t COVARIANCE_DIMENSIONS = 6;

rocket_kaya_controller::RocketKayaController::RocketKayaController() :
		controller_interface::ControllerInterface()
{
}

rocket_kaya_controller::CallbackReturn rocket_kaya_controller::RocketKayaController::on_init()
{
	try {
		auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
		auto_declare<double>("kinematics.r", std::numeric_limits<double>::quiet_NaN());
		auto_declare<double>("kinematics.d", std::numeric_limits<double>::quiet_NaN());
		auto_declare<std::vector<double>>("kinematics.alpha", std::vector<double>());
		auto_declare<std::string>("base_frame_id", "base_link");
		auto_declare<std::string>("odom_frame_id", "odom");
		auto_declare<double>("cmd_vel_timeout", std::numeric_limits<double>::quiet_NaN());
		auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
		auto_declare<std::vector<double>>("twist_covariance_diagonal",  std::vector<double>());
		auto_declare<double>("linear_vel_limit", std::numeric_limits<double>::quiet_NaN());
		auto_declare<double>("linear_accel_limit", std::numeric_limits<double>::quiet_NaN());
		auto_declare<double>("angular_vel_limit", std::numeric_limits<double>::quiet_NaN());
		auto_declare<double>("angular_accel_limit", std::numeric_limits<double>::quiet_NaN());

		return CallbackReturn::SUCCESS;

	} catch (const std::exception &e) {
		RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
		return CallbackReturn::ERROR;
	}
}

rocket_kaya_controller::CallbackReturn rocket_kaya_controller::RocketKayaController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
{
	// Get the base and odom frame ids
	base_frame_id = get_node()->get_parameter("base_frame_id").as_string();
	odom_frame_id = get_node()->get_parameter("odom_frame_id").as_string();

	// Get the joint names
	joint_names = get_node()->get_parameter("joints").as_string_array();
	if (joint_names.empty()) {
		RCLCPP_ERROR(get_node()->get_logger(), "joint_name parameter was empty");
		return CallbackReturn::ERROR;
	}

	// Get the wheel base distance
	d = get_node()->get_parameter("kinematics.d").as_double();
	if (std::isnan(d)) {
		RCLCPP_ERROR(get_node()->get_logger(), "kinematics.d parameter was empty");
		return CallbackReturn::ERROR;
	}

	// Get the wheel radius
	r = get_node()->get_parameter("kinematics.r").as_double();
	if (std::isnan(r)) {
		RCLCPP_ERROR(get_node()->get_logger(), "kinematics.d parameter was empty");
		return CallbackReturn::ERROR;
	}

	// Get the wheel positions around the base
	alpha = get_node()->get_parameter("kinematics.alpha").as_double_array();
	if (alpha.empty()) {
		RCLCPP_ERROR(get_node()->get_logger(), "kinematics.alpha parameter was empty");
		return CallbackReturn::ERROR;
	}

	// Get the pose covariance
	pose_covariance_diagonal = get_node()->get_parameter("pose_covariance_diagonal").as_double_array();
	if (pose_covariance_diagonal.size() != COVARIANCE_DIMENSIONS) {
		RCLCPP_ERROR(get_node()->get_logger(), "bad pose covariance, size if not %zu", COVARIANCE_DIMENSIONS);
		return CallbackReturn::ERROR;
	}

	// Get the twist covariance
	twist_covariance_diagonal = get_node()->get_parameter("twist_covariance_diagonal").as_double_array();
	if (twist_covariance_diagonal.size() != COVARIANCE_DIMENSIONS) {
		RCLCPP_ERROR(get_node()->get_logger(), "bad twist covariance, size if not %zu", COVARIANCE_DIMENSIONS);
		return CallbackReturn::ERROR;
	}

	// Get the cmd timeout
	double timeout = get_node()->get_parameter("cmd_vel_timeout").as_double();
	if (timeout == std::numeric_limits<double>::quiet_NaN()) {
		RCLCPP_ERROR(get_node()->get_logger(), "cmd velocity timeout missing");
		return CallbackReturn::ERROR;
	}
	cmd_vel_timeout = std::chrono::milliseconds(std::lround(timeout * 1000));

	// Get the velocity and accel limits
	linear_vel_limit = get_node()->get_parameter("linear_vel_limit").as_double();
	linear_accel_limit = get_node()->get_parameter("linear_accel_limit").as_double();
	angular_vel_limit = get_node()->get_parameter("angular_vel_limit").as_double();
	angular_accel_limit = get_node()->get_parameter("angular_accel_limit").as_double();

	// Try to reset the controller
	if (!reset())
		return CallbackReturn::ERROR;

	// Setup the velocity limited publishers
	auto limited_velocity_publisher = node_->create_publisher<geometry_msgs::msg::TwistStamped>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
	realtime_limited_velocity_publisher = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(limited_velocity_publisher);

	// Setup the odometry publisher
	auto odometry_publisher = node_->create_publisher<nav_msgs::msg::Odometry>(DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
	realtime_odometry_publisher = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher);

	// Setup the odometry transform publisher
	auto odometry_transform_publisher = node_->create_publisher<tf2_msgs::msg::TFMessage>(DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
	realtime_odometry_transform_publisher = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odometry_transform_publisher);

	// Setup the cmd velocity subscribers
	cmd_vel_subscriber = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(DEFAULT_COMMAND_TOPIC, 10, std::bind(&RocketKayaController::cmd_vel_callback, this, std::placeholders::_1));

	// Initialize the twist msg buffer
	received_cmd_vel.set(std::make_shared<geometry_msgs::msg::TwistStamped>());

	// Initialize the velocity projection matrices
	J << sin(alpha[0]), -cos(alpha[0]), -d, sin(alpha[1]), -cos(alpha[1]), -d, sin(alpha[2]), -cos(alpha[2]), -d;
	G = J.inverse();

	// Initialize odometry message
	realtime_odometry_publisher->msg_.header.frame_id = odom_frame_id;
	realtime_odometry_publisher->msg_.child_frame_id = base_frame_id;
	realtime_odometry_publisher->msg_.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);
	for (size_t index = 0; index < COVARIANCE_DIMENSIONS; ++index) {
		// Diagonal offsets are 0, 7, 14, 21, 28, 35
		realtime_odometry_publisher->msg_.pose.covariance[COVARIANCE_DIMENSIONS * index + index] = pose_covariance_diagonal[index];
		realtime_odometry_publisher->msg_.twist.covariance[COVARIANCE_DIMENSIONS * index + index] = twist_covariance_diagonal[index];
	}

	// keeping track of odom and base_link transforms only
	realtime_odometry_transform_publisher->msg_.transforms.resize(1);
	realtime_odometry_transform_publisher->msg_.transforms.front().header.frame_id = odom_frame_id;
	realtime_odometry_transform_publisher->msg_.transforms.front().child_frame_id = base_frame_id;

	// Every is ready to go
	RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
	return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration rocket_kaya_controller::RocketKayaController::command_interface_configuration() const
{
	controller_interface::InterfaceConfiguration command_interfaces_config;
	command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

	command_interfaces_config.names.reserve(joint_names.size());
	for (const auto &joint : joint_names)
		command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);

	return command_interfaces_config;
}

controller_interface::InterfaceConfiguration rocket_kaya_controller::RocketKayaController::state_interface_configuration() const
{
	controller_interface::InterfaceConfiguration state_interfaces_config;
	state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

	state_interfaces_config.names.reserve(joint_names.size());
	for (const auto &joint : joint_names) {
		state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
		state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
	}

	return state_interfaces_config;
}

rocket_kaya_controller::CallbackReturn rocket_kaya_controller::RocketKayaController::on_activate(const rclcpp_lifecycle::State&/*previous_state*/)
{
	// Create a wheel handle for each joint
	wheel_handles.reserve(joint_names.size());
	for (const auto &wheel_name : joint_names) {

		// Extract the wheel position handle
		const auto x_state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_name](const auto &interface) {return interface.get_name() == wheel_name && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;});
		if (x_state_handle == state_interfaces_.cend()) {
			RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain wheel position state handle for %s", wheel_name.c_str());
			return CallbackReturn::ERROR;
		}

		// Extract the wheel velocity handle
		const auto v_state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_name](const auto &interface) {return interface.get_name() == wheel_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;});
		if (v_state_handle == state_interfaces_.cend()) {
			RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain wheel velocity state handle for %s", wheel_name.c_str());
			return CallbackReturn::ERROR;
		}

		// Extract wheel command velocity handle
		const auto v_desired = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_name](const auto &interface) {return interface.get_name() == wheel_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;});
		if (v_desired == command_interfaces_.end()) {
			RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain wheel velocity command handle for %s", wheel_name.c_str());
			return CallbackReturn::ERROR;
		}

		// Save the handles
		wheel_handles.emplace_back(WheelHandle { std::ref(*x_state_handle), std::ref(*v_state_handle), std::ref(*v_desired) });
	}

	// It worked
	is_halted = false;
	is_subcriber_active = true;

	RCLCPP_DEBUG(node_->get_logger(), "Subscribers and publishers are now active.");
	return CallbackReturn::SUCCESS;
}

rocket_kaya_controller::CallbackReturn rocket_kaya_controller::RocketKayaController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
{
	is_subcriber_active = false;
	return CallbackReturn::SUCCESS;
}

rocket_kaya_controller::CallbackReturn rocket_kaya_controller::RocketKayaController::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
{
	if (!reset())
		return CallbackReturn::ERROR;

//	received_velocity_msg_ptr_.set(std::make_shared<Twist>());
	return CallbackReturn::SUCCESS;
}

rocket_kaya_controller::CallbackReturn rocket_kaya_controller::RocketKayaController::on_error([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
{
	if (!reset())
		return CallbackReturn::ERROR;
	return CallbackReturn::SUCCESS;
}

rocket_kaya_controller::CallbackReturn rocket_kaya_controller::RocketKayaController::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
{
	return CallbackReturn::SUCCESS;
}

double rocket_kaya_controller::RocketKayaController::limit(double v, double v0, double vlimit, double alimit, double dt)
{
	if (!std::isnan(alimit))
		v = v0 + std::clamp(v - v0, -alimit * dt, alimit * dt);

	if (!std::isnan(vlimit))
		v = std::clamp(v, -vlimit, vlimit);

	return v;
}

controller_interface::return_type rocket_kaya_controller::RocketKayaController::update(const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period)
{
	auto timestamp = time;

	// Are we in the inactive state? If so, halt is needed
	if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
		if (!is_halted) {
			halt();
			is_halted = true;
		}
		return controller_interface::return_type::OK;
	}

	// Extract the current velocity command
	std::shared_ptr<geometry_msgs::msg::TwistStamped> current_cmd;
	received_cmd_vel.get(current_cmd);
	if (current_cmd == nullptr) {
		RCLCPP_WARN(get_node()->get_logger(), "Velocity message received was a nullptr");
		return controller_interface::return_type::ERROR;
	}

	// Calculate delta time and check for timeout, brake if a timeout has been detected
	const auto dt = timestamp - current_cmd->header.stamp;
	if (dt > cmd_vel_timeout) {
		current_cmd->twist.linear.x = 0.0;
		current_cmd->twist.linear.y = 0.0;
		current_cmd->twist.angular.z = 0.0;
	}

	// Calculate the odometry, NEED workup in matlab doc
	Eigen::Vector3d u(wheel_handles[0].x.get().get_value(), wheel_handles[1].x.get().get_value(), wheel_handles[2].x.get().get_value());
	Eigen::Vector3d delta_u = u - last_u;
	Eigen::Vector3d vb = G * delta_u;
	double vbx = vb(0, 0);
	double vby = vb(1, 0);
	double vbw = vb(2, 0);
	Eigen::Vector3d delta_qb;
	if (vbw != 0) {
		double sbw = sin(vbw);
		double cbw = cos(vbw);
		delta_qb << (vbx * sbw + vby * (cbw - 1)) / vbw, (vby * sbw + vbx * (1 - cbw)) / vbw, vbw;
	} else
		delta_qb << vbx, vby, 0;

	Eigen::Matrix3d tsb;
	double ctheta = cos(last_q(2, 0));
	double stheta = sin(last_q(2, 0));
	tsb << ctheta, -stheta, 0, stheta, ctheta, 0, 0, 0, 1;
	Eigen::Vector3d delta_q = tsb * delta_qb;
	Eigen::Vector3d q = last_q + delta_q;

	// Get the orientation as quaternion
	tf2::Quaternion orientation;
	orientation.setRPY(0.0, 0.0, q(2, 0));

	// Publish the odometery if not busy
	if (realtime_odometry_publisher->trylock()) {
		realtime_odometry_publisher->msg_.header.stamp = timestamp;
		realtime_odometry_publisher->msg_.pose.pose.position.x = q(0, 0);
		realtime_odometry_publisher->msg_.pose.pose.position.y = q(1, 0);
		realtime_odometry_publisher->msg_.pose.pose.orientation.x = orientation.x();
		realtime_odometry_publisher->msg_.pose.pose.orientation.y = orientation.y();
		realtime_odometry_publisher->msg_.pose.pose.orientation.z = orientation.z();
		realtime_odometry_publisher->msg_.pose.pose.orientation.w = orientation.w();
		realtime_odometry_publisher->msg_.twist.twist.linear.x = delta_q(0, 0);
		realtime_odometry_publisher->msg_.twist.twist.linear.y = delta_q(1, 0);
		realtime_odometry_publisher->msg_.twist.twist.angular.z = delta_q(2, 0);
		realtime_odometry_publisher->unlockAndPublish();
	}

	// Publish the odometry transform
	if (realtime_odometry_transform_publisher->trylock()) {
		realtime_odometry_transform_publisher->msg_.transforms.front().header.stamp = timestamp;
		realtime_odometry_transform_publisher->msg_.transforms.front().transform.translation.x = q(0, 0);
		realtime_odometry_transform_publisher->msg_.transforms.front().transform.translation.y = q(1, 0);
		realtime_odometry_transform_publisher->msg_.transforms.front().transform.rotation.x = orientation.x();
		realtime_odometry_transform_publisher->msg_.transforms.front().transform.rotation.y = orientation.y();
		realtime_odometry_transform_publisher->msg_.transforms.front().transform.rotation.z = orientation.z();
		realtime_odometry_transform_publisher->msg_.transforms.front().transform.rotation.w = orientation.w();
		realtime_odometry_transform_publisher->unlockAndPublish();
	}

	// Extract and limit the velocity commands
	double linear_x_cmd = limit(current_cmd->twist.linear.x, last_cmd.twist.linear.x, linear_vel_limit, linear_accel_limit, dt.seconds());
	double linear_y_cmd = limit(current_cmd->twist.linear.y, last_cmd.twist.linear.y, linear_vel_limit, linear_accel_limit, dt.seconds());
	double angular_z_cmd = limit(current_cmd->twist.angular.z, last_cmd.twist.angular.z, angular_vel_limit, angular_accel_limit, dt.seconds());

	// Try to publish the limited velocity command if not busy
	if (realtime_limited_velocity_publisher->trylock()) {
		realtime_limited_velocity_publisher->msg_.header.stamp = timestamp;
		realtime_limited_velocity_publisher->msg_.twist.linear.x = linear_x_cmd;
		realtime_limited_velocity_publisher->msg_.twist.linear.y = linear_y_cmd;
		realtime_limited_velocity_publisher->msg_.twist.angular.z = angular_z_cmd;
		realtime_limited_velocity_publisher->unlockAndPublish();
	}

	// Setup the commanded velocity
	Eigen::Vector3d qdot;
	qdot << linear_x_cmd, linear_y_cmd, angular_z_cmd;

	// Calculate the wheel velocities
	Eigen::Vector3d udot;
	udot = (J * qdot).array() * (1 / r);

	// Set wheels velocities:
	for (std::vector<WheelHandle>::size_type i = 0; i < wheel_handles.size(); ++i)
		wheel_handles[i].v_desired.get().set_value(udot[i]);

	// Update
	last_q = q;
	last_u = u;
	last_cmd = *current_cmd;
	last_timestamp = timestamp;

	return controller_interface::return_type::OK;
}

bool rocket_kaya_controller::RocketKayaController::reset()
{
	is_halted = false;
	last_q = Eigen::Vector3d(0.0, 0.0, 0.0);
	last_u = Eigen::Vector3d(0.0, 0.0, 0.0);
	return true;
}

void rocket_kaya_controller::RocketKayaController::halt()
{
	for (const auto &wheel : wheel_handles)
		wheel.v_desired.get().set_value(0.0);
}

void rocket_kaya_controller::RocketKayaController::cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
	// Drop is not active
	if (!is_subcriber_active) {
		RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
		return;
	}

	// Patch up the timestamp
	if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
		RCLCPP_WARN_ONCE(get_node()->get_logger(), "Received TwistStamped with zero timestamp, setting it to current time, this message will only be shown once");
		msg->header.stamp = get_node()->get_clock()->now();
	}

	// Save the message
	received_cmd_vel.set(std::move(msg));
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rocket_kaya_controller::RocketKayaController, controller_interface::ControllerInterface)
