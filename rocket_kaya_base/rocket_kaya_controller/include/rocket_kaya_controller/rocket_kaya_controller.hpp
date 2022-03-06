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

#ifndef ROCKET_KAYA_CONTROLLER__ROCKET_KAYA_CONTROLLER_HPP_
#define ROCKET_KAYA_CONTROLLER__ROCKET_KAYA_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <queue>

#include <eigen3/Eigen/Dense>

#include <rocket_kaya_controller/visibility_control.h>
#include <controller_interface/controller_interface.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace rocket_kaya_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RocketKayaController : public controller_interface::ControllerInterface
{
	public:
		ROCKET_KAYA_CONTROLLER_PUBLIC RocketKayaController();

		ROCKET_KAYA_CONTROLLER_PUBLIC controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

		ROCKET_KAYA_CONTROLLER_PUBLIC controller_interface::InterfaceConfiguration command_interface_configuration() const override;
		ROCKET_KAYA_CONTROLLER_PUBLIC controller_interface::InterfaceConfiguration state_interface_configuration() const override;

		ROCKET_KAYA_CONTROLLER_PUBLIC CallbackReturn on_init() override;
		ROCKET_KAYA_CONTROLLER_PUBLIC CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
		ROCKET_KAYA_CONTROLLER_PUBLIC CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
		ROCKET_KAYA_CONTROLLER_PUBLIC CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
		ROCKET_KAYA_CONTROLLER_PUBLIC CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
		ROCKET_KAYA_CONTROLLER_PUBLIC CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
		ROCKET_KAYA_CONTROLLER_PUBLIC CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;


	protected:
		struct WheelHandle
		{
			std::reference_wrapper<const hardware_interface::LoanedStateInterface> x;
			std::reference_wrapper<const hardware_interface::LoanedStateInterface> v;
			std::reference_wrapper<hardware_interface::LoanedCommandInterface> v_desired;
		};

		void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
		double limit(double v, double v0, double vlimit, double alimit, double dt);

		bool reset();
		void halt();

		std::vector<std::string> joint_names;

		double r = 0.0;
		double d = 0.0;
		std::vector<double> alpha;
	    std::string base_frame_id = "base_link";
	    std::string odom_frame_id = "odom";
	    std::vector<double> pose_covariance_diagonal;
	    std::vector<double> twist_covariance_diagonal;
	    std::chrono::milliseconds cmd_vel_timeout;
	    double linear_vel_limit;
	    double linear_accel_limit;
	    double angular_vel_limit;
	    double angular_accel_limit;

	    Eigen::Matrix3d J;
		Eigen::Matrix3d G;
		Eigen::Vector3d last_u;
		Eigen::Vector3d last_q;

		rclcpp::Time last_timestamp;

		std::vector<WheelHandle> wheel_handles;
		bool is_subcriber_active = false;
		bool is_halted = true;

		realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>> received_cmd_vel;
		geometry_msgs::msg::TwistStamped last_cmd;

		std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher = nullptr;
		std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher = nullptr;
		std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> realtime_limited_velocity_publisher = nullptr;

		rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscriber = nullptr;
};

}

#endif
