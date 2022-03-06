// Copyright 2021 Red Rocket Computing, LLC
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

#ifndef ROCKET_KAYA_HARDWARE__ROCKET_KAYA_HARDWARE_HPP_
#define ROCKET_KAYA_HARDWARE__ROCKET_KAYA_HARDWARE_HPP_

#include <string>
#include <vector>
#include <memory>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <dynamixel_sdk/port_handler_linux.h>
#include <dynamixel_sdk/protocol2_packet_handler.h>

#include <rocket_kaya_hardware/visibility_control.h>

namespace rocket_kaya_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RocketKayaHardware : public hardware_interface::SystemInterface
{
	public:

		RCLCPP_SHARED_PTR_DEFINITIONS(RocketKayaHardware)

		ROCKET_KAYA_HARDWARE_PUBLIC RocketKayaHardware();
		ROCKET_KAYA_HARDWARE_PUBLIC virtual ~RocketKayaHardware() = default;

		ROCKET_KAYA_HARDWARE_PUBLIC CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
		ROCKET_KAYA_HARDWARE_PUBLIC std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
		ROCKET_KAYA_HARDWARE_PUBLIC std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
		ROCKET_KAYA_HARDWARE_PUBLIC CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
		ROCKET_KAYA_HARDWARE_PUBLIC CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
		ROCKET_KAYA_HARDWARE_PUBLIC hardware_interface::return_type read() override;
		ROCKET_KAYA_HARDWARE_PUBLIC hardware_interface::return_type write() override;
//
//		ROCKET_KAYA_HARDWARE_PUBLIC hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;
//		ROCKET_KAYA_HARDWARE_PUBLIC std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
//		ROCKET_KAYA_HARDWARE_PUBLIC std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
//		ROCKET_KAYA_HARDWARE_PUBLIC hardware_interface::return_type start() override;
//		ROCKET_KAYA_HARDWARE_PUBLIC hardware_interface::return_type stop() override;
//		ROCKET_KAYA_HARDWARE_PUBLIC hardware_interface::return_type read() override;
//		ROCKET_KAYA_HARDWARE_PUBLIC hardware_interface::return_type write() override;

	private:
		struct present_raw
		{
			uint16_t realtime_tick;
			uint8_t moving;
			uint8_t moving_status;
			int16_t present_pwm;
			int16_t present_load;
			int32_t present_velocity;
			int32_t present_position;
		} __attribute__((packed));

		int set_torque(bool on);
		int set_operating_mode(int mode);
		int set_led(bool on);

		std::unique_ptr<dynamixel::PortHandlerLinux> port_handler;
		dynamixel::Protocol2PacketHandler* protocol_handler;

		std::vector<double> velocity_commands;
		std::vector<double> position_states;
		std::vector<double> velocity_states;
		std::vector<double> effort_states;

		std::vector<uint8_t> joint_ids;
		std::vector<double> position_offset;
};

}

#endif
