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

#include <limits>
#include <vector>

#include <rocket_kaya_hardware/rocket_kaya_hardware.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

constexpr uint16_t operating_mode_addr = 11;
constexpr uint16_t torque_enable_addr = 64;
constexpr uint16_t led_addr = 65;
constexpr uint16_t goal_velocity_addr = 104;
constexpr uint16_t realtime_tick_addr = 120;

constexpr uint8_t velocity_control_mode = 1;
constexpr uint8_t position_control_mode = 3;
constexpr uint8_t extended_position_control_mode = 4;
constexpr uint8_t pwm_control_mode = 16;

constexpr double velocity_scale = 0.104719755 * 0.229;
constexpr double position_scale = (2 * M_PI) / 4096;
constexpr double effort_scale = 1.5 *(0.1 / 100.0);

rocket_kaya_hardware::RocketKayaHardware::RocketKayaHardware() :
	protocol_handler{dynamixel::Protocol2PacketHandler::getInstance()}
{
}

int rocket_kaya_hardware::RocketKayaHardware::set_torque(bool on)
{
	// Build the cmd on the stack
	size_t i = 0;
	uint8_t *cmd = static_cast<uint8_t *>(alloca(joint_ids.size() * 2));
	for (auto id : joint_ids) {
		cmd[i] = id;
		cmd[i + 1] = on;
		i += 2;
	}

	// Send it off
	return protocol_handler->syncWriteTxOnly(port_handler.get(), torque_enable_addr, 1, cmd, joint_ids.size() * 2);
}

int rocket_kaya_hardware::RocketKayaHardware::set_operating_mode(int mode)
{
	// Build the cmd on the stack
	size_t i = 0;
	uint8_t *cmd = static_cast<uint8_t *>(alloca(joint_ids.size() * 2));
	for (auto id : joint_ids) {
		cmd[i] = id;
		cmd[i + 1] = mode & 0xff;;
		i += 2;
	}

	// Send it off
	return protocol_handler->syncWriteTxOnly(port_handler.get(), operating_mode_addr, 1, cmd, joint_ids.size() * 2);
}

int rocket_kaya_hardware::RocketKayaHardware::set_led(bool on)
{
	// Build the cmd on the stack
	size_t i = 0;
	uint8_t *cmd = static_cast<uint8_t *>(alloca(joint_ids.size() * 2));
	for (auto id : joint_ids) {
		cmd[i] = id;
		cmd[i + 1] = on;
		i += 2;
	}

	// Send it off
	return protocol_handler->syncWriteTxOnly(port_handler.get(), led_addr, 1, cmd, joint_ids.size() * 2);
}

CallbackReturn rocket_kaya_hardware::RocketKayaHardware::on_init(const hardware_interface::HardwareInfo &info)
{
	int status;

	// Forward
	if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
		return CallbackReturn::ERROR;

	// Setup the commands and states
	position_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
	velocity_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
	effort_states.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
	velocity_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
	position_offset.resize(info_.joints.size(), 0.0);

	// Get the u2d2 port name
	auto const serial_port_param = info.hardware_parameters.find("serial-port");
	if (serial_port_param == info.hardware_parameters.cend()) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "could find serial port parameter");
		return CallbackReturn::ERROR;
	}

	// Get the u2d2 baud rate
	auto const baud_rate_param = info.hardware_parameters.find("baud-rate");
	if (baud_rate_param == info.hardware_parameters.cend()) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "could find baud-rate parameter");
		return CallbackReturn::ERROR;
	}

	// Open the port
	port_handler = std::make_unique<dynamixel::PortHandlerLinux>(serial_port_param->second.c_str());
	if (!port_handler->openPort()) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "problem opening U2D2 port: %s", serial_port_param->second.c_str());
		return CallbackReturn::ERROR;
	}

	// Set the baud rate
	if (!port_handler->setBaudRate(std::stoi(baud_rate_param->second))) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "problem setting U2D2 %s to baud rate: %s", serial_port_param->second.c_str(), baud_rate_param->second.c_str());
		return CallbackReturn::ERROR;
	}

	// Loop through each joint, configure it
	for (auto const &joint : info.joints) {

		// Extract the joint id
		auto const joint_id_param = joint.parameters.find("id");
		if (joint_id_param == joint.parameters.cend()) {
			RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "joint '%s' missing id parameter", joint.name.c_str());
			return CallbackReturn::ERROR;
		}
		uint8_t joint_id = stoul(joint_id_param->second) & 0xff;

		// Extract the model number
		auto const joint_model_param = joint.parameters.find("model");
		if (joint_model_param == joint.parameters.cend()) {
			RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "joint '%s' missing model parameter", joint.name.c_str());
			return CallbackReturn::ERROR;
		}
		uint16_t joint_model = stoul(joint_model_param->second) & 0xffff;

		// Ping the joint
		uint16_t model;
		uint8_t error = 0;
		status = protocol_handler->ping(port_handler.get(), joint_id, &model, &error);
		if (status != COMM_SUCCESS) {
			RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "joint '%s' could not ping id %hhu: %d", joint.name.c_str(), joint_id, status);
			return CallbackReturn::ERROR;
		}
		if (error != 0) {
			RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "joint '%s' ping failed with id %hhu: %d", joint.name.c_str(), joint_id, status);
			return CallbackReturn::ERROR;
		}

		// Match the model
		if (model != joint_model) {
			RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "joint '%s' model %hu does not match: %hu", joint.name.c_str(), model, joint_model);
			return CallbackReturn::ERROR;
		}

		// Look good, push the id
		joint_ids.push_back(joint_id);
	}

	// Turn the leds off
	status = set_led(false);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "could not turn leds on: %d", status);
		return CallbackReturn::ERROR;
	}

	// Turn the torque off for all motors
	status = set_torque(false);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "could not turn torque off: %d", status);
		return CallbackReturn::ERROR;
	}

	// Set the operating mode to velocity for all motors
	status = set_operating_mode(velocity_control_mode);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "could not set velocity control mode: %d", status);
		return CallbackReturn::ERROR;
	}

	// Write the zero velocity
	if (write() != hardware_interface::return_type::OK) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "could not send the zero velocity command");
		return CallbackReturn::ERROR;
	}

	// Read the current state
	if (read() != hardware_interface::return_type::OK) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "could not read the current state: %d", status);
		return CallbackReturn::ERROR;
	}

	// Update the offset
	for (std::vector<double>::size_type i = 0; i < joint_ids.size(); ++i)
		position_offset[i] = position_states[i];

	RCLCPP_INFO(rclcpp::get_logger("RocketKayaHardware"), "hardware interface configured");

	return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> rocket_kaya_hardware::RocketKayaHardware::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> state_interfaces;
	for (uint i = 0; i < info_.joints.size(); i++) {
		state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states[i]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states[i]));
	}
	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> rocket_kaya_hardware::RocketKayaHardware::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> command_interfaces;
	for (uint i = 0; i < info_.joints.size(); i++)
		command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands[i]));
	return command_interfaces;
}

CallbackReturn rocket_kaya_hardware::RocketKayaHardware::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
	// Enable torque
	int status = set_torque(true);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "could not turn torque on: %d", status);
		return CallbackReturn::ERROR;
	}

	// Turn the leds on
	status = set_led(true);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "could not turn leds on: %d", status);
		return CallbackReturn::ERROR;
	}

	RCLCPP_INFO(rclcpp::get_logger("RocketKayaHardware"), "joints powered");

	return CallbackReturn::SUCCESS;
}

CallbackReturn rocket_kaya_hardware::RocketKayaHardware::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
	// Enable torque
	int status = set_torque(false);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "could not turn torque off: %d", status);
		return CallbackReturn::ERROR;
	}

	// Turn the leds on
	status = set_led(false);
	if (status != COMM_SUCCESS) {
		RCLCPP_FATAL(rclcpp::get_logger("RocketKayaHardware"), "could not turn leds off: %d", status);
		return CallbackReturn::ERROR;
	}

	RCLCPP_INFO(rclcpp::get_logger("RocketKayaHardware"), "joints de-powered");

	return CallbackReturn::SUCCESS;
}

hardware_interface::return_type rocket_kaya_hardware::RocketKayaHardware::read()
{
	int status;
	uint8_t error;
	struct present_raw *present = static_cast<struct present_raw *>(alloca(joint_ids.size() * sizeof(struct present_raw)));

	// Send the sync read request
	status = protocol_handler->syncReadTx(port_handler.get(), realtime_tick_addr, sizeof(struct present_raw), joint_ids.data(), joint_ids.size());
	if (status != COMM_SUCCESS) {
		RCLCPP_ERROR(rclcpp::get_logger("RocketKayaHardware"), "could not send the sync read: %d", status);
		return hardware_interface::return_type::ERROR;
	}

	// Try to read all the responses
	size_t i = 0;
	for (auto id : joint_ids) {

		// Read the response
		error = 0;
		status = protocol_handler->readRx(port_handler.get(), id, sizeof(struct present_raw), reinterpret_cast<uint8_t *>(&present[i]), &error);
		if (status != COMM_SUCCESS) {
			RCLCPP_ERROR(rclcpp::get_logger("RocketKayaHardware"), "could not read response for %hhu: %d", id, status);
			return hardware_interface::return_type::ERROR;
		}

		// Process the response
		position_states[i] = (present[i].present_position - position_offset[i]) * position_scale;
		velocity_states[i] = present[i].present_velocity * velocity_scale;
		effort_states[i] = present[i].present_load * effort_scale;

		// Move to the next buffer
		++i;
	}

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type rocket_kaya_hardware::RocketKayaHardware::write()
{
	// Build the cmd on the stack
	size_t i = 0;
	uint8_t *cmd = static_cast<uint8_t *>(alloca(joint_ids.size() * 5));
	for (auto id : joint_ids) {
		cmd[i * 5] = id;
		int32_t raw_velocity = std::lround(velocity_commands[i] / velocity_scale);
		memcpy(&cmd[(i * 5) + 1], &raw_velocity, sizeof(raw_velocity));
		++i;
	}

	// Send it off
	return protocol_handler->syncWriteTxOnly(port_handler.get(), goal_velocity_addr, sizeof(int32_t), cmd, joint_ids.size() * 5) == COMM_SUCCESS ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rocket_kaya_hardware::RocketKayaHardware, hardware_interface::SystemInterface)
