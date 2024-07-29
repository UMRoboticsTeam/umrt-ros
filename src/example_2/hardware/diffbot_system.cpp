// Copyright 2021 ros2_control Development Team
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

#include "ros2_control_demo_example_2/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <algorithm>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define PWM_NEUTRAL 1.5
#define PWM_AMPLITUDE 0.5

#define PWM_FREQUENCY 50 // Hz, 20 ms period. -njreichert

namespace ros2_control_demo_example_2
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  wheel_radius_ = hardware_interface::stod(info_.hardware_parameters.at("param_wheel_radius"));

  RCLCPP_INFO(
    rclcpp::get_logger("DiffBotSystemHardware"),
    "Wheel Radius: %f",
    wheel_radius_
  );

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    WheelInfo current_wheel = {
      .name = joint.name,
      .command = std::numeric_limits<double>::quiet_NaN(),
      .velocity = std::numeric_limits<double>::quiet_NaN(),
      .position = std::numeric_limits<double>::quiet_NaN(),
    };

    wheels.push_back(current_wheel);
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Got new wheel with name: \"%s\"",
      wheels.back().name.c_str()
    );
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &wheels[i].position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheels[i].velocity));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheels[i].command));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values
  for (auto &wheel : this->wheels)
  {
    if (std::isnan(wheel.position))
    {
      wheel.position = 0;
      wheel.velocity = 0;
      wheel.command = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  for (std::size_t i = 0; i < wheels.size(); i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    wheels[i].position = wheels[i].position + period.seconds() * wheels[i].velocity;

    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'!", wheels[i].position,
      wheels[i].velocity, info_.joints[i].name.c_str());
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_2::DiffBotSystemHardware::set_pwm_wheel_speed(
  int channel, double angular_speed)
{
  // Between -1 and 1 m/s by definition in diffbot_controllers.yaml.
  double linear_wheel_speed = std::clamp(angular_speed * this->wheel_radius_, -1.0, 1.0);

  // HACK ALERT!!!!!!!!!!!!!!!
  // This assumes the following:
  // 1. Joints are passed in the following order: LF, RF, LB, RB
  // 2. Servo outputs are wired in the EXACT SAME ORDER AS IN ASSUMPTION 1, STARTING AT PORT 0!
  //
  // Since a linear wheel speed that is positive results in wheels spinning in
  // the same direction (i.e.: Reverse on the other side), we need to reverse 
  // wheels on the "other side".
  // 
  // Obviously this is horrible but is the best we have right now. -njreichert 2024-07-22
  if (channel % 2 == 1)
  {
    linear_wheel_speed = -1 * linear_wheel_speed;
  }

  // Typical PWM Servo control assumes:
  // - 1.0 == Full reverse
  // - 1.5 == Neutral / stopped
  // - 2.0 == Full forward
  double motor_pulse_width = std::clamp(PWM_NEUTRAL + (linear_wheel_speed * PWM_AMPLITUDE), 1.0, 2.0);

  RCLCPP_INFO(
    rclcpp::get_logger("DiffBotSystemHardware"),
    "Setting PWM channel %d to %f!", 
    channel, motor_pulse_width);

  try
  {
    this->pwm_device_.set_pwm_freq(PWM_FREQUENCY);
    this->pwm_device_.set_pwm_ms(channel, motor_pulse_width);
  } 
  catch (std::exception& e) // TODO: Make this look less hacky. -njreichert
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Can't communicate with PWM Controller! (%s)",
      e.what()
    );

    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_2::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool pwm_driver_unreachable = false;

  for (auto i = 0u; i < wheels.size(); i++)
  {

    wheels[i].velocity = wheels[i].command;

    if (this->set_pwm_wheel_speed(i, wheels[i].command) != hardware_interface::return_type::OK)
    {
      pwm_driver_unreachable = true;
    }
  }

  if (pwm_driver_unreachable)
  {
    return hardware_interface::return_type::ERROR;
  }
  else
  {
    return hardware_interface::return_type::OK;
  }
}

//
// Attempt to reset the PWM Driver.
// 
// If we cannot reset it, we have no option but to kill the program. -njreichert
//
hardware_interface::CallbackReturn ros2_control_demo_example_2::DiffBotSystemHardware::on_error(
  [[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
{
  hardware_interface::CallbackReturn retval = hardware_interface::CallbackReturn::SUCCESS;

  try
  {
    // TODO: Can we re-open the i2c device, or is it not worth it? -njreichert
    this->pwm_device_.set_pwm_freq(PWM_FREQUENCY);
    for (size_t channel_num = 0; channel_num < 4; channel_num++)
    {
      this->pwm_device_.set_pwm_ms(channel_num, PWM_NEUTRAL);
    }
  }
  catch (std::exception& e) // TODO: Make this look less hacky. -njreichert
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Can't communicate with PWM Controller! (%s)",
      e.what()
    );

    retval = hardware_interface::CallbackReturn::ERROR;
  }

  return retval;
}

}  // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_2::DiffBotSystemHardware, hardware_interface::SystemInterface)
