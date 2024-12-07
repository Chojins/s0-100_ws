#include "robot_arm_control/servo_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace robot_arm_control
{
hardware_interface::CallbackReturn ServoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize storage for command and state interfaces
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    RCLCPP_INFO(rclcpp::get_logger("ServoHardware"), 
      "Found joint '%s' in URDF", joint.name.c_str());
    
    // Verify that the joint has both command and state interfaces
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ServoHardware"),
        "Joint '%s' has incorrect command interfaces", joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ServoHardware"),
        "Joint '%s' has incorrect state interfaces", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ServoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  RCLCPP_INFO(
    rclcpp::get_logger("ServoHardware"),
    "Starting to export %zu state interfaces", info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("ServoHardware"),
      "Creating state interface for joint '%s'", info_.joints[i].name.c_str());

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    
    RCLCPP_INFO(
      rclcpp::get_logger("ServoHardware"),
      "Successfully added state interface for joint '%s'", info_.joints[i].name.c_str());
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ServoHardware"),
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ServoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
    
    RCLCPP_INFO(rclcpp::get_logger("ServoHardware"),
      "Exported command interface for joint '%s'", info_.joints[i].name.c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ServoHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoHardware"), "Activating hardware interface");
  
  // Initialize current position as 0
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_commands_[i] = 0.0;
    RCLCPP_INFO(
      rclcpp::get_logger("ServoHardware"),
      "Initial position for joint %zu: %f", i, hw_positions_[i]);
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoHardware"), "Deactivating hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ServoHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    RCLCPP_DEBUG(rclcpp::get_logger("ServoHardware"), 
      "Got position state %.5f for joint %zu", hw_positions_[i], i);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ServoHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    RCLCPP_DEBUG(rclcpp::get_logger("ServoHardware"), 
      "Writing command %.5f for joint %zu", hw_commands_[i], i);
    
    // For testing, copy commands to positions
    hw_positions_[i] = hw_commands_[i];
  }
  return hardware_interface::return_type::OK;
}

}  // namespace robot_arm_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_arm_control::ServoHardware, hardware_interface::SystemInterface)