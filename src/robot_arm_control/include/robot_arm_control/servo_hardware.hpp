#ifndef ROBOT_ARM_CONTROL_SERVO_HARDWARE_HPP_
#define ROBOT_ARM_CONTROL_SERVO_HARDWARE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_arm_control
{
class ServoHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
};

}  // namespace robot_arm_control

#endif  // ROBOT_ARM_CONTROL_SERVO_HARDWARE_HPP_ 