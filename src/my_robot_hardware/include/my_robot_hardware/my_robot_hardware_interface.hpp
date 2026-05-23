#ifndef MY_ROBOT_HARDWARE_INTERFACE_HPP //prevents file frombeing included multiple times
#define MY_ROBOT_HARDWARE_INTERFACE_HPP

#include <string> //allows strings
#include <vector> //aallows vector arryas of position ,command etc

// ROS2 control headers
#include "hardware_interface/system_interface.hpp" //fromm ros2_control package
#include "hardware_interface/types/hardware_interface_return_values.hpp" //OK AND ERROR in read() and write() commands
#include "rclcpp/rclcpp.hpp" //gives nodes etc
#include "rclcpp_lifecycle/state.hpp" //gives lifecycle nodes

// Linux serial headers for opening serial baudrate set and read/write
#include <termios.h>
#include <unistd.h>

namespace my_robot_hardware //like a folder for code prevents name collisons
{
     //class name made into plugin      //inheriting from system interface      
class MyRobotHardwareInterface : public hardware_interface::SystemInterface
{
public: //ros2 is allowed to call these fucntions

  //return type sucess or fail       //called once method
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;  //input of the method

  // Called when ROS2 control activates — opens serial port
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Called when ROS2 control deactivates — closes serial port
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Called every control cycle — reads joint states FROM ESP32
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Called every control cycle — writes commands TO ESP32
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Tells ros2_control what state data we provide (position)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Tells ros2_control what command data we accept (position)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private: //ros neever calls there directly
  // Serial port helpers
  bool openSerial(const std::string & port, int baud); //inputs devtty,baudrate
  void closeSerial();
  bool writeSerial(const std::string & msg); //inputs string message to serial
  std::string readLine(); //reads lines until \n

  // Serial port file descriptor
  int serial_fd_; // if this is <0 mean no serial open linux thing

  // Serial port name e.g. "/dev/ttyUSB0"
  std::string serial_port_;

  // Storage for joint positions — ros2_control reads/writes these
  // Index 0=base_yaw_joint, 1=arm_joint1, 2=arm_joint2
  std::vector<double> hw_positions_;      // current positions (from ESP32)   //bridge b/w controller and HWI
  std::vector<double> hw_commands_;       // commanded positions (to ESP32)

  // Logger
  rclcpp::Logger logger_ = rclcpp::get_logger("MyRobotHardwareInterface");
};

}  // namespace my_robot_hardware

#endif  // MY_ROBOT_HARDWARE_INTERFACE_HPP