#include "my_robot_hardware/my_robot_hardware_interface.hpp" //implemnts the hpp
#include <fcntl.h> //allows open() serial
#include <termios.h> // serial for serial port config
#include <unistd.h> //allows write() read() and close()
#include <sstream> //used for string formatting
#include <pluginlib/class_list_macros.hpp> //needed for exporting class using pluginlib
#include "hardware_interface/types/hardware_interface_type_values.hpp" //error fix for hw_if_position


namespace my_robot_hardware //same as hpp
{

// ─────────────────────────────────────────────
// on_init — called once when plugin loads
// Reads parameters from your .ros2_control.xacro
// ─────────────────────────────────────────────
hardware_interface::CallbackReturn MyRobotHardwareInterface::on_init(  //return type sucess or error
  const hardware_interface::HardwareInfo & info) //on init belongs to class we created in hpp
{
  // Always call the parent first — it populates info_ for us
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR; //if error stops with messaage error
  }

  // Read serial port param from xacro — we'll define this in Part D
  // Falls back to /dev/ttyUSB0 if not set
  if (info_.hardware_parameters.count("serial_port")) { //if inside param inside info using count if string exist pass true
    serial_port_ = info_.hardware_parameters.at("serial_port"); //stores the param
  } else {
    serial_port_ = "/dev/ttyUSB0"; //if no entry default to this usb
  }

  RCLCPP_INFO(logger_, "Serial port set to: %s", serial_port_.c_str()); //logs serial port  and convert string to c 
  // Resize our storage vectors to match number of joints (3)
  hw_positions_.resize(info_.joints.size(), 0.0); //reszie positon vector to no. of joints
  hw_commands_.resize(info_.joints.size(), 0.0); //same 

  RCLCPP_INFO(logger_, "on_init successful — %zu joints found", info_.joints.size()); // prints log of succesful inti and joints number
  return hardware_interface::CallbackReturn::SUCCESS; //returns with sucess
}//inside on init

// ─────────────────────────────────────────────
// on_activate — called when controller starts
// Opens the serial port to the ESP32
// ─────────────────────────────────────────────
hardware_interface::CallbackReturn MyRobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating — opening serial port %s", serial_port_.c_str()); //logs print

  if (!openSerial(serial_port_, 115200)) { //if this is not true
    RCLCPP_ERROR(logger_, "Failed to open serial port %s", serial_port_.c_str());//print this error
    return hardware_interface::CallbackReturn::ERROR; //retunr with error
  }

  // Wait for ESP32 to send ARM_READY
  RCLCPP_INFO(logger_, "Waiting for ESP32...");
  rclcpp::sleep_for(std::chrono::seconds(2)); //delay 2 seconds

  RCLCPP_INFO(logger_, "Hardware interface activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────
// on_deactivate — called when controller stops
// Closes the serial port cleanly
// ─────────────────────────────────────────────
hardware_interface::CallbackReturn MyRobotHardwareInterface::on_deactivate( //jus format same as hpp
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating — closing serial port");
  closeSerial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────
// export_state_interfaces
// Tells ros2_control: "I can give you position for each joint"
// ─────────────────────────────────────────────
std::vector<hardware_interface::StateInterface> //blah<> is dynamic array with input type
MyRobotHardwareInterface::export_state_interfaces() //calling fucntion xport
{
  std::vector<hardware_interface::StateInterface> state_interfaces;//“Create a variable named state_interfaces
                                                                  //  which is a vector (dynamic list) of hardware_interface::StateInterface objects.”

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back( ///empalace puts vectro insde the array
      info_.joints[i].name,           // joint name e.g. "base_yaw_joint"
      hardware_interface::HW_IF_POSITION,  // interface type = "position"
      &hw_positions_[i]               // pointer to our storage variable
    );
  }
  return state_interfaces;
}

// ─────────────────────────────────────────────
// export_command_interfaces
// Tells ros2_control: "I accept position commands for each joint"
// ─────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
MyRobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]
    );
  }
  return command_interfaces;
}

// ─────────────────────────────────────────────
// SERIAL HELPERS
// ─────────────────────────────────────────────

bool MyRobotHardwareInterface::openSerial(const std::string & port, int /*baud*/) //funciton which returns bool value
{
  // Open the port — O_RDWR=read+write, O_NOCTTY=don't make it controlling terminal
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) return false; //if port open failed

  struct termios tty;//struct- user define data type gorups related datatypes 
  tcgetattr(serial_fd_, &tty); //termius for baudrate etc  //store atribute in tty

  // Set baud rate
  cfsetispeed(&tty, B115200); //updating tty input rate
  cfsetospeed(&tty, B115200); //uopdating tty output rate

  // 8N1 mode — 8 data bits, no parity, 1 stop bit (same as Arduino Serial.begin)
  tty.c_cflag &= ~PARENB;   // no parity
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;        // 8 data bits
  tty.c_cflag |= CREAD | CLOCAL;

  // Raw mode — don't process data, just pass it through as-is
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  // Read timeout: wait up to 0.5s for data
  tty.c_cc[VMIN]  = 0; //dont wait for min bytes
  tty.c_cc[VTIME] = 1; //wait 0.5 s for data

  tcsetattr(serial_fd_, TCSANOW, &tty); //apply setting to tty
  return true; //working serial port
}

void MyRobotHardwareInterface::closeSerial() //function
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool MyRobotHardwareInterface::writeSerial(const std::string & msg) //function accpet string and stores in msg
{
  return ::write(serial_fd_, msg.c_str(), msg.size()) > 0; //for writting to serial
}

std::string MyRobotHardwareInterface::readLine() // calling readline fucntion from hpp
{
  std::string line;
  char c;
  // Read one character at a time until newline
  while (::read(serial_fd_, &c, 1) > 0) {
    if (c == '\n') break; //break at new line
    line += c;
  }
  return line; //return value
}

// ─────────────────────────────────────────────
// read() — called every control cycle (~100Hz)
// Reads "STA x.xxxx x.xxxx x.xxxx" from ESP32
// and puts the values into hw_positions_
// ─────────────────────────────────────────────
hardware_interface::return_type MyRobotHardwareInterface::read( //format of read
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  std::string line = readLine(); //read line funciton in hpp

  // Ignore empty lines and ERR lines
  if (line.empty() || line.find("ERR") != std::string::npos) {
    return hardware_interface::return_type::OK;
  }

  // Check it starts with STA
  if (line.rfind("STA", 0) != 0) {
    return hardware_interface::return_type::OK;
  }

  // Parse "STA x.xxxx x.xxxx x.xxxx"
  double p0, p1, p2;
  if (sscanf(line.c_str(), "STA %lf %lf %lf", &p0, &p1, &p2) == 3) {
    hw_positions_[0] = p0;
    hw_positions_[1] = p1;
    hw_positions_[2] = p2;
  }

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────
// write() — called every control cycle (~100Hz)
// Takes values from hw_commands_ and sends
// "CMD x.xxxx x.xxxx x.xxxx\n" to ESP32
// ─────────────────────────────────────────────
hardware_interface::return_type MyRobotHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
    // Only send if command actually changed
  static double last_cmd[3] = {0.0, 0.0, 0.0};
  
  if (hw_commands_[0] == last_cmd[0] &&
      hw_commands_[1] == last_cmd[1] &&
      hw_commands_[2] == last_cmd[2]) {
    return hardware_interface::return_type::OK;  // nothing changed, skip
  }

  last_cmd[0] = hw_commands_[0];
  last_cmd[1] = hw_commands_[1];
  last_cmd[2] = hw_commands_[2];
  
  
    // Flush any pending incoming data before sending command
  tcflush(serial_fd_, TCIFLUSH);  

  // Format the command string exactly like our Serial Monitor test
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "CMD %.4f %.4f %.4f\n",  //snprintf = string +printf ith szie limit
    hw_commands_[0],
    hw_commands_[1],
    hw_commands_[2]);

  writeSerial(std::string(cmd));

  return hardware_interface::return_type::OK;
}


} // namespace my_robot_hardware


// This line registers the plugin with ROS2 — must be at the bottom of the file
// We'll add it at the end of Part C2
PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware::MyRobotHardwareInterface, //in library ::class
  hardware_interface::SystemInterface //inhertis from this both aprt of ros2_control
)