#include "xhand_control_ros2/xhand_control_ros2.hpp"

int main(int argc, char** argv) {
  using namespace xhand_control_ros2;
  rclcpp::init(argc, argv);

  auto xhand_control = std::make_shared<XHandControlROS>();
  if (!xhand_control->init()) {
    RCLCPP_ERROR(rclcpp::get_logger("xhand_control_ros2"), "Failed to initialize xhand_control");
    // return 0; 
  } else {
    xhand_control->run();
  }

  rclcpp::shutdown();
  return 0;
}