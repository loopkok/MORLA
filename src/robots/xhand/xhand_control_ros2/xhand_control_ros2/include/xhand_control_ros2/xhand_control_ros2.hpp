#ifndef __XHAND_CONTROL_ROS2_HPP__
#define __XHAND_CONTROL_ROS2_HPP__

#include <rclcpp/rclcpp.hpp>

#include "xhand_control.hpp"
#include "xhand_control_interfaces/msg/finger_sensor_state.hpp"
#include "xhand_control_interfaces/srv/read_hand_info.hpp"
#include "xhand_control_interfaces/srv/reset_sensor.hpp"
#include "xhand_control_interfaces/srv/set_hand_id.hpp"
#include "xhand_control_interfaces/srv/set_hand_name.hpp"
#include "xhand_control_interfaces/msg/x_hand_command.hpp"
#include "xhand_control_interfaces/msg/x_hand_sensor_state.hpp"
#include "xhand_control_interfaces/msg/x_hand_sensor_state_array.hpp"
#include "xhand_control_interfaces/msg/x_hand_state.hpp"
#include "xhand_control_interfaces/msg/x_hand_state_array.hpp"
#include "std_msgs/msg/bool.hpp"

using xhand_control_interfaces::msg::FingerSensorState;
using xhand_control_interfaces::msg::XHandCommand;
using xhand_control_interfaces::msg::XHandSensorState;
using xhand_control_interfaces::msg::XHandSensorStateArray;
using xhand_control_interfaces::msg::XHandState;
using xhand_control_interfaces::msg::XHandStateArray;

using xhand_control_interfaces::srv::ReadHandInfo;
using xhand_control_interfaces::srv::ResetSensor;
using xhand_control_interfaces::srv::SetHandId;
using xhand_control_interfaces::srv::SetHandName;

using namespace std::placeholders;

namespace xhand_control_ros2
{
  class XHandControlROS : public rclcpp::Node
  {
  public:
    XHandControlROS();
    ~XHandControlROS();

    bool init();
    void run();

  private:
    bool init_parameters();
    void command_callback(const XHandCommand::SharedPtr msg);
    void read_hand_info(ReadHandInfo::Request::SharedPtr req, ReadHandInfo::Response::SharedPtr res);
    void set_hand_id(SetHandId::Request::SharedPtr req, SetHandId::Response::SharedPtr res);
    void set_hand_name(SetHandName::Request::SharedPtr req, SetHandName::Response::SharedPtr res);
    void reset_sensor(ResetSensor::Request::SharedPtr req, ResetSensor::Response::SharedPtr res);
    bool map_to_vector(const std::vector<std::string> msg_names, std::vector<int>& map_vector);
    bool is_valid_hand_id(const uint8_t hand_id) const;

  private:
    rclcpp::Publisher<XHandStateArray>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr test_pub_;

    rclcpp::Subscription<XHandCommand>::SharedPtr command_sub_;
    rclcpp::Service<ReadHandInfo>::SharedPtr read_info_srv_;
    rclcpp::Service<SetHandId>::SharedPtr set_id_srv_;
    rclcpp::Service<SetHandName>::SharedPtr set_name_srv_;
    rclcpp::Service<ResetSensor>::SharedPtr reset_sensor_srv_;
    std::shared_ptr<xhand_control::XHandControl> xhand_control_;
    std::vector<uint8_t> hand_ids_;
    HandCommand_t hand_command_;
    HandState_t hand_state_;
    XHandStateArray hand_state_array_;
    std::vector<std::string> finger_joint_names_{
        "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
        "index_bend_joint", "index_joint1", "index_joint2",
        "mid_joint1", "mid_joint2", "ring_joint1",
        "ring_joint2", "pinky_joint1", "pinky_joint2"};
    double update_rate_{10.0};
  };
} // namespace xhand_control_ros2

#endif
