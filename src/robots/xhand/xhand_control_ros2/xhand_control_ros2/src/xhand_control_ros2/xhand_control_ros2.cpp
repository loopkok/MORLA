#include "xhand_control_ros2/xhand_control_ros2.hpp"

namespace xhand_control_ros2
{

  XHandControlROS::XHandControlROS() : Node("xhand_control_ros2") {
    state_pub_ =
        this->create_publisher<XHandStateArray>("xhand_state", rclcpp::QoS(1));
    command_sub_ =
        this->create_subscription<XHandCommand>("xhand_command", 1, std::bind(&XHandControlROS::command_callback, this, _1));

    read_info_srv_ =
        this->create_service<ReadHandInfo>("read_hand_info", std::bind(&XHandControlROS::read_hand_info, this, _1, _2));
    set_id_srv_ =
        this->create_service<SetHandId>("set_hand_id", std::bind(&XHandControlROS::set_hand_id, this, _1, _2));
    set_name_srv_ =
        this->create_service<SetHandName>("set_hand_name", std::bind(&XHandControlROS::set_hand_name, this, _1, _2));
    reset_sensor_srv_ =
        this->create_service<ResetSensor>("reset_hand_sensor", std::bind(&XHandControlROS::reset_sensor, this, _1, _2));
  }

  XHandControlROS::~XHandControlROS() {}

  bool XHandControlROS::init()
  {
    if (!init_parameters())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize parameters");
      return false;
    }
    xhand_control_ = std::make_unique<xhand_control::XHandControl>();
    RCLCPP_INFO(this->get_logger(), "XHandControlROS initialized,version %s",
                xhand_control_->get_sdk_version().c_str());
    //auto ifnames = xhand_control_->enumerate_devices("EtherCAT");
    //if (ifnames.empty())
    //{
    //  RCLCPP_ERROR(this->get_logger(), "No XHand devices found");
    //  return false;
    //}
    //std::string device_name{ifnames[0]};
    // 替换为 USB 串口通讯
    std::string port_name = this->get_parameter("port_name").as_string();
    uint32_t baudrate = 3000000;
    
    RCLCPP_INFO(this->get_logger(), "Attempting to open XHand on %s with baudrate %d", port_name.c_str(), baudrate);
    
    auto ret = xhand_control_->open_serial(port_name, baudrate);
    //if (xhand_control_->open_ethercat(device_name))
    if(ret)
    {
    
      RCLCPP_INFO(this->get_logger(), "XHand serial device opened successfully on %s", port_name.c_str());
      //RCLCPP_INFO(this->get_logger(), "XHand device opened");
      hand_ids_ = xhand_control_->list_hands_id();
      if (hand_ids_.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "No XHand devices found");
        return false;
      }
      // if (hand_ids_[0] == NULL)
      // {
      //   RCLCPP_WARN(this->get_logger(), "No hand found");

      //   while (hand_ids_[0] == NULL)
      //   {
      //     RCLCPP_WARN(this->get_logger(), "Reopen device...");
      //     xhand_control_->close_device();
      //     if (xhand_control_->open_ethercat(device_name))
      //     {
      //       hand_ids_ = xhand_control_->list_hands_id();
      //     }
      //     rclcpp::Rate(1).sleep();

      //   }
      //   RCLCPP_WARN(this->get_logger(), "Hand id: %s", hand_ids_[0]);
      // }

      std::vector<std::string> finger{"thumb", "index", "middle", "ring",
                                      "pinky"};
      for (auto &id : hand_ids_)
      {
        // Hand state
        hand_state_array_.hand_id.push_back(id);
        hand_state_array_.hand_name.push_back(
            xhand_control_->get_hand_name(id).second);
        hand_state_array_.hand_type.push_back(
            xhand_control_->get_hand_type(id).second);
        XHandState state;
        state.name = finger_joint_names_;
        state.position.resize(finger_joint_names_.size(), 0);
        state.effort.resize(finger_joint_names_.size(), 0);
        state.temperature.resize(finger_joint_names_.size(), 0);
        state.error_code.resize(finger_joint_names_.size(), 0);
        hand_state_array_.hand_states.push_back(state);

        // sensor state
        XHandSensorState sensor_state;
        for (int i = 0; i < finger.size(); i++)
        {
          FingerSensorState finger_sensor_state;
          finger_sensor_state.location = finger[i];
          finger_sensor_state.raw_force.resize(120);
          finger_sensor_state.raw_temperature.resize(20);
          sensor_state.finger_sensor_states.push_back(finger_sensor_state);
        }
        hand_state_array_.sensor_states.push_back(sensor_state);
        // hand_sensor_state_array_.hand_name.push_back
      }
      return true;
    }
    return false;
  }

  void XHandControlROS::run()
  {
    rclcpp::Rate update_rate(update_rate_);
    while (rclcpp::ok())
    {
      for (auto &id : hand_ids_)
      {
        hand_state_ = xhand_control_->read_state(id).second;
        for (int i = 0; i < hand_state_array_.hand_id.size(); i++)
        {
          if (hand_state_array_.hand_id[i] == id)
          {
            auto &pub_state = hand_state_array_.hand_states[i];
            for (int joint_idx = 0; joint_idx < hand_state_.finger_state.size();
                 joint_idx++)
            {
              pub_state.position[joint_idx] =
                  hand_state_.finger_state[joint_idx].position;
              pub_state.effort[joint_idx] =
                  hand_state_.finger_state[joint_idx].torque;
            }

            for (int j = 0; j < hand_state_.sensor_data.size(); j++)
            {
              auto &read_sensor_state = hand_state_.sensor_data[j];
              auto &pub_sensor_state =
                  hand_state_array_.sensor_states[i].finger_sensor_states[j];
              pub_sensor_state.calc_force.x = read_sensor_state.calc_force.fx;
              pub_sensor_state.calc_force.y = read_sensor_state.calc_force.fy;
              pub_sensor_state.calc_force.z = read_sensor_state.calc_force.fz;
              for (int k = 0; k < read_sensor_state.raw_force.size(); k++)
              {
                pub_sensor_state.raw_force[k].x =
                    read_sensor_state.raw_force[k].fx;
                pub_sensor_state.raw_force[k].y =
                    read_sensor_state.raw_force[k].fy;
                pub_sensor_state.raw_force[k].z =
                    read_sensor_state.raw_force[k].fz;
              }
              pub_sensor_state.calc_temperature =
                  read_sensor_state.calc_temperature;
              for (int k = 0; k < read_sensor_state.temperature.size(); k++)
              {
                pub_sensor_state.raw_temperature[k] =
                    read_sensor_state.temperature[k];
              }
            }
            break;
          }
        }
      }
      hand_state_array_.header.stamp = this->now();
      state_pub_->publish(hand_state_array_);
      rclcpp::spin_some(this->get_node_base_interface());
      update_rate.sleep();
    }
  }

  bool XHandControlROS::init_parameters()
  {
    int error = 0;
    this->declare_parameter("update_rate", 10.0);
    error += !this->get_parameter("update_rate", update_rate_);
    
    this->declare_parameter("port_name", "/dev/ttyUSB0");

    return error == 0;
  }

  void XHandControlROS::command_callback(const XHandCommand::SharedPtr msg)
  {
    if (!is_valid_hand_id(msg->hand_id))
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid hand id");
      return;
    }
    if (msg->name.size() != 0 && msg->name.size() == msg->position.size() &&
        msg->name.size() == msg->kp.size() &&
        msg->name.size() == msg->kd.size() &&
        msg->name.size() == msg->ki.size() &&
        msg->name.size() == msg->effort_limit.size())
    {
      std::vector<int> map_vec;
      if (!map_to_vector(msg->name, map_vec))
      {
        RCLCPP_ERROR(this->get_logger(), "Invalid command");
        return;
      }
      HandCommand_t cmd;
      for (int i = 0; i < cmd.finger_command.size(); ++i)
      {
        cmd.finger_command[i].id = i;
        cmd.finger_command[i].position = 0;
        cmd.finger_command[i].kp = 0;
        cmd.finger_command[i].kd = 0;
        cmd.finger_command[i].ki = 0;
        cmd.finger_command[i].tor_max = 0;
        cmd.finger_command[i].mode = 0;
      }
      for (int i = 0; i < msg->name.size(); i++)
      {
        cmd.finger_command[map_vec[i]].id = map_vec[i];
        cmd.finger_command[map_vec[i]].position = msg->position[i];
        cmd.finger_command[map_vec[i]].kp = msg->kp[i];
        cmd.finger_command[map_vec[i]].kd = msg->kd[i];
        cmd.finger_command[map_vec[i]].ki = msg->ki[i];
        cmd.finger_command[map_vec[i]].tor_max = msg->effort_limit[i];
        cmd.finger_command[map_vec[i]].mode = msg->mode;
      }
      xhand_control_->send_command(msg->hand_id, cmd);
      //RCLCPP_INFO(this->get_logger(), "Command sent");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid command");
    }
  }

  void XHandControlROS::read_hand_info(ReadHandInfo::Request::SharedPtr req, ReadHandInfo::Response::SharedPtr res)
  {
    if (!is_valid_hand_id(req->hand_id))
    {
      res->success = false;
      res->info = "Invalid hand id";
      return;
    }

    switch (req->info_type)
    {
    case ReadHandInfo::Request::HAND_TPYE:
      res->success = true;
      res->info = xhand_control_->get_hand_type(req->hand_id).second;
      break;
    case ReadHandInfo::Request::SERIAL_NUMBER:
      res->success = true;
      res->info = xhand_control_->get_serial_number(req->hand_id).second;
      break;
    case ReadHandInfo::Request::VERSION:
      res->success = true;
      res->info = xhand_control_->read_version(req->hand_id, 0).second;
      break;
    case ReadHandInfo::Request::HAND_NAME:
      res->success = true;
      res->info = xhand_control_->get_hand_name(req->hand_id).second;
      break;
    default:
      res->success = false;
      res->info = "Invalid info type";
      break;
    }
    return;
  }
  
  void XHandControlROS::set_hand_id(SetHandId::Request::SharedPtr req, SetHandId::Response::SharedPtr res)
  {
    if (!is_valid_hand_id(req->current_id))
    {
      res->success = false;
      res->message = "Invalid hand id";
      return;
    }
    auto ret = xhand_control_->set_hand_id(req->current_id, req->new_id);
    if (!ret)
    {
      res->success = false;
      res->message = ret.error_message;
    }
    else
    {
      res->success = true;
    }
    return;
  }
  
  void XHandControlROS::set_hand_name(SetHandName::Request::SharedPtr req, SetHandName::Response::SharedPtr res)
  {
    if (!is_valid_hand_id(req->hand_id))
    {
      res->success = false;
      res->message = "Invalid hand id";
      return;
    }
    auto ret = xhand_control_->set_hand_name(req->hand_id, req->hand_name);
    if (!ret)
    {
      res->success = false;
      res->message = ret.error_message;
    }
    else
    {
      res->success = true;
    }
    return;
  }
  
  void XHandControlROS::reset_sensor(ResetSensor::Request::SharedPtr req, ResetSensor::Response::SharedPtr res)
  {
    if (!is_valid_hand_id(req->hand_id))
    {
      res->success = false;
      res->message = "Invalid hand id";
      return;
    }
    if (req->sensor_id < 0x11 || req->sensor_id > 0x15)
    {
      res->success = false;
      res->message = "Invalid sensor id";
      return;
    }
    auto ret = xhand_control_->reset_sensor(req->hand_id, req->sensor_id);
    if (!ret)
    {
      res->success = false;
      res->message = ret.error_message;
    }
    else
    {
      res->success = true;
    }
    return;
  }
  
  bool XHandControlROS::map_to_vector(const std::vector<std::string> msg_names, std::vector<int>& map_vector)
  {
    map_vector.resize(msg_names.size());
    for (int i = 0; i < msg_names.size(); i++)
    {
      bool found = false;
      for (int j = 0; j < finger_joint_names_.size(); j++)
      {
        if (finger_joint_names_[j] == msg_names[i])
        {
          map_vector[i] = j;
          found = true;
          break;
        }
      }
      if (!found)
      {
        RCLCPP_ERROR(this->get_logger(), "Invalid finger name: %s", msg_names[i].c_str());
        return false;
      }
    }
    return true;
  }

  bool XHandControlROS::is_valid_hand_id(const uint8_t hand_id) const
  {
    if (std::find(hand_ids_.begin(), hand_ids_.end(), hand_id) == hand_ids_.end())
    {
      return false;
    }
    return true;
  }
} // namespace xhand_control_ros2
