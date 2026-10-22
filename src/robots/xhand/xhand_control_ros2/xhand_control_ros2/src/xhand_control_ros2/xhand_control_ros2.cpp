#include "xhand_control_ros2/xhand_control_ros2.hpp"
#include <cmath>

namespace xhand_control_ros2
{

  XHandControlROS::XHandControlROS() : Node("xhand_control_ros2") {
    state_pub_ =
        this->create_publisher<XHandStateArray>("xhand_state", rclcpp::SensorDataQoS().keep_last(1));
    sdk_status_pub_ = this->create_publisher<std_msgs::msg::String>("sdk_status", rclcpp::QoS(1));
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
    start_feedback_srv_ = this->create_service<std_srvs::srv::Trigger>("start_feedback",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        if (!piano_feedback_mode_ || !xhand_control_ || hand_ids_.empty() || piano_lease_fresh()) {
          res->success = false; res->message = "Feedback start requires idle piano mode and initialized serial"; return;
        }
        set_passive_command(); feedback_active_ = true; ++feedback_generation_;
        res->success = true; res->message = "Explicit passive feedback start accepted; waiting for device acknowledgement";
      });
    stop_feedback_srv_ = this->create_service<std_srvs::srv::Trigger>("stop_feedback",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        set_passive_command();
        res->success = true;
        if (piano_feedback_mode_ && feedback_active_ && xhand_control_) {
          for (auto id : hand_ids_) if (!xhand_control_->send_command(id, poll_command_)) res->success = false;
        }
        feedback_active_ = false;
        res->message = res->success ? "Passive stop sent; feedback stopped" : "Passive stop acknowledgement failed; inspect hardware";
      });
    auto lease_qos = rclcpp::QoS(1).reliable().transient_local();
    piano_lease_sub_ = this->create_subscription<std_msgs::msg::String>("/policy_inference/state", lease_qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        try {
          const auto state = nlohmann::json::parse(msg->data);
          const double now = this->now().seconds();
          const double stamp = state.at("sent_at").get<double>();
          piano_lease_allowed_ = state.value("activity", std::string()) == "playback"
             && !state.value("takeover_pending", false) && std::isfinite(stamp) && now-stamp >= -.05 && now-stamp <= .5;
          piano_lease_seen_ = std::chrono::steady_clock::now();
        } catch (...) { piano_lease_allowed_ = false; }
      });
  }

  XHandControlROS::~XHandControlROS()
  {
    if (xhand_control_)
    {
      if (piano_feedback_mode_ && feedback_active_) {
        set_passive_command();
        for (auto id : hand_ids_) xhand_control_->send_command(id, poll_command_);
      }
      // Explicitly stop the vendor reader and release the serial device.
      xhand_control_->close_device();
    }
  }

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
      rclcpp::spin_some(this->get_node_base_interface());
      bool feedback_valid = !piano_feedback_mode_ || feedback_active_;
      int read_error = 0;
      nlohmann::json joint_diagnostics = nlohmann::json::array();
      nlohmann::json send_ack = nullptr;
      std::string read_message = feedback_valid ? "" : "Awaiting explicit passive feedback initialization";
      for (auto &id : hand_ids_)
      {
        if (!feedback_valid) break;
        if (piano_feedback_mode_) {
          if (!piano_lease_fresh()) set_passive_command();
          const auto sent = xhand_control_->send_command(id, poll_command_);
          send_ack = static_cast<bool>(sent);
          if (!sent) {
            feedback_valid = false; read_error = sent.error_code; read_message = sent.error_message; break;
          }
        }
        const auto reading = xhand_control_->read_state(id, false);
        if (!reading.first)
        {
          feedback_valid = false;
          read_error = reading.first.error_code;
          read_message = reading.first.error_message;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "XHand %u state read failed: %s; feedback not refreshed",
                              static_cast<unsigned>(id), reading.first.error_message.c_str());
          continue;
        }
        hand_state_ = reading.second;
        bool joints_valid = hand_state_.finger_state.size() == finger_joint_names_.size();
        for (const auto &joint : hand_state_.finger_state)
        {
          joints_valid = joints_valid && std::isfinite(joint.position) && std::isfinite(joint.torque);
        }
        if (!joints_valid)
        {
          feedback_valid = false;
          read_error = -1;
          read_message = "non-finite or wrong-sized joint feedback";
          continue;
        }
        if (piano_feedback_mode_) {
          nlohmann::json joints = nlohmann::json::array();
          for (size_t j = 0; j < hand_state_.finger_state.size(); ++j) {
            const auto &actual = hand_state_.finger_state[j];
            const auto &target = poll_command_.finger_command[j];
            joints.push_back({{"name", finger_joint_names_[j]}, {"position_rad", actual.position},
                {"raw_position", actual.raw_position}, {"current_ma", actual.torque},
                {"temperature_raw", actual.temperature}, {"commboard_err", actual.commboard_err},
                {"jointboard_err", actual.jonitboard_err}, {"tipboard_err", actual.tipboard_err},
                {"command_mode", target.mode}, {"target_rad", target.position},
                {"kp", target.kp}, {"current_limit_ma", target.tor_max}});
          }
          joint_diagnostics.push_back({{"hand_id", id}, {"joints", joints}});
        }
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

            for (size_t j = 0; j < hand_state_.sensor_data.size() &&
                 j < hand_state_array_.sensor_states[i].finger_sensor_states.size(); j++)
            {
              auto &read_sensor_state = hand_state_.sensor_data[j];
              auto &pub_sensor_state =
                  hand_state_array_.sensor_states[i].finger_sensor_states[j];
              pub_sensor_state.calc_force.x = read_sensor_state.calc_force.fx;
              pub_sensor_state.calc_force.y = read_sensor_state.calc_force.fy;
              pub_sensor_state.calc_force.z = read_sensor_state.calc_force.fz;
              for (size_t k = 0; k < read_sensor_state.raw_force.size() && k < pub_sensor_state.raw_force.size(); k++)
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
              for (size_t k = 0; k < read_sensor_state.temperature.size() && k < pub_sensor_state.raw_temperature.size(); k++)
              {
                pub_sensor_state.raw_temperature[k] =
                    read_sensor_state.temperature[k];
              }
            }
            break;
          }
        }
      }
      if (feedback_valid)
      {
        hand_state_array_.header.stamp = this->now();
        state_pub_->publish(hand_state_array_);
      }
      std_msgs::msg::String diagnostic;
      nlohmann::json sdk_status = {{"feedback_valid", feedback_valid},
          {"error_code", read_error}, {"error", read_message}, {"feedback_active", feedback_active_},
          {"feedback_generation", feedback_generation_}, {"passive", !piano_lease_fresh()}};
      if (piano_feedback_mode_) {
        bool passive = true;
        for (const auto &target : poll_command_.finger_command) passive = passive && target.mode == 0;
        sdk_status["passive"] = passive;
        sdk_status["send_ack"] = send_ack;
        sdk_status["device_diagnostics"] = feedback_valid ? joint_diagnostics : nlohmann::json::array();
      }
      diagnostic.data = sdk_status.dump();
      sdk_status_pub_->publish(diagnostic);
      update_rate.sleep();
    }
  }

  bool XHandControlROS::init_parameters()
  {
    int error = 0;
    this->declare_parameter("update_rate", 10.0);
    error += !this->get_parameter("update_rate", update_rate_);
    
    this->declare_parameter("port_name", "/dev/ttyUSB0");
    this->declare_parameter("piano_feedback_mode", false);
    this->get_parameter("piano_feedback_mode", piano_feedback_mode_);
    set_passive_command();

    return error == 0;
  }

  void XHandControlROS::command_callback(const XHandCommand::SharedPtr msg)
  {
    if (piano_feedback_mode_ && (!feedback_active_ || !piano_lease_fresh())) {
      RCLCPP_ERROR(this->get_logger(), "Piano hand command requires explicit feedback start and fresh playback lease");
      return;
    }
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
      // NEXUS command_packet.hpp: reserved words must also be initialized.
      HandCommand_t cmd{};
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
      if (piano_feedback_mode_) poll_command_ = cmd;
      else xhand_control_->send_command(msg->hand_id, cmd);
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

  void XHandControlROS::set_passive_command() {
    poll_command_ = HandCommand_t{};
    for (size_t i=0; i<poll_command_.finger_command.size(); ++i) poll_command_.finger_command[i].id = i;
  }

  bool XHandControlROS::piano_lease_fresh() const {
    return piano_lease_allowed_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now()-piano_lease_seen_).count() <= .5;
  }
} // namespace xhand_control_ros2
