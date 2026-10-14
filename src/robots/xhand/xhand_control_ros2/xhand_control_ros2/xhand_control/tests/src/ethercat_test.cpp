#include <string.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <sstream>

#include "xhand_control.hpp"
std::atomic<bool> exit_flag(false);

// Signal handler for SIGINT
void signalHandler(int signum) {
  std::cout << "\nInterrupt signal (" << signum << ") received.\n";
  exit_flag.store(true);
}
int main() {
  std::signal(SIGINT, signalHandler);
  xhand_control::XHandControl xhand_control;
  auto ifnames = xhand_control.enumerate_devices("EtherCAT");

  if (ifnames.empty()) {
    std::cout << "No EtherCAT devices found" << std::endl;
    return 0;
  }

  for (const auto& ifname : ifnames) {
    std::cout << ifname << std::endl;
  }

  std::string device_name{ifnames[0]};

  auto ret = xhand_control.open_ethercat(device_name);
  int mode = 3;
  int kp = 225;
  int ki = 225;
  int kd = 225;
  int tor_max = 0;
  if (ret) {
    std::cout << "Ethercat opened successfully" << std::endl;
    auto hand_list = xhand_control.list_hands_id();
    for (auto hand_id : hand_list) {
      std::cout << "hand id: " << (int)hand_id << std::endl;
    }

    {
      auto ret = xhand_control.get_hand_type(hand_list[0]);
      if (ret.first)
        std::cout << "type: " << ret.second << std::endl;
      else
        std::cout << "get type failed:" << ret.first.error_code
                  << ret.first.error_message << std::endl;
    }

    // Reset hand name
    {
      auto ret = xhand_control.set_hand_name(hand_list[0], "xhand");
      if (ret)
        std::cout << "set name successfully,current name "
                  << xhand_control.get_hand_name(hand_list[0]).second
                  << std::endl;
      else
        std::cout << "set name failed:" << ret.error_code;
    }

    // Reset hand sensor
    {
      // sensor id is [0x11,0x15], only call if needed
      auto ret = xhand_control.reset_sensor(hand_list[0], 0x11);
      if (ret)
        std::cout << "reset sensor successfully" << std::endl;
      else
        std::cout << "reset sensor failed:" << ret.error_code << std::endl;
    }

    {
      // read palm version
      auto ret = xhand_control.read_version(hand_list[0], 0x80 | hand_list[0]);
      if (ret.first) {
        std::cout << " plam version: " << ret.second << std::endl;
      } else {
        std::cout << "read version failed:" << ret.first.error_code << "  "
                  << ret.first.error_message << std::endl;
      }
    }

    {
      auto ret = xhand_control.read_version(hand_list[0], 17);
      if (ret.first) {
        std::cout << " finger 0 version: " << ret.second << std::endl;
      } else {
        std::cout << "read version failed:" << ret.first.error_code << "  "
                  << ret.first.error_message << std::endl;
      }
    }

    // Get action counts
    {
      auto ret = xhand_control.get_action_counts(hand_list[0], 1000);
      if (ret.first) {
        for (int i = 0; i < ret.second.size(); i++) {            
            std::cout << "joint " << i << " action total counts: " << ret.second[i][0] << std::endl;
            std::cout << "joint " << i << " action direction counts: " << ret.second[i][1] << std::endl;
        }

      } else {
        std::cout << "get action group count failed:" << ret.first.error_code
                  << "  " << ret.first.error_message << std::endl;
      }
    }

    // read hand parameter
    {
      auto ret = xhand_control.read_parameters(hand_list[0]);
      if (ret.first) {
        for (int i = 0; i < 12; i++) {
          printf("finger %d parameter: %d,%d,%d,%d,%d,%d\n", i,
                 ret.second.position_stretching[i],
                 ret.second.position_closing[i], ret.second.position_zero[i],
                 ret.second.position_init[i], ret.second.angle_stretching[i],
                 ret.second.angle_closing[i]);
        }
      } else {
        std::cout << "read hand parameter failed:" << ret.first.error_code
                  << "  " << ret.first.error_message << std::endl;
      }
    }

    {
      std::cout << "select follow num to choose fingure mode:" << std::endl;
      std::cout << "\t0\t---\tPassive Mode" << std::endl;
      std::cout << "\t3\t---\tPosition Control Mode" << std::endl;
      std::cout << "\t5\t---\tForce Control Mode" << std::endl;
      std::string input;
      std::cin >> input;
      std::cin.get();

      int choice;
      std::istringstream iss(input);
      if (!(iss >> choice)) {
          std::cerr << "Invalid input. Please enter a number." << std::endl;
          xhand_control.close_device();
          std::cout << "Exiting program.\n";
          return 1;
      }

      switch (choice) {
          case 0:
              std::cout << "Selected mode: Passive Mode" << std::endl;
              break;
          case 3:
              std::cout << "Selected mode: Position Control Mode Control Mode" << std::endl;
              kp = 225;
              ki = 0;
              kd = 0;
              tor_max = 350;
              break;
          case 5:
              std::cout << "Selected mode: Force Control Mode" << std::endl;
              kp = 0;
              ki = 0;
              kd = 0;
              tor_max = 0;
              break;
          default:
              std::cout << "Invalid choice. Please select a valid mode." << std::endl;
              break;
      }
      mode = choice;
    }

    // {
    //   std::cout << "press y or n to calibrate finger joint" << std::endl;
    //   std::string input;
    //   std::cin >> input;
    //   std::cin.get();
    //   bool calibration_success = false;
    //   if (input == "y") {
    //     while (!exit_flag.load() && calibration_success == false)
    //     {
    //       // wait for user to press enter the button
    //       std::vector<int8_t> calibrated_angles = {
    //         10, 60, 30, 0, 10, 10, 10, 10, 10, 10, 10, 10
    //       };

    //       std::vector<int8_t> angles_limit = {
    //           0, 105, -60, 90, -10, 105, 10, -10, 0, 110, 0, 110, 
    //           0, 110, 0, 110, 0, 110, 0, 110, 0, 110, 0, 110};

    //       // MH2.25
    //       std::vector<int8_t> lower_angles = {
    //           0, -60, -10, -10, 0, 0, 0, 0, 0, 0, 0, 0
    //       };
    //       std::vector<int8_t> upper_angles = {
    //           105, 90, 105, 10, 110, 110, 110, 110, 110, 110, 110, 110
    //       };

    //       for (size_t i = 0; i < lower_angles.size(); i++)
    //       {
    //         angles_limit[i * 2] = lower_angles[i];
    //         angles_limit[i * 2 + 1] = upper_angles[i];
    //       }
          

    //       std::string tools = "S1";
    //       uint8_t step = 1;

    //       // uint8_t device_id = 0x80;
          
    //       auto ret = xhand_control.calibrate_joint_by_mold(hand_list[0], tools,step, calibrated_angles, angles_limit);

    //       if (ret) {
    //         std::cout << "command sent successfully" << std::endl;
    //         calibration_success = true;
    //       } else {
    //         std::cout << "\ncommand sent failed:\ncode: " << ret.error_code << "\nmsg: "
    //                   << ret.error_message << std::endl;
    //         std::cout << "\npress y or n to calibrate finger joint" << std::endl;
    //         std::string input;
    //         std::cin >> input;
    //         std::cin.get();
    //       }
    //     }
        
    //   }
    // }

    // Firmware upgrade

    // joint
    // {
    //   std::cout << "press y or n to upgrade joint" << std::endl;
    //   std::string input;
    //   std::cin >> input;
    //   if (input == "y") {
    //     for (int i = 5; i < 12; i++) {
    //       auto ret = xhand_control.upgrade_device(
    //           hand_list[0], i,
    //           "../firmware/jointboard_g0_v1.0.7_20241016151903.bin");
    //       if (ret) {
    //         std::cout << "upgrade joint successfully" << std::endl;
    //       } else {
    //         std::cout << "upgrade joint failed:" << ret.error_code << "  "
    //                   << ret.error_message << std::endl;
    //         break;
    //       }
    //     }
    //   }
    // }

    // fingure
    // {
    //   std::cout << "press y or n to upgrade finger" << std::endl;
    //   std::string input;
    //   std::cin >> input;
    //   if (input == "y") {
    //     for (int i = 17; i < 22; i++) {
    //       auto ret = xhand_control.upgrade_device(
    //           hand_list[0], i,
    //           "../firmware/fingerboard_g071_v1.0.2_20250108150204.bin");
    //       if (ret) {
    //         std::cout << "upgrade finger successfully" << std::endl;
    //       } else {
    //         std::cout << "upgrade finger failed:" << ret.error_code << "  "
    //                   << ret.error_message << std::endl;
    //         break;
    //       }
    //     }
    //   }
    // }

    // palm
    // {
    //   // user confirm if upgrade palm y or n
    //   std::cout << "press y or n to upgrade palm" << std::endl;
    //   std::string input;
    //   std::cin >> input;
    //   if (input == "y") {
    //     auto ret = xhand_control.upgrade_device(
    //         hand_list[0], 0x80 | hand_list[0],
    //           "../firmware/commboard_v133_20250218093732.bin");
    //     if (ret) {
    //       std::cout << "upgrade palm successfully" << std::endl;
    //     } else {
    //       std::cout << "upgrade palm failed:" << ret.error_code << "  "
    //                 << ret.error_message << std::endl;
    //     }
    //   }
    // }

    // if (xhand_control.reset_hand(hand_list[0])) {
    //   std::cout << "reset hand successfully" << std::endl;
    // }
    // sensor id is [0x11,0x15], only call if needed
    // if (xhand_control.reset_sensor(hand_list[0], 0x11)) {
    //   std::cout << "reset sensor successfully" << std::endl;
    // }
  } else {
    std::cout << "Failed to open EtherCAT device" << ret.error_code
              << ret.error_message << std::endl;
    return 0;
  }
  auto hand_list = xhand_control.list_hands_id();
  // Main loop
  HandCommand_t command;
  memset(&command, 0, sizeof(command));
  for (int i = 0; i < 12; i++) {
    command.finger_command[i].id = i;
    command.finger_command[i].position = 0;
    command.finger_command[i].kp = kp;
    command.finger_command[i].ki = ki;
    command.finger_command[i].kd = kd;
    command.finger_command[i].mode = mode;
  }
  while (!exit_flag.load()) {
    std::cout << "Enter finger id: ";
    std::string finger_input;
    std::cin >> finger_input;
    std::istringstream iss(finger_input);
    int finger_id;
    char extra;
    if (!(iss >> finger_id) || (iss >> extra) || finger_id < 0 || finger_id > 11) {
      std::cout << "Invalid finger id: " << finger_input << std::endl;
      continue;
    }
    if (exit_flag.load()) break;

    float position = 0.0f;
    if (mode == 5) {
      std::cout << "Enter tor_max (±1200): ";
      std::string tor_input;
      std::cin >> tor_input;
      std::istringstream tor_iss(tor_input);
      int tor_val;
      char tor_extra;
      if (!(tor_iss >> tor_val) || (tor_iss >> tor_extra) || tor_val < -1200 || tor_val > 1200) {
        std::cout << "Invalid tor_max: " << tor_input << ". Please enter an integer between -1200 and 1200." << std::endl;
        continue;
      }
      tor_max = tor_val;
    } else {
      std::cout << "Enter position: ";
      std::cin >> position;  // Wait for user input
    }
    if (exit_flag.load()) break;
    
    command.finger_command[finger_id].position = position;
    command.finger_command[finger_id].kp = kp;
    command.finger_command[finger_id].ki = ki;
    command.finger_command[finger_id].kd = kd;
    command.finger_command[finger_id].mode = mode;
    command.finger_command[finger_id].tor_max = tor_max;
    auto ret = xhand_control.send_command(hand_list[0], command);
    if (ret) {
      std::cout << "command sent successfully" << std::endl;
    } else {
      std::cout << "command sent failed " << ret.error_code << "  "
                << ret.error_message << std::endl;
    }

    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      auto ret = xhand_control.read_state(hand_list[0]);
      if (ret.first) {
        auto state = ret.second;
        // int finger_id = 0;
        std::cout << "Finger " << finger_id
                  << " position: " << (state.finger_state[finger_id].position)
                  << " control mode: "
                  << (state.finger_state[finger_id].raw_position & 0xf)
                  << " raw position: "
                  << ((state.finger_state[finger_id].raw_position >> 4) & 0xfff)
                  << " torq: " << state.finger_state[finger_id].torque
                  << " commboard_err:" << state.finger_state[finger_id].commboard_err
                  << " jointboard_err:" << state.finger_state[finger_id].jonitboard_err
                  << " tipboard_err:" << state.finger_state[finger_id].tipboard_err
                  << std::endl;
        // for (int finger_id = 0; finger_id < 1; finger_id++) {
        //   auto state = ret.second;
        //   std::cout << "Finger " << finger_id << " position: "
        //             << ((state.finger_state[finger_id].raw_position >> 4) &
        //                 0x0fff)
        //             << " control mode: "
        //             << (state.finger_state[finger_id].raw_position & 0xf)
        //             << " raw position: "
        //             << state.finger_state[finger_id].raw_position <<
        //             std::endl;
        // }
      } else {
        std::cout << "read state failed" << ret.first.error_code << "  "
                  << ret.first.error_message << std::endl;
      }
    }

    // std::cout << "Finger " << finger_id
    //           << " position: " << state.finger_state[finger_id].position
    //           << std::endl;
  }
  xhand_control.close_device();
  std::cout << "Exiting program.\n";
  return 0;
}