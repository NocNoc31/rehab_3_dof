#ifndef ODRIVE_CAN_INTERFACE_HPP
#define ODRIVE_CAN_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstdint>
#include <string>

class ODriveCANInterface : public rclcpp::Node {
public:
  explicit ODriveCANInterface(const std::string& node_name, const std::string& can_interface, uint32_t node_id);
  ~ODriveCANInterface();

  void set_axis_state(uint32_t state);
  void set_controller_mode(uint32_t control_mode, uint32_t input_mode);
  void set_input_position(float pos, int16_t vel_ff = 0, int16_t torque_ff = 0);
  void set_input_torque(float torque);
  void set_input_velocity(float vel, float torque_ff = 0);
  void set_velocity_limit(float vel_limit);
  void set_torque_limit(float torque_limit);

  struct Heartbeat {
    uint32_t axis_state;
    uint8_t axis_error;
    uint8_t axis_flags;
    uint8_t controller_flags;
    uint8_t motor_flags;
  };
  Heartbeat get_heartbeat();

  struct EncoderEstimates {
    float pos;
    float vel;
  };
  EncoderEstimates get_encoder_estimates();

  void estop();
  void clear_errors(uint8_t identify = 0);
  float get_vbus_voltage();

private:
  void send_can_message(uint32_t cmd_id, const uint8_t* data, uint8_t len, bool rtr = false);
  bool receive_can_message(struct can_frame& frame, int timeout_ms);

  int can_socket_;
  uint32_t node_id_;
};

#endif // ODRIVE_CAN_INTERFACE_HPP