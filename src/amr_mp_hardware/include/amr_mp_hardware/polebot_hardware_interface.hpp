#ifndef AMR_MP_HARDWARE__POLEBOT_HARDWARE_INTERFACE_HPP_

#define AMR_MP_HARDWARE__POLEBOT_HARDWARE_INTERFACE_HPP_



#include <string>

#include <vector>

#include <thread>

#include <mutex>

#include <atomic>

#include <chrono>

#include <cstdint>



#include "rclcpp/rclcpp.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"

#include "hardware_interface/system_interface.hpp"

#include "hardware_interface/handle.hpp"

#include "hardware_interface/hardware_info.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "std_msgs/msg/int64_multi_array.hpp"



using hardware_interface::return_type;



namespace amr_mp_hardware

{



class PoleBotHardwareInterface : public hardware_interface::SystemInterface

{

public:

  PoleBotHardwareInterface() = default;

  ~PoleBotHardwareInterface() = default;



  return_type configure(const hardware_interface::HardwareInfo & info) override;



  std::vector<hardware_interface::StateInterface>

  export_state_interfaces() override;



  std::vector<hardware_interface::CommandInterface>

  export_command_interfaces() override;



  return_type start() override;

  return_type stop() override;



  std::string get_name() const override

  { return "PoleBotHardwareInterface"; }



  hardware_interface::status get_status() const override

  { return hardware_interface::status::UNKNOWN; }



  return_type read()  override;

  return_type write() override;



private:

  void ticks_callback(

    const std_msgs::msg::Int64MultiArray::SharedPtr msg);



  hardware_interface::HardwareInfo info_;



  double ticks_per_rev_   {360000.0};

  double gear_ratio_      {1.0};

  double wheel_radius_m_  {0.055};

  double max_linear_vel_  {0.5};

  double max_vel_rad_s_   {9.09};

  std::string wheel_ticks_topic_ {"/wheel_ticks"};

  std::string wheel_cmd_topic_   {"/wheel_cmd_vel"};



  std::vector<double> hw_positions_;

  std::vector<double> hw_velocities_;

  std::vector<double> hw_commands_;

  std::vector<uint32_t> last_ticks_;

  bool ticks_initialized_ {false};

  std::chrono::steady_clock::time_point last_tick_time_;

  std::mutex state_mutex_;



  std::shared_ptr<rclcpp::Node> hw_node_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr ticks_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  cmd_pub_;



  std::thread spin_thread_;

  std::atomic<bool> spinning_ {false};

};



}  // namespace amr_mp_hardware



#endif

