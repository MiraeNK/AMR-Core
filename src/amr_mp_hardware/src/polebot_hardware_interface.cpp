#include "amr_mp_hardware/polebot_hardware_interface.hpp"

#include <cmath>



namespace amr_mp_hardware

{



return_type PoleBotHardwareInterface::configure(

  const hardware_interface::HardwareInfo & info)

{

  info_ = info;



  auto get_d = [&](const std::string & k, double dv) -> double {

    auto it = info_.hardware_parameters.find(k);

    return (it != info_.hardware_parameters.end()) ?

      std::stod(it->second) : dv;

  };

  auto get_s = [&](const std::string & k, const std::string & dv) -> std::string {

    auto it = info_.hardware_parameters.find(k);

    return (it != info_.hardware_parameters.end()) ? it->second : dv;

  };



  ticks_per_rev_     = get_d("ticks_per_rev",    360000.0);

  gear_ratio_        = get_d("gear_ratio",        1.0);

  wheel_radius_m_    = get_d("wheel_radius_m",    0.055);

  max_linear_vel_    = get_d("max_linear_vel",    0.5);

  wheel_ticks_topic_ = get_s("wheel_ticks_topic", "/wheel_ticks");

  wheel_cmd_topic_   = get_s("wheel_cmd_topic",   "/wheel_cmd_vel");



  max_vel_rad_s_ = (wheel_radius_m_ > 0.0) ?

    max_linear_vel_ / wheel_radius_m_ : 9.09;



  if (info_.joints.size() != 2) {

    RCLCPP_ERROR(rclcpp::get_logger("PoleBotHW"),

      "Butuh 2 joint. Ditemukan: %zu", info_.joints.size());

    return return_type::ERROR;

  }



  hw_positions_.assign(2, 0.0);

  hw_velocities_.assign(2, 0.0);

  hw_commands_.assign(2, 0.0);

  last_ticks_.assign(2, 0u);



  RCLCPP_INFO(rclcpp::get_logger("PoleBotHW"),

    "Configure OK: ticks=%.0f gear=%.1f r=%.3fm lin=%.2f rad=%.2f",

    ticks_per_rev_, gear_ratio_, wheel_radius_m_,

    max_linear_vel_, max_vel_rad_s_);



  return return_type::OK;

}



std::vector<hardware_interface::StateInterface>

PoleBotHardwareInterface::export_state_interfaces()

{

  std::vector<hardware_interface::StateInterface> si;

  si.emplace_back(info_.joints[0].name,

    hardware_interface::HW_IF_POSITION, &hw_positions_[0]);

  si.emplace_back(info_.joints[0].name,

    hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]);

  si.emplace_back(info_.joints[1].name,

    hardware_interface::HW_IF_POSITION, &hw_positions_[1]);

  si.emplace_back(info_.joints[1].name,

    hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]);

  return si;

}



std::vector<hardware_interface::CommandInterface>

PoleBotHardwareInterface::export_command_interfaces()

{

  std::vector<hardware_interface::CommandInterface> ci;

  ci.emplace_back(info_.joints[0].name,

    hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);

  ci.emplace_back(info_.joints[1].name,

    hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]);

  return ci;

}



return_type PoleBotHardwareInterface::start()

{

  std::fill(hw_positions_.begin(),  hw_positions_.end(),  0.0);

  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  std::fill(hw_commands_.begin(),   hw_commands_.end(),   0.0);

  ticks_initialized_ = false;



  rclcpp::NodeOptions opts;

  opts.automatically_declare_parameters_from_overrides(true);

  hw_node_ = std::make_shared<rclcpp::Node>(

    "polebot_hw_node", opts);



  ticks_sub_ = hw_node_->create_subscription<std_msgs::msg::Int64MultiArray>(

    wheel_ticks_topic_, 10,

    [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg) {

      ticks_callback(msg);

    });



  cmd_pub_ = hw_node_->create_publisher<std_msgs::msg::Float64MultiArray>(

    wheel_cmd_topic_, 10);



  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor_->add_node(hw_node_);



  spinning_ = true;

  spin_thread_ = std::thread([this]() {

    while (spinning_.load()) {

      executor_->spin_some(std::chrono::milliseconds(10));

    }

  });



  RCLCPP_INFO(rclcpp::get_logger("PoleBotHW"),

    "Start: sub=%s pub=%s",

    wheel_ticks_topic_.c_str(), wheel_cmd_topic_.c_str());



  return return_type::OK;

}



return_type PoleBotHardwareInterface::stop()

{

  if (cmd_pub_) {

    std_msgs::msg::Float64MultiArray stop_msg;

    stop_msg.data = {0.0, 0.0};

    cmd_pub_->publish(stop_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  }



  spinning_ = false;

  if (spin_thread_.joinable()) { spin_thread_.join(); }



  executor_.reset();

  ticks_sub_.reset();

  cmd_pub_.reset();

  hw_node_.reset();



  RCLCPP_INFO(rclcpp::get_logger("PoleBotHW"), "Stop — motor di-nol.");

  return return_type::OK;

}



return_type PoleBotHardwareInterface::read()

{

  return return_type::OK;

}



return_type PoleBotHardwareInterface::write()

{

  double sl = 0.0, sr = 0.0;

  if (max_vel_rad_s_ > 0.0) {

    sl = (hw_commands_[0] / max_vel_rad_s_) * 100.0;

    sr = (hw_commands_[1] / max_vel_rad_s_) * 100.0;

  }

  sl = std::max(-100.0, std::min(100.0, sl));

  sr = std::max(-100.0, std::min(100.0, sr));



  if (cmd_pub_) {

    std_msgs::msg::Float64MultiArray msg;

    msg.data = {sr, sl};

    cmd_pub_->publish(msg);

  }

  return return_type::OK;

}



void PoleBotHardwareInterface::ticks_callback(

  const std_msgs::msg::Int64MultiArray::SharedPtr msg)

{

  if (msg->data.size() < 2) return;



  auto now = std::chrono::steady_clock::now();

  auto l   = static_cast<uint32_t>(msg->data[0]);

  auto r   = static_cast<uint32_t>(msg->data[1]);



  std::lock_guard<std::mutex> lock(state_mutex_);



  if (!ticks_initialized_) {

    last_ticks_[0]  = l;

    last_ticks_[1]  = r;

    last_tick_time_ = now;

    ticks_initialized_ = true;

    return;

  }



  double dt = std::chrono::duration<double>(

    now - last_tick_time_).count();

  if (dt <= 0.0) return;



  auto sdelta = [](uint32_t c, uint32_t p) -> int32_t {

    uint32_t d = (c - p) & 0xFFFFFFFFu;

    return (d < 0x80000000u) ?

      static_cast<int32_t>(d) :

      static_cast<int32_t>(

        static_cast<int64_t>(d) - 0x100000000LL);

  };



  const double rpt = (2.0 * M_PI) / (ticks_per_rev_ * gear_ratio_);

  double dl = static_cast<double>(sdelta(l, last_ticks_[0])) * rpt;

  double dr = static_cast<double>(sdelta(r, last_ticks_[1])) * rpt;



  hw_positions_[0]  += dl;

  hw_positions_[1]  += dr;

  hw_velocities_[0]  = dl / dt;

  hw_velocities_[1]  = dr / dt;



  last_ticks_[0]  = l;

  last_ticks_[1]  = r;

  last_tick_time_ = now;

}



}  // namespace amr_mp_hardware



PLUGINLIB_EXPORT_CLASS(

  amr_mp_hardware::PoleBotHardwareInterface,

  hardware_interface::SystemInterface)

