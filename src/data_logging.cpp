#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <crazyflie_interfaces/msg/position.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

// -------------------- helpers --------------------
static std::string expand_user(const std::string& path)
{
  if (!path.empty() && path[0] == '~') {
    const char* home = std::getenv("HOME");
    if (home) return std::string(home) + path.substr(1);
  }
  return path;
}

static std::string now_mmddhhmm()
{
  std::time_t t = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  char buf[64];
  std::strftime(buf, sizeof(buf), "%m%d%H%M", &tm);
  return std::string(buf);
}

// ===========================================================
class DataLoggingNode : public rclcpp::Node
{
public:
  // 기존 56 + state_body_vel(2) = 58
  static constexpr int kDataLen = 58;

  DataLoggingNode()
  : Node("data_logging")
  {
    RCLCPP_INFO(this->get_logger(), "data_logging node started!");

    // -------------------------
    // Parameters (CSV)
    // -------------------------
    csv_dir_ = this->declare_parameter<std::string>(
      "csv_dir", "~/hitl_ws/src/flying_pen/bag/logging"
    );
    csv_dir_ = expand_user(csv_dir_);

    flush_every_n_ = this->declare_parameter<int>("flush_every_n", 200);
    if (flush_every_n_ < 1) flush_every_n_ = 200;

    // -------------------------
    // CSV setup
    // -------------------------
    try {
      std::filesystem::create_directories(csv_dir_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to create csv_dir '%s': %s", csv_dir_.c_str(), e.what());
    }

    csv_path_ = (std::filesystem::path(csv_dir_) / (now_mmddhhmm() + ".csv")).string();
    csv_.open(csv_path_, std::ios::out | std::ios::trunc);

    if (!csv_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
    } else {
      write_csv_header();
      RCLCPP_INFO(get_logger(), "CSV logging enabled: %s", csv_path_.c_str());
    }

    // === publisher ===
    data_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/data_logging_msg", 10);

    // === subscribers ===

    // 0) ROS cmd_position
    sub_cmd_position_ = this->create_subscription<crazyflie_interfaces::msg::Position>(
      "/cf2/cmd_position", 10,
      std::bind(&DataLoggingNode::cmdPositionCallback, this, _1));

    // 1) Force input raw
    sub_input_force_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/cf_F_input_raw", 10,
      std::bind(&DataLoggingNode::forceCallback, this, _1));

    // 2) Force input scaled
    sub_input_scaled_force_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/cf_F_input_scaled", 10,
      std::bind(&DataLoggingNode::forceScaledCallback, this, _1));

    // 3) Pose
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cf2/pose", 10,
      std::bind(&DataLoggingNode::poseCallback, this, _1));

    // 4) Acc
    sub_acc_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/stateEstimate_acc", 10,
      std::bind(&DataLoggingNode::accCallback, this, _1));

    // 5) Velocity (EKF)
    sub_vel_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/stateEstimate_velocity", 10,
      std::bind(&DataLoggingNode::velCallback, this, _1));

    // 6) Voltage
    sub_voltage_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/cf_voltage", 10,
      std::bind(&DataLoggingNode::voltageCallback, this, _1));

    // 7) Final setpoint
    sub_setpoint_pos_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/cf_setpoint_pos", 10,
      std::bind(&DataLoggingNode::setpointPosCallback, this, _1));

    // 8) MOB
    sub_fext_mob_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/cf_Fext_MOB", 10,
      std::bind(&DataLoggingNode::fextMobCallback, this, _1));

    // 9) DOB
    sub_fext_dob_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/cf_Fext_DOB", 10,
      std::bind(&DataLoggingNode::fextDobCallback, this, _1));

    // 10) su_cmd_dbg
    sub_su_cmd_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/su_cmd_dbg", 10,
      std::bind(&DataLoggingNode::suCmdCallback, this, _1));

    // 11) vel_from_pos
    sub_vel_from_pos_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/vel_from_pos", 10,
      std::bind(&DataLoggingNode::velFromPosCallback, this, _1));

    // 12) kalman_q
    sub_kalman_q_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/kalman_q", 10,
      std::bind(&DataLoggingNode::kalmanQCallback, this, _1));

    // 13) kalman_qComp
    sub_kalman_qcomp_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/kalman_qComp", 10,
      std::bind(&DataLoggingNode::kalmanQCompCallback, this, _1));

    // 14) gyro feedback
    sub_gyro_feedback_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/gyro_feedback", 10,
      std::bind(&DataLoggingNode::gyroFeedbackCallback, this, _1));

    // 15) state_body_vel (posCtl.bodyVX, posCtl.bodyVY)
    sub_state_body_vel_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/state_body_vel", 10,
      std::bind(&DataLoggingNode::stateBodyVelCallback, this, _1));

    // 16) vel_des (posCtl.targetVX, targetVY, targetVZ)
    sub_vel_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/vel_des", 10,
      std::bind(&DataLoggingNode::velDesCallback, this, _1));

    // 17) att_des (controller.roll/pitch/yaw)
    sub_att_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/att_des", 10,
      std::bind(&DataLoggingNode::attDesCallback, this, _1));

    // 18) rate_des (controller.rollRate/pitchRate/yawRate)
    sub_rate_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/rate_des", 10,
      std::bind(&DataLoggingNode::rateDesCallback, this, _1));
  }

  ~DataLoggingNode() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

  void loopOnce()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(kDataLen);

    // 0~43 : 기존 44개
    // 44~46 : gyro_feedback (3)
    // 47~48 : state_body_vel (2)
    // 49~51 : vel_des (3)
    // 52~54 : att_des (3)
    // 55~57 : rate_des (3)

    // 0-2
    msg.data.push_back(global_force_input_(0));
    msg.data.push_back(global_force_input_(1));
    msg.data.push_back(global_force_input_(2));

    // 3-5
    msg.data.push_back(global_position_meas_(0));
    msg.data.push_back(global_position_meas_(1));
    msg.data.push_back(global_position_meas_(2));

    // 6-8
    msg.data.push_back(rpy_angle_meas_(0));
    msg.data.push_back(rpy_angle_meas_(1));
    msg.data.push_back(rpy_angle_meas_(2));

    // 9-11
    msg.data.push_back(global_acc_meas_(0));
    msg.data.push_back(global_acc_meas_(1));
    msg.data.push_back(global_acc_meas_(2));

    // 12-14
    msg.data.push_back(global_vel_meas_(0));
    msg.data.push_back(global_vel_meas_(1));
    msg.data.push_back(global_vel_meas_(2));

    // 15-16
    msg.data.push_back(vbat_raw_);
    msg.data.push_back(vbat_filtered_);

    // 17-20
    msg.data.push_back(cmd_position_(0));
    msg.data.push_back(cmd_position_(1));
    msg.data.push_back(cmd_position_(2));
    msg.data.push_back(cmd_yaw_deg_);

    // 21-23
    msg.data.push_back(global_force_input_scaled_(0));
    msg.data.push_back(global_force_input_scaled_(1));
    msg.data.push_back(global_force_input_scaled_(2));

    // 24-27
    msg.data.push_back(final_setpoint_pos_(0));
    msg.data.push_back(final_setpoint_pos_(1));
    msg.data.push_back(final_setpoint_pos_(2));
    msg.data.push_back(final_setpoint_yaw_deg_);

    // 28
    msg.data.push_back(su_cmd_fx_);

    // 29-31
    msg.data.push_back(world_Fext_MOB_(0));
    msg.data.push_back(world_Fext_MOB_(1));
    msg.data.push_back(world_Fext_MOB_(2));

    // 32-34
    msg.data.push_back(world_Fext_DOB_(0));
    msg.data.push_back(world_Fext_DOB_(1));
    msg.data.push_back(world_Fext_DOB_(2));

    // 35-37
    msg.data.push_back(vel_from_pos_(0));
    msg.data.push_back(vel_from_pos_(1));
    msg.data.push_back(vel_from_pos_(2));

    // 38-40
    msg.data.push_back(rpy_kalman_(0));
    msg.data.push_back(rpy_kalman_(1));
    msg.data.push_back(rpy_kalman_(2));

    // 41-43
    msg.data.push_back(rpy_comp_(0));
    msg.data.push_back(rpy_comp_(1));
    msg.data.push_back(rpy_comp_(2));

    // 44-46 gyro feedback
    msg.data.push_back(gyro_fb_(0));
    msg.data.push_back(gyro_fb_(1));
    msg.data.push_back(gyro_fb_(2));

    // 47-48 state_body_vel (actual body yaw-aligned VX,VY)
    msg.data.push_back(state_body_vel_(0));
    msg.data.push_back(state_body_vel_(1));

    // 49-51 vel_des (desired targetVX,targetVY,targetVZ)
    msg.data.push_back(vel_des_(0));
    msg.data.push_back(vel_des_(1));
    msg.data.push_back(vel_des_(2));

    // 52-54 att_des
    msg.data.push_back(att_des_(0));
    msg.data.push_back(att_des_(1));
    msg.data.push_back(att_des_(2));

    // 55-57 rate_des
    msg.data.push_back(rate_des_(0));
    msg.data.push_back(rate_des_(1));
    msg.data.push_back(rate_des_(2));

    if (msg.data.size() < static_cast<size_t>(kDataLen)) {
      const size_t old = msg.data.size();
      msg.data.resize(kDataLen, std::numeric_limits<double>::quiet_NaN());
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "data_logging_msg had %zu (<%d). padded to %d with NaN.", old, kDataLen, kDataLen);
    } else if (msg.data.size() > static_cast<size_t>(kDataLen)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "data_logging_msg had %zu (>%d). truncating to %d.", msg.data.size(), kDataLen, kDataLen);
      msg.data.resize(kDataLen);
    }

    data_pub_->publish(msg);
    log_csv_row(msg);

    have_published_once_ = true;
  }

private:
  // ---------------- CSV ----------------
  void write_csv_header()
  {
    csv_ << "t_sec";
    for (int i = 0; i < kDataLen; ++i) csv_ << ",d" << i;
    csv_ << ",validity_bitmask\n";
    csv_.flush();
  }

  void log_csv_row(const std_msgs::msg::Float64MultiArray& msg)
  {
    if (!csv_.is_open()) return;

    uint32_t mask = 0u;
    const bool size_ok = (msg.data.size() == static_cast<size_t>(kDataLen));
    if (size_ok) mask |= (1u << 0);
    if (have_published_once_) mask |= (1u << 1);
    mask |= (1u << 2);

    const double t = this->get_clock()->now().seconds();

    csv_ << std::setprecision(10) << std::fixed;
    csv_ << t;

    for (int i = 0; i < kDataLen; ++i) {
      double v = std::numeric_limits<double>::quiet_NaN();
      if (i < static_cast<int>(msg.data.size())) v = msg.data[i];
      csv_ << "," << v;
    }
    csv_ << "," << static_cast<uint64_t>(mask) << "\n";

    ++csv_line_count_;
    if (csv_line_count_ <= 20 || (csv_line_count_ % static_cast<uint64_t>(flush_every_n_) == 0)) {
      csv_.flush();
    }
  }

  // ================= callbacks =================
  void forceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      global_force_input_(0) = msg->values[0];
      global_force_input_(1) = msg->values[1];
      global_force_input_(2) = msg->values[2];
    }
  }

  void forceScaledCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      global_force_input_scaled_(0) = msg->values[0];
      global_force_input_scaled_(1) = msg->values[1];
      global_force_input_scaled_(2) = msg->values[2];
    }
  }

  void fextMobCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      world_Fext_MOB_(0) = msg->values[0];
      world_Fext_MOB_(1) = msg->values[1];
      world_Fext_MOB_(2) = msg->values[2];
    }
  }

  void fextDobCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      world_Fext_DOB_(0) = msg->values[0];
      world_Fext_DOB_(1) = msg->values[1];
      world_Fext_DOB_(2) = msg->values[2];
    }
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    global_position_meas_(0) = msg->pose.position.x;
    global_position_meas_(1) = msg->pose.position.y;
    global_position_meas_(2) = msg->pose.position.z;

    tf2::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
    q.normalize();

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    rpy_angle_meas_(0) = roll;
    rpy_angle_meas_(1) = pitch;
    rpy_angle_meas_(2) = yaw;
  }

  void accCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      global_acc_meas_(0) = 3.0 * msg->values[0];
      global_acc_meas_(1) = 3.0 * msg->values[1];
      global_acc_meas_(2) = 3.0 * msg->values[2];
    }
  }

  void velCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      global_vel_meas_(0) = msg->values[0];
      global_vel_meas_(1) = msg->values[1];
      global_vel_meas_(2) = msg->values[2];
    }
  }

  void voltageCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 1) vbat_raw_ = msg->values[0];
    if (msg->values.size() >= 2) vbat_filtered_ = msg->values[1];
  }

  void cmdPositionCallback(const crazyflie_interfaces::msg::Position::SharedPtr msg)
  {
    cmd_position_(0) = msg->x;
    cmd_position_(1) = msg->y;
    cmd_position_(2) = msg->z;
    cmd_yaw_deg_     = msg->yaw;
  }

  void setpointPosCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 4) {
      final_setpoint_pos_(0)  = msg->values[0];
      final_setpoint_pos_(1)  = msg->values[1];
      final_setpoint_pos_(2)  = msg->values[2];
      final_setpoint_yaw_deg_ = msg->values[3];
    }
  }

  void suCmdCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 2) {
      su_cmd_fx_ = msg->values[1];
    }
  }

  void velFromPosCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      vel_from_pos_(0) = msg->values[0];
      vel_from_pos_(1) = msg->values[1];
      vel_from_pos_(2) = msg->values[2];
    }
  }

  void kalmanQCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 4) {
      const double w = msg->values[0];
      const double x = msg->values[1];
      const double y = msg->values[2];
      const double z = msg->values[3];

      tf2::Quaternion q(x, y, z, w);
      q.normalize();

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      rpy_kalman_(0) = roll;
      rpy_kalman_(1) = pitch;
      rpy_kalman_(2) = yaw;
    }
  }

  void kalmanQCompCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 4) {
      const double w = msg->values[0];
      const double x = msg->values[1];
      const double y = msg->values[2];
      const double z = msg->values[3];

      tf2::Quaternion q(x, y, z, w);
      q.normalize();

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      rpy_comp_(0) = roll;
      rpy_comp_(1) = pitch;
      rpy_comp_(2) = yaw;
    }
  }

  void gyroFeedbackCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      gyro_fb_(0) = msg->values[0];
      gyro_fb_(1) = msg->values[1];
      gyro_fb_(2) = msg->values[2];
    }
  }

  void stateBodyVelCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 2) {
      state_body_vel_(0) = msg->values[0];
      state_body_vel_(1) = msg->values[1];
    }
  }

  void velDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      vel_des_(0) = msg->values[0];
      vel_des_(1) = msg->values[1];
      vel_des_(2) = msg->values[2];
    }
  }

  void attDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      att_des_(0) = msg->values[0];
      att_des_(1) = msg->values[1];
      att_des_(2) = msg->values[2];
    }
  }

  void rateDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      rate_des_(0) = msg->values[0];
      rate_des_(1) = msg->values[1];
      rate_des_(2) = msg->values[2];
    }
  }

  // ================= members =================

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr data_pub_;

  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_input_scaled_force_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_input_force_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_fext_mob_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_fext_dob_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr           sub_pose_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_acc_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_vel_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_voltage_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr       sub_cmd_position_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_setpoint_pos_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_su_cmd_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_vel_from_pos_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_kalman_q_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_kalman_qcomp_;

  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_gyro_feedback_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_state_body_vel_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_vel_des_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_att_des_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_rate_des_;

  Eigen::Vector3d global_force_input_        = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_force_input_scaled_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_position_meas_      = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy_angle_meas_            = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_acc_meas_           = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_vel_meas_           = Eigen::Vector3d::Zero();
  Eigen::Vector3d final_setpoint_pos_        = Eigen::Vector3d::Zero();
  Eigen::Vector3d world_Fext_MOB_            = Eigen::Vector3d::Zero();
  Eigen::Vector3d world_Fext_DOB_            = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_from_pos_              = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy_kalman_                = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy_comp_                  = Eigen::Vector3d::Zero();

  Eigen::Vector3d gyro_fb_       = Eigen::Vector3d::Zero();
  Eigen::Vector2d state_body_vel_ = Eigen::Vector2d::Zero();
  Eigen::Vector3d vel_des_       = Eigen::Vector3d::Zero();
  Eigen::Vector3d att_des_       = Eigen::Vector3d::Zero();
  Eigen::Vector3d rate_des_      = Eigen::Vector3d::Zero();

  double final_setpoint_yaw_deg_ = 0.0;
  double vbat_raw_              = 0.0;
  double vbat_filtered_         = 0.0;
  Eigen::Vector3d cmd_position_ = Eigen::Vector3d::Zero();
  double cmd_yaw_deg_           = 0.0;
  double su_cmd_fx_             = 0.0;

  std::string csv_dir_;
  std::string csv_path_;
  std::ofstream csv_;
  uint64_t csv_line_count_{0};
  int flush_every_n_{200};
  bool have_published_once_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DataLoggingNode>();

  rclcpp::Rate rate(100.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->loopOnce();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
