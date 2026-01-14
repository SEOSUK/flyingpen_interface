// data_logging_specific_all.cpp
// 목적: firmware_logging (YAML: default_topics + custom_topics) 기반으로
//       모든 값을 CSV에 기록 + Float64MultiArray로 publish
//
// [YAML 기준 포함 토픽]
// default_topics: pose, status
// custom_topics : cf_setpoint_pos, su_cmd_dbg, vel_from_pos,
//                 stateEstimate_velocity, stateEstimate_acc, state_body_vel,
//                 gyro_feedback, vel_des, att_des, rate_des,
//                 kalman_att_q, kalman_att_qComp, kalman_att_err
//
// (YAML에 없는 토픽: kalman_contact_diag, kalman_acc_ext, kalman_mode_flags 등은 삭제)

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
#include <array>
#include <vector>

#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <crazyflie_interfaces/msg/status.hpp>  // /cf_x/status

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

static inline double qnan() { return std::numeric_limits<double>::quiet_NaN(); }

// ===========================================================
class DataLoggingSpecificNode : public rclcpp::Node
{
public:
  // ====== Packed output layout (Float64MultiArray & CSV columns) ======
  //
  // 0.. 5   pose:                x y z roll pitch yaw
  // 6       status:              battery_voltage
  //
  // 7.. 10  cf_setpoint_pos:      x y z yaw_sp
  // 11..12  su_cmd_dbg:           use_vel_mode cmd_fx
  // 13..15  vel_from_pos:         vx vy vz
  //
  // 16..18  stateEstimate_velocity: vx vy vz
  // 19..21  stateEstimate_acc:      ax ay az
  //
  // 22..23  state_body_vel:       bodyVX bodyVY
  // 24..26  gyro_feedback:        gx gy gz
  // 27..29  vel_des:              targetVX targetVY targetVZ
  // 30..32  att_des:              roll pitch yaw
  // 33..35  rate_des:             rollRate pitchRate yawRate
  //
  // 36..39  kalman_att_q:         q0 q1 q2 q3
  // 40..43  kalman_att_qComp:     qComp0 qComp1 qComp2 qComp3
  // 44..46  kalman_att_err:       stateD0 stateD1 stateD2
  //
  static constexpr int kDataLen = 47;

  DataLoggingSpecificNode()
  : Node("data_logging_specific")
  {
    RCLCPP_INFO(this->get_logger(), "data_logging_specific node started!");

    // -------------------------
    // Parameters
    // -------------------------
    csv_dir_ = expand_user(this->declare_parameter<std::string>(
      "csv_dir", "~/hitl_ws/src/flying_pen/bag/logging_specific"
    ));
    flush_every_n_ = this->declare_parameter<int>("flush_every_n", 200);
    if (flush_every_n_ < 1) flush_every_n_ = 200;

    publish_topic_ = this->declare_parameter<std::string>(
      "publish_topic", "/data_logging_specific_msg"
    );
    cf_ns_ = this->declare_parameter<std::string>("cf_ns", "/cf2");

    loop_hz_ = this->declare_parameter<double>("loop_hz", 100.0);
    stale_warn_sec_ = this->declare_parameter<double>("stale_warn_sec", 0.5);
    stale_fail_sec_ = this->declare_parameter<double>("stale_fail_sec", 2.0);

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
    data_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(publish_topic_, 10);

    // === subscribers ===
    // default_topics: pose, status
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      cf_ns_ + "/pose", 10,
      std::bind(&DataLoggingSpecificNode::poseCallback, this, _1));

    sub_status_ = this->create_subscription<crazyflie_interfaces::msg::Status>(
      cf_ns_ + "/status", 10,
      std::bind(&DataLoggingSpecificNode::statusCallback, this, _1));

    // custom_topics (YAML)
    sub_cf_setpoint_pos_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_setpoint_pos", 10,
      std::bind(&DataLoggingSpecificNode::cfSetpointPosCallback, this, _1));

    sub_su_cmd_dbg_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/su_cmd_dbg", 10,
      std::bind(&DataLoggingSpecificNode::suCmdDbgCallback, this, _1));

    sub_vel_from_pos_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/vel_from_pos", 10,
      std::bind(&DataLoggingSpecificNode::velFromPosCallback, this, _1));

    sub_stateEstimate_velocity_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/stateEstimate_velocity", 10,
      std::bind(&DataLoggingSpecificNode::stateEstimateVelocityCallback, this, _1));

    sub_stateEstimate_acc_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/stateEstimate_acc", 10,
      std::bind(&DataLoggingSpecificNode::stateEstimateAccCallback, this, _1));

    sub_state_body_vel_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/state_body_vel", 10,
      std::bind(&DataLoggingSpecificNode::stateBodyVelCallback, this, _1));

    sub_gyro_feedback_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/gyro_feedback", 10,
      std::bind(&DataLoggingSpecificNode::gyroFeedbackCallback, this, _1));

    sub_vel_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/vel_des", 10,
      std::bind(&DataLoggingSpecificNode::velDesCallback, this, _1));

    sub_att_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/att_des", 10,
      std::bind(&DataLoggingSpecificNode::attDesCallback, this, _1));

    sub_rate_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/rate_des", 10,
      std::bind(&DataLoggingSpecificNode::rateDesCallback, this, _1));

    sub_kalman_att_q_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/kalman_att_q", 10,
      std::bind(&DataLoggingSpecificNode::kalmanAttQCallback, this, _1));

    sub_kalman_att_qComp_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/kalman_att_qComp", 10,
      std::bind(&DataLoggingSpecificNode::kalmanAttQCompCallback, this, _1));

    sub_kalman_att_err_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/kalman_att_err", 10,
      std::bind(&DataLoggingSpecificNode::kalmanAttErrCallback, this, _1));
  }

  ~DataLoggingSpecificNode() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

  double loop_hz() const { return loop_hz_; }

  void loopOnce()
  {
    const double t = now_sec();

    // ---------- pack ----------
    std_msgs::msg::Float64MultiArray out;
    out.data.reserve(kDataLen);

    // 0..5 pose
    push3(out, pose_xyz_);
    push3(out, pose_rpy_);

    // 6 status
    out.data.push_back(status_batt_v_);

    // 7..10 cf_setpoint_pos
    push4(out, setpoint_xyzyaw_);

    // 11..12 su_cmd_dbg
    out.data.push_back(su_cmd_dbg_[0]);
    out.data.push_back(su_cmd_dbg_[1]);

    // 13..15 vel_from_pos
    push3(out, vel_from_pos_);

    // 16..18 stateEstimate_velocity
    push3(out, stateEstimate_vel_);

    // 19..21 stateEstimate_acc
    push3(out, stateEstimate_acc_);

    // 22..23 state_body_vel
    push2(out, state_body_vel_);

    // 24..26 gyro_feedback
    push3(out, gyro_feedback_);

    // 27..29 vel_des
    push3(out, vel_des_);

    // 30..32 att_des
    push3(out, att_des_);

    // 33..35 rate_des
    push3(out, rate_des_);

    // 36..39 kalman_att_q
    push4(out, kalman_att_q_);

    // 40..43 kalman_att_qComp
    push4(out, kalman_att_qComp_);

    // 44..46 kalman_att_err
    push3(out, kalman_att_err_);

    // size guard
    if (out.data.size() != static_cast<size_t>(kDataLen)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "Packed length mismatch: got %zu, expected %d", out.data.size(), kDataLen);
      out.data.resize(kDataLen, qnan());
    }

    // ---------- health / stale check ----------
    const uint64_t mask = build_validity_mask(t);

    // ---------- publish + csv ----------
    data_pub_->publish(out);
    log_csv_row(t, out, mask);

    have_published_once_ = true;
    warn_if_stale(t);
  }

private:
  // ===== packing helpers =====
  static void push2(std_msgs::msg::Float64MultiArray& m, const std::array<double,2>& a) {
    m.data.push_back(a[0]); m.data.push_back(a[1]);
  }
  static void push3(std_msgs::msg::Float64MultiArray& m, const std::array<double,3>& a) {
    m.data.push_back(a[0]); m.data.push_back(a[1]); m.data.push_back(a[2]);
  }
  static void push4(std_msgs::msg::Float64MultiArray& m, const std::array<double,4>& a) {
    m.data.push_back(a[0]); m.data.push_back(a[1]); m.data.push_back(a[2]); m.data.push_back(a[3]);
  }

  // ===== CSV =====
  void write_csv_header()
  {
    csv_ << "t_sec";

    for (int i = 0; i < kDataLen; ++i) {
      csv_ << "," << col_name(i);
    }

    // ages
    csv_ << ",age_pose"
         << ",age_status"
         << ",age_setpoint"
         << ",age_su_cmd_dbg"
         << ",age_vel_from_pos"
         << ",age_stateEst_vel"
         << ",age_stateEst_acc"
         << ",age_state_body_vel"
         << ",age_gyro_feedback"
         << ",age_vel_des"
         << ",age_att_des"
         << ",age_rate_des"
         << ",age_kalman_att_q"
         << ",age_kalman_att_qComp"
         << ",age_kalman_att_err";

    csv_ << ",validity_bitmask\n";
    csv_.flush();
  }

  std::string col_name(int idx) const
  {
    switch (idx) {
      // pose
      case 0: return "pose_x"; case 1: return "pose_y"; case 2: return "pose_z";
      case 3: return "pose_roll"; case 4: return "pose_pitch"; case 5: return "pose_yaw";

      // status
      case 6: return "status_battery_voltage";

      // cf_setpoint_pos
      case 7: return "sp_x"; case 8: return "sp_y"; case 9: return "sp_z"; case 10: return "sp_yaw_sp";

      // su_cmd_dbg
      case 11: return "su_use_vel_mode"; case 12: return "su_cmd_fx";

      // vel_from_pos
      case 13: return "velFromPos_vx"; case 14: return "velFromPos_vy"; case 15: return "velFromPos_vz";

      // stateEstimate_velocity
      case 16: return "est_vx"; case 17: return "est_vy"; case 18: return "est_vz";

      // stateEstimate_acc
      case 19: return "est_ax"; case 20: return "est_ay"; case 21: return "est_az";

      // state_body_vel
      case 22: return "body_vx"; case 23: return "body_vy";

      // gyro_feedback
      case 24: return "gyro_x"; case 25: return "gyro_y"; case 26: return "gyro_z";

      // vel_des
      case 27: return "velDes_vx"; case 28: return "velDes_vy"; case 29: return "velDes_vz";

      // att_des
      case 30: return "attDes_roll"; case 31: return "attDes_pitch"; case 32: return "attDes_yaw";

      // rate_des
      case 33: return "rateDes_p"; case 34: return "rateDes_q"; case 35: return "rateDes_r";

      // kalman_att_q
      case 36: return "kal_q0"; case 37: return "kal_q1"; case 38: return "kal_q2"; case 39: return "kal_q3";

      // kalman_att_qComp
      case 40: return "kal_qComp0"; case 41: return "kal_qComp1"; case 42: return "kal_qComp2"; case 43: return "kal_qComp3";

      // kalman_att_err
      case 44: return "kal_stateD0"; case 45: return "kal_stateD1"; case 46: return "kal_stateD2";

      default: return "d" + std::to_string(idx);
    }
  }

  void log_csv_row(double t, const std_msgs::msg::Float64MultiArray& msg, uint64_t mask)
  {
    if (!csv_.is_open()) return;

    csv_ << std::setprecision(10) << std::fixed;
    csv_ << t;

    for (int i = 0; i < kDataLen; ++i) {
      const double v = (i < static_cast<int>(msg.data.size())) ? msg.data[i] : qnan();
      csv_ << "," << v;
    }

    // ages
    csv_ << "," << age_sec(t, t_last_pose_)
         << "," << age_sec(t, t_last_status_)
         << "," << age_sec(t, t_last_setpoint_)
         << "," << age_sec(t, t_last_su_cmd_dbg_)
         << "," << age_sec(t, t_last_vel_from_pos_)
         << "," << age_sec(t, t_last_stateEst_vel_)
         << "," << age_sec(t, t_last_stateEst_acc_)
         << "," << age_sec(t, t_last_state_body_vel_)
         << "," << age_sec(t, t_last_gyro_feedback_)
         << "," << age_sec(t, t_last_vel_des_)
         << "," << age_sec(t, t_last_att_des_)
         << "," << age_sec(t, t_last_rate_des_)
         << "," << age_sec(t, t_last_kalman_att_q_)
         << "," << age_sec(t, t_last_kalman_att_qComp_)
         << "," << age_sec(t, t_last_kalman_att_err_);

    csv_ << "," << static_cast<unsigned long long>(mask) << "\n";

    ++csv_line_count_;
    if (csv_line_count_ <= 20 || (csv_line_count_ % static_cast<uint64_t>(flush_every_n_) == 0)) {
      csv_.flush();
    }
  }

  static double age_sec(double now, double last)
  {
    if (!std::isfinite(last)) return std::numeric_limits<double>::infinity();
    const double a = now - last;
    return (a < 0.0) ? 0.0 : a;
  }

  // ===== validity + stale logic =====
  uint64_t build_validity_mask(double t)
  {
    // bit 정의:
    // 0: packed size ok
    // 1: published at least once
    // 2.. : topic fresh (< stale_fail_sec_)
    //
    // 2  pose
    // 3  status
    // 4  cf_setpoint_pos
    // 5  su_cmd_dbg
    // 6  vel_from_pos
    // 7  stateEstimate_velocity
    // 8  stateEstimate_acc
    // 9  state_body_vel
    // 10 gyro_feedback
    // 11 vel_des
    // 12 att_des
    // 13 rate_des
    // 14 kalman_att_q
    // 15 kalman_att_qComp
    // 16 kalman_att_err

    uint64_t m = 0ull;

    m |= (1ull << 0);
    if (have_published_once_) m |= (1ull << 1);

    if (age_sec(t, t_last_pose_) < stale_fail_sec_)               m |= (1ull << 2);
    if (age_sec(t, t_last_status_) < stale_fail_sec_)             m |= (1ull << 3);

    if (age_sec(t, t_last_setpoint_) < stale_fail_sec_)           m |= (1ull << 4);
    if (age_sec(t, t_last_su_cmd_dbg_) < stale_fail_sec_)         m |= (1ull << 5);
    if (age_sec(t, t_last_vel_from_pos_) < stale_fail_sec_)       m |= (1ull << 6);

    if (age_sec(t, t_last_stateEst_vel_) < stale_fail_sec_)       m |= (1ull << 7);
    if (age_sec(t, t_last_stateEst_acc_) < stale_fail_sec_)       m |= (1ull << 8);

    if (age_sec(t, t_last_state_body_vel_) < stale_fail_sec_)     m |= (1ull << 9);
    if (age_sec(t, t_last_gyro_feedback_) < stale_fail_sec_)      m |= (1ull << 10);

    if (age_sec(t, t_last_vel_des_) < stale_fail_sec_)            m |= (1ull << 11);
    if (age_sec(t, t_last_att_des_) < stale_fail_sec_)            m |= (1ull << 12);
    if (age_sec(t, t_last_rate_des_) < stale_fail_sec_)           m |= (1ull << 13);

    if (age_sec(t, t_last_kalman_att_q_) < stale_fail_sec_)       m |= (1ull << 14);
    if (age_sec(t, t_last_kalman_att_qComp_) < stale_fail_sec_)   m |= (1ull << 15);
    if (age_sec(t, t_last_kalman_att_err_) < stale_fail_sec_)     m |= (1ull << 16);

    return m;
  }

  void warn_if_stale(double t)
  {
    auto warn_topic = [&](const char* name, double age) {
      if (age > stale_warn_sec_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Topic stale: %-22s age=%.3fs (warn>%.3fs, fail>%.3fs)",
          name, age, stale_warn_sec_, stale_fail_sec_);
      }
    };

    warn_topic("pose",                  age_sec(t, t_last_pose_));
    warn_topic("status",                age_sec(t, t_last_status_));

    warn_topic("cf_setpoint_pos",       age_sec(t, t_last_setpoint_));
    warn_topic("su_cmd_dbg",            age_sec(t, t_last_su_cmd_dbg_));
    warn_topic("vel_from_pos",          age_sec(t, t_last_vel_from_pos_));

    warn_topic("stateEstimate_velocity",age_sec(t, t_last_stateEst_vel_));
    warn_topic("stateEstimate_acc",     age_sec(t, t_last_stateEst_acc_));

    warn_topic("state_body_vel",        age_sec(t, t_last_state_body_vel_));
    warn_topic("gyro_feedback",         age_sec(t, t_last_gyro_feedback_));

    warn_topic("vel_des",               age_sec(t, t_last_vel_des_));
    warn_topic("att_des",               age_sec(t, t_last_att_des_));
    warn_topic("rate_des",              age_sec(t, t_last_rate_des_));

    warn_topic("kalman_att_q",          age_sec(t, t_last_kalman_att_q_));
    warn_topic("kalman_att_qComp",      age_sec(t, t_last_kalman_att_qComp_));
    warn_topic("kalman_att_err",        age_sec(t, t_last_kalman_att_err_));
  }

  // ================= callbacks =================
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pose_xyz_[0] = msg->pose.position.x;
    pose_xyz_[1] = msg->pose.position.y;
    pose_xyz_[2] = msg->pose.position.z;

    tf2::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
    q.normalize();

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pose_rpy_[0] = roll;
    pose_rpy_[1] = pitch;
    pose_rpy_[2] = yaw;

    t_last_pose_ = now_sec();
  }

  void statusCallback(const crazyflie_interfaces::msg::Status::SharedPtr msg)
  {
    // 최소 요구: 배터리 전압만 기록
    status_batt_v_ = msg->battery_voltage;
    t_last_status_ = now_sec();
  }

  void cfSetpointPosCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 4) {
      setpoint_xyzyaw_[0] = msg->values[0];
      setpoint_xyzyaw_[1] = msg->values[1];
      setpoint_xyzyaw_[2] = msg->values[2];
      setpoint_xyzyaw_[3] = msg->values[3];
      t_last_setpoint_ = now_sec();
    }
  }

  void suCmdDbgCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 2) {
      su_cmd_dbg_[0] = msg->values[0];
      su_cmd_dbg_[1] = msg->values[1];
      t_last_su_cmd_dbg_ = now_sec();
    }
  }

  void velFromPosCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      vel_from_pos_[0] = msg->values[0];
      vel_from_pos_[1] = msg->values[1];
      vel_from_pos_[2] = msg->values[2];
      t_last_vel_from_pos_ = now_sec();
    }
  }

  void stateEstimateVelocityCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      stateEstimate_vel_[0] = msg->values[0];
      stateEstimate_vel_[1] = msg->values[1];
      stateEstimate_vel_[2] = msg->values[2];
      t_last_stateEst_vel_ = now_sec();
    }
  }

  void stateEstimateAccCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      stateEstimate_acc_[0] = msg->values[0];
      stateEstimate_acc_[1] = msg->values[1];
      stateEstimate_acc_[2] = msg->values[2];
      t_last_stateEst_acc_ = now_sec();
    }
  }

  void stateBodyVelCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 2) {
      state_body_vel_[0] = msg->values[0];
      state_body_vel_[1] = msg->values[1];
      t_last_state_body_vel_ = now_sec();
    }
  }

  void gyroFeedbackCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      gyro_feedback_[0] = msg->values[0];
      gyro_feedback_[1] = msg->values[1];
      gyro_feedback_[2] = msg->values[2];
      t_last_gyro_feedback_ = now_sec();
    }
  }

  void velDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      vel_des_[0] = msg->values[0];
      vel_des_[1] = msg->values[1];
      vel_des_[2] = msg->values[2];
      t_last_vel_des_ = now_sec();
    }
  }

  void attDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      att_des_[0] = msg->values[0];
      att_des_[1] = msg->values[1];
      att_des_[2] = msg->values[2];
      t_last_att_des_ = now_sec();
    }
  }

  void rateDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      rate_des_[0] = msg->values[0];
      rate_des_[1] = msg->values[1];
      rate_des_[2] = msg->values[2];
      t_last_rate_des_ = now_sec();
    }
  }

  void kalmanAttQCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 4) {
      kalman_att_q_[0] = msg->values[0];
      kalman_att_q_[1] = msg->values[1];
      kalman_att_q_[2] = msg->values[2];
      kalman_att_q_[3] = msg->values[3];
      t_last_kalman_att_q_ = now_sec();
    }
  }

  void kalmanAttQCompCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 4) {
      kalman_att_qComp_[0] = msg->values[0];
      kalman_att_qComp_[1] = msg->values[1];
      kalman_att_qComp_[2] = msg->values[2];
      kalman_att_qComp_[3] = msg->values[3];
      t_last_kalman_att_qComp_ = now_sec();
    }
  }

  void kalmanAttErrCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      kalman_att_err_[0] = msg->values[0];
      kalman_att_err_[1] = msg->values[1];
      kalman_att_err_[2] = msg->values[2];
      t_last_kalman_att_err_ = now_sec();
    }
  }

  double now_sec() { return this->get_clock()->now().seconds(); }

  // ================= members =================
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr data_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr           sub_pose_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Status>::SharedPtr        sub_status_;

  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_cf_setpoint_pos_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_su_cmd_dbg_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_vel_from_pos_;

  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_stateEstimate_velocity_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_stateEstimate_acc_;

  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_state_body_vel_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_gyro_feedback_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_vel_des_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_att_des_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_rate_des_;

  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_kalman_att_q_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_kalman_att_qComp_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_kalman_att_err_;

  // Params / io
  std::string csv_dir_;
  std::string csv_path_;
  std::ofstream csv_;
  uint64_t csv_line_count_{0};
  int flush_every_n_{200};
  bool have_published_once_{false};

  std::string publish_topic_;
  std::string cf_ns_;

  double loop_hz_{100.0};
  double stale_warn_sec_{0.5};
  double stale_fail_sec_{2.0};

  // Stored values (NaN init)
  std::array<double,3> pose_xyz_ = {qnan(), qnan(), qnan()};
  std::array<double,3> pose_rpy_ = {qnan(), qnan(), qnan()};

  double status_batt_v_ = qnan();

  std::array<double,4> setpoint_xyzyaw_ = {qnan(), qnan(), qnan(), qnan()};
  std::array<double,2> su_cmd_dbg_      = {qnan(), qnan()};
  std::array<double,3> vel_from_pos_    = {qnan(), qnan(), qnan()};

  std::array<double,3> stateEstimate_vel_ = {qnan(), qnan(), qnan()};
  std::array<double,3> stateEstimate_acc_ = {qnan(), qnan(), qnan()};

  std::array<double,2> state_body_vel_  = {qnan(), qnan()};
  std::array<double,3> gyro_feedback_   = {qnan(), qnan(), qnan()};
  std::array<double,3> vel_des_         = {qnan(), qnan(), qnan()};
  std::array<double,3> att_des_         = {qnan(), qnan(), qnan()};
  std::array<double,3> rate_des_        = {qnan(), qnan(), qnan()};

  std::array<double,4> kalman_att_q_      = {qnan(), qnan(), qnan(), qnan()};
  std::array<double,4> kalman_att_qComp_  = {qnan(), qnan(), qnan(), qnan()};
  std::array<double,3> kalman_att_err_    = {qnan(), qnan(), qnan()};

  // per-topic last update times (seconds)
  double t_last_pose_ = qnan();
  double t_last_status_ = qnan();

  double t_last_setpoint_ = qnan();
  double t_last_su_cmd_dbg_ = qnan();
  double t_last_vel_from_pos_ = qnan();

  double t_last_stateEst_vel_ = qnan();
  double t_last_stateEst_acc_ = qnan();

  double t_last_state_body_vel_ = qnan();
  double t_last_gyro_feedback_ = qnan();
  double t_last_vel_des_ = qnan();
  double t_last_att_des_ = qnan();
  double t_last_rate_des_ = qnan();

  double t_last_kalman_att_q_ = qnan();
  double t_last_kalman_att_qComp_ = qnan();
  double t_last_kalman_att_err_ = qnan();
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DataLoggingSpecificNode>();

  const double hz = node->loop_hz();
  rclcpp::Rate rate(hz);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->loopOnce();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
