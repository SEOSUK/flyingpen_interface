#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>  // M_PI

#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class RvizVisual : public rclcpp::Node
{
public:
  RvizVisual()
  : Node("rviz_visual"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
  {
    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10),
      rmw_qos_profile_sensor_data);

    sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/data_logging_msg",
      qos,
      std::bind(&RvizVisual::dataCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      10ms, std::bind(&RvizVisual::publishTfTimer, this));

    mob_force_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/force_mob_marker", 10);
    dob_force_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/force_dob_marker", 10);
    acc_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/acc_marker", 10);
    vel_pos_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/vel_pos_marker", 10);
    vel_ekf_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/vel_ekf_marker", 10);

    wall_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/wall_marker", 10);

    pos_.setZero();
    rpy_meas_.setZero();
    rpy_kalman_.setZero();
    rpy_comp_.setZero();

    world_Fext_MOB_.setZero();
    world_Fext_DOB_.setZero();
    global_force_input_.setZero();
    global_force_input_scaled_.setZero();
    global_acc_meas_.setZero();
    global_vel_meas_.setZero();
    vel_from_pos_.setZero();
    cmd_pos_.setZero();
    final_pos_command_.setZero();

    RCLCPP_INFO(this->get_logger(), "rviz_visual started. subscribing /data_logging_msg");
  }

private:
  void dataCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // 최소 길이: 44 (0~43)
    if (msg->data.size() < 44) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "msg size too small (%zu), expected >= 44", msg->data.size());
      return;
    }

    // ===== 인덱스 정의 =====
    //  0- 2 : global_force_input_        (Fx, Fy, Fz)      [World, raw]
    //  3- 5 : global_position_meas_      (px, py, pz)      [World]
    //  6- 8 : rpy_angle_meas_            (roll, pitch, yaw) [rad] (from pose)
    //  9-11 : global_acc_meas_           (ax, ay, az)      [World]
    // 12-14 : global_vel_meas_           (vx, vy, vz)      [World, EKF]
    // 15-16 : vbat_raw_, vbat_filtered_  [V]
    // 17-19 : cmd_position_              (x,y,z)           [ROS cmd_position]
    // 20    : cmd_yaw_deg_               [deg]
    // 21-23 : global_force_input_scaled_ (Fx, Fy, Fz)      [World, scaled]
    // 24-26 : final_setpoint_pos_        (x,y,z)           [ctrl target in FW]
    // 27    : final_setpoint_yaw_deg_    [deg]
    // 28    : su_cmd_fx_                 [N or arb]
    // 29-31 : world_Fext_MOB_            [N]
    // 32-34 : world_Fext_DOB_            [N]
    // 35-37 : vel_from_pos_              [m/s]
    // 38-40 : rpy_kalman_                 [rad]
    // 41-43 : rpy_comp_                   [rad]

    // --- 위치 / 외부 자세 ---
    pos_[0] = msg->data[3];
    pos_[1] = msg->data[4];
    pos_[2] = msg->data[5];

    rpy_meas_[0] = msg->data[6];
    rpy_meas_[1] = msg->data[7];
    rpy_meas_[2] = msg->data[8];

    // --- force input (world, raw/scaled) ---
    global_force_input_[0] = msg->data[0];
    global_force_input_[1] = msg->data[1];
    global_force_input_[2] = msg->data[2];

    global_force_input_scaled_[0] = msg->data[21];
    global_force_input_scaled_[1] = msg->data[22];
    global_force_input_scaled_[2] = msg->data[23];

    // --- accel / vel (world) ---
    global_acc_meas_[0] = msg->data[9];
    global_acc_meas_[1] = msg->data[10];
    global_acc_meas_[2] = msg->data[11];

    global_vel_meas_[0] = msg->data[12];
    global_vel_meas_[1] = msg->data[13];
    global_vel_meas_[2] = msg->data[14];

    // --- voltage ---
    vbat_raw_      = msg->data[15];
    vbat_filtered_ = msg->data[16];

    // --- ROS cmd position / yaw ---
    cmd_pos_[0] = msg->data[17];
    cmd_pos_[1] = msg->data[18];
    cmd_pos_[2] = msg->data[19];
    cmd_yaw_deg_ = msg->data[20];

    // --- controller final setpoint (FW target) ---
    final_pos_command_[0] = msg->data[24];
    final_pos_command_[1] = msg->data[25];
    final_pos_command_[2] = msg->data[26];
    final_yaw_deg_command_ = msg->data[27];

    // --- MOB/DOB 외란력 추정 (World) ---
    world_Fext_MOB_[0] = msg->data[29];
    world_Fext_MOB_[1] = msg->data[30];
    world_Fext_MOB_[2] = msg->data[31];

    world_Fext_DOB_[0] = msg->data[32];
    world_Fext_DOB_[1] = msg->data[33];
    world_Fext_DOB_[2] = msg->data[34];

    // --- vel_from_pos ---
    vel_from_pos_[0] = msg->data[35];
    vel_from_pos_[1] = msg->data[36];
    vel_from_pos_[2] = msg->data[37];

    // --- kalman/comp RPY ---
    rpy_kalman_[0] = msg->data[38];
    rpy_kalman_[1] = msg->data[39];
    rpy_kalman_[2] = msg->data[40];

    rpy_comp_[0] = msg->data[41];
    rpy_comp_[1] = msg->data[42];
    rpy_comp_[2] = msg->data[43];
  }

  void publishTfTimer()
  {
    auto stamp = this->get_clock()->now();

    // ===== 1) measured pose (외부 pose 기반) =====
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id  = "crazyflie";

    tf_msg.transform.translation.x = pos_[0];
    tf_msg.transform.translation.y = pos_[1];
    tf_msg.transform.translation.z = pos_[2];

    tf2::Quaternion q_meas;
    q_meas.setRPY(rpy_meas_[0], rpy_meas_[1], rpy_meas_[2]);
    tf_msg.transform.rotation.x = q_meas.x();
    tf_msg.transform.rotation.y = q_meas.y();
    tf_msg.transform.rotation.z = q_meas.z();
    tf_msg.transform.rotation.w = q_meas.w();

    tf_broadcaster_->sendTransform(tf_msg);

    // ===== 2) 제어기 최종 명령 pose (final_pos_command_ + final_yaw_deg_command_) =====
    geometry_msgs::msg::TransformStamped tf_cmd;
    tf_cmd.header.stamp = stamp;
    tf_cmd.header.frame_id = "world";
    tf_cmd.child_frame_id  = "crazyflie_cmd";

    tf_cmd.transform.translation.x = final_pos_command_[0];
    tf_cmd.transform.translation.y = final_pos_command_[1];
    tf_cmd.transform.translation.z = final_pos_command_[2];

    double yaw_cmd_rad = final_yaw_deg_command_ * M_PI / 180.0;
    tf2::Quaternion q_cmd;
    q_cmd.setRPY(0.0, 0.0, yaw_cmd_rad);
    tf_cmd.transform.rotation.x = q_cmd.x();
    tf_cmd.transform.rotation.y = q_cmd.y();
    tf_cmd.transform.rotation.z = q_cmd.z();
    tf_cmd.transform.rotation.w = q_cmd.w();

    tf_broadcaster_->sendTransform(tf_cmd);

    // ===== 3) Kalman attitude TF (RPY 기반) =====
    {
      geometry_msgs::msg::TransformStamped tf_k;
      tf_k.header.stamp = stamp;
      tf_k.header.frame_id = "world";
      tf_k.child_frame_id  = "crazyflie_kalman";
      tf_k.transform.translation.x = pos_[0];
      tf_k.transform.translation.y = pos_[1];
      tf_k.transform.translation.z = pos_[2];

      tf2::Quaternion qk;
      qk.setRPY(rpy_kalman_[0], rpy_kalman_[1], rpy_kalman_[2]);
      tf_k.transform.rotation.x = qk.x();
      tf_k.transform.rotation.y = qk.y();
      tf_k.transform.rotation.z = qk.z();
      tf_k.transform.rotation.w = qk.w();

      tf_broadcaster_->sendTransform(tf_k);
    }

    // ===== 4) Complementary attitude TF (RPY 기반) =====
    {
      geometry_msgs::msg::TransformStamped tf_c;
      tf_c.header.stamp = stamp;
      tf_c.header.frame_id = "world";
      tf_c.child_frame_id  = "crazyflie_comp";
      tf_c.transform.translation.x = pos_[0];
      tf_c.transform.translation.y = pos_[1];
      tf_c.transform.translation.z = pos_[2];

      tf2::Quaternion qc;
      qc.setRPY(rpy_comp_[0], rpy_comp_[1], rpy_comp_[2]);
      tf_c.transform.rotation.x = qc.x();
      tf_c.transform.rotation.y = qc.y();
      tf_c.transform.rotation.z = qc.z();
      tf_c.transform.rotation.w = qc.w();

      tf_broadcaster_->sendTransform(tf_c);
    }

    // ===== 공통 시작점 (기체 위치) =====
    geometry_msgs::msg::Point p0;
    p0.x = pos_[0];
    p0.y = pos_[1];
    p0.z = pos_[2];

    // ===== 5) MOB 기반 World 힘 화살표 =====
    {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = "world";
      marker.ns = "force_mob";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p1;
      const double scale_factor_F = 10.0;
      p1.x = pos_[0] + world_Fext_MOB_[0] * scale_factor_F;
      p1.y = pos_[1] + world_Fext_MOB_[1] * scale_factor_F;
      p1.z = pos_[2] + world_Fext_MOB_[2] * scale_factor_F;

      marker.points = {p0, p1};

      marker.scale.x = 0.02;
      marker.scale.y = 0.04;
      marker.scale.z = 0.06;

      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      marker.lifetime = rclcpp::Duration(0, 0);
      mob_force_pub_->publish(marker);
    }

    // ===== 6) DOB 기반 World 힘 화살표 =====
    {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = "world";
      marker.ns = "force_dob";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p1;
      const double scale_factor_F = 10.0;
      p1.x = pos_[0] + world_Fext_DOB_[0] * scale_factor_F;
      p1.y = pos_[1] + world_Fext_DOB_[1] * scale_factor_F;
      p1.z = pos_[2] + world_Fext_DOB_[2] * scale_factor_F;

      marker.points = {p0, p1};

      marker.scale.x = 0.02;
      marker.scale.y = 0.04;
      marker.scale.z = 0.06;

      marker.color.r = 0.6f;
      marker.color.g = 0.0f;
      marker.color.b = 0.8f;
      marker.color.a = 1.0f;

      marker.lifetime = rclcpp::Duration(0, 0);
      dob_force_pub_->publish(marker);
    }

    // ===== 7) World 가속도 화살표 =====
    {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = "world";
      marker.ns = "acceleration";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p1;
      const double scale_factor_A = 0.5;
      p1.x = pos_[0] + global_acc_meas_[0] * scale_factor_A;
      p1.y = pos_[1] + global_acc_meas_[1] * scale_factor_A;
      p1.z = pos_[2] + global_acc_meas_[2] * scale_factor_A;

      marker.points = {p0, p1};

      marker.scale.x = 0.015;
      marker.scale.y = 0.03;
      marker.scale.z = 0.05;

      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0f;

      marker.lifetime = rclcpp::Duration(0, 0);
      acc_pub_->publish(marker);
    }

    // ===== 8-1) vel_from_pos 속도 화살표 =====
    {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = "world";
      marker.ns = "velocity_from_pos";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p1;
      const double scale_factor_V = 1.0;
      p1.x = pos_[0] + vel_from_pos_[0] * scale_factor_V;
      p1.y = pos_[1] + vel_from_pos_[1] * scale_factor_V;
      p1.z = pos_[2] + vel_from_pos_[2] * scale_factor_V;

      marker.points = {p0, p1};

      marker.scale.x = 0.015;
      marker.scale.y = 0.03;
      marker.scale.z = 0.05;

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      marker.lifetime = rclcpp::Duration(0, 0);
      vel_pos_pub_->publish(marker);
    }

    // ===== 8-2) EKF velocity 화살표 =====
    {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = "world";
      marker.ns = "velocity_ekf";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point p1;
      const double scale_factor_V = 1.0;
      p1.x = pos_[0] + global_vel_meas_[0] * scale_factor_V;
      p1.y = pos_[1] + global_vel_meas_[1] * scale_factor_V;
      p1.z = pos_[2] + global_vel_meas_[2] * scale_factor_V;

      marker.points = {p0, p1};

      marker.scale.x = 0.015;
      marker.scale.y = 0.03;
      marker.scale.z = 0.05;

      marker.color.r = 1.0f;
      marker.color.g = 0.8f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      marker.lifetime = rclcpp::Duration(0, 0);
      vel_ekf_pub_->publish(marker);
    }

    // ===== 9) 벽 마커 =====
    {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = "world";
      marker.ns = "wall";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = 1.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.8;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.01;
      marker.scale.y = 1.0;
      marker.scale.z = 0.6;

      marker.color.r = 0.3f;
      marker.color.g = 0.3f;
      marker.color.b = 1.0f;
      marker.color.a = 0.3f;

      marker.lifetime = rclcpp::Duration(0, 0);
      wall_pub_->publish(marker);
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // marker publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mob_force_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dob_force_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr acc_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_pos_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_ekf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wall_pub_;

  // buffers
  Eigen::Vector3d pos_;
  Eigen::Vector3d rpy_meas_;     // 외부 pose 기반 RPY
  Eigen::Vector3d rpy_kalman_;   // kalman q -> RPY
  Eigen::Vector3d rpy_comp_;     // comp q -> RPY

  Eigen::Vector3d global_force_input_;
  Eigen::Vector3d global_force_input_scaled_;
  Eigen::Vector3d global_acc_meas_;
  Eigen::Vector3d global_vel_meas_;

  Eigen::Vector3d vel_from_pos_;

  double vbat_raw_      = 0.0;
  double vbat_filtered_ = 0.0;

  Eigen::Vector3d cmd_pos_;
  double          cmd_yaw_deg_ = 0.0;

  Eigen::Vector3d final_pos_command_;
  double          final_yaw_deg_command_ = 0.0;

  Eigen::Vector3d world_Fext_MOB_;
  Eigen::Vector3d world_Fext_DOB_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizVisual>());
  rclcpp::shutdown();
  return 0;
}
