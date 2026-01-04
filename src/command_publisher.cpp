#include <rclcpp/rclcpp.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>          // keyboard_input 용
#include <ncurses.h>
#include <array>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class CommandPublisher : public rclcpp::Node
{
public:
  CommandPublisher()
  : Node("command_publisher")
  {
    // position 명령
    cf_position_pub_ = this->create_publisher<crazyflie_interfaces::msg::Position>(
      "cf2/cmd_position", 10);

    // keyboard_input 퍼블리셔 (o/p 키용)
    key_pub_ = this->create_publisher<std_msgs::msg::String>(
      "keyboard_input", 10);

    // su_interface 로 보내는 모드/force 커맨드
    use_vel_mode_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "su/use_vel_mode", 10);
    force_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "su/cmd_force", 10);

    // 키보드 control step size
    pos_delta_[0] = this->declare_parameter<double>("dx", 0.03);
    pos_delta_[1] = this->declare_parameter<double>("dy", 0.03);
    pos_delta_[2] = this->declare_parameter<double>("dz", 0.03);
    yaw_delta_deg_ = this->declare_parameter<double>("dyaw_deg", 5.0);
    force_delta_   = this->declare_parameter<double>("df", 0.01);

    cmd_xyz_yaw_.fill(0.0);
    force_des_ = 0.0;
    use_vel_mode_ = 0.0;   // 기본: position mode
    status_msg_ = "ready";


    // ncurses 초기화
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    // 색상 초기화 (있을 때만)
    color_enabled_ = false;
    if (has_colors()) {
      start_color();
      use_default_colors();                 // 배경은 터미널 기본색 유지
      init_pair(1, COLOR_YELLOW, -1);       // 글씨만 노란색
      color_enabled_ = true;
    }

    mvprintw(0, 0, "command_publisher running. press 't' to quit.");
    mvprintw(2, 0, "keys: w/s(x), a/d(y), e/q(z), z/c(yaw), x(reset pos)");
    mvprintw(3, 0, "      j/k/l(force cmd_fx), i(toggle pos/vel+reset), o(ARM), p(DISARM)");
    refresh();

    RCLCPP_INFO(this->get_logger(), "command_publisher started.");

    // 50ms timer
    timer_ = this->create_wall_timer(
      50ms, std::bind(&CommandPublisher::timerCallback, this));
  }

  ~CommandPublisher() override
  {
    endwin();
  }

private:
  void timerCallback()
  {
    int ch = getch();
    if (ch != ERR) {
      handleKey(static_cast<char>(ch));
    }



    publishPositionCmd();
    drawStatusLine();
  }

  void handleKey(char c)
  {
    // 위치 명령 조작
    if (c == 'w')       cmd_xyz_yaw_[0] += pos_delta_[0];
    else if (c == 's')  cmd_xyz_yaw_[0] -= pos_delta_[0];
    else if (c == 'a')  cmd_xyz_yaw_[1] += pos_delta_[1];
    else if (c == 'd')  cmd_xyz_yaw_[1] -= pos_delta_[1];
    else if (c == 'e')  cmd_xyz_yaw_[2] += pos_delta_[2];
    else if (c == 'q')  cmd_xyz_yaw_[2] -= pos_delta_[2];
    else if (c == 'z')  cmd_xyz_yaw_[3] += yaw_delta_deg_;
    else if (c == 'c')  cmd_xyz_yaw_[3] -= yaw_delta_deg_;
    else if (c == 'x')  cmd_xyz_yaw_.fill(0.0);

    // force 조작 → su_interface 로 전송
    else if (c == 'j')  { force_des_ += force_delta_; publishForce(); }
    else if (c == 'k')  { force_des_ -= force_delta_; publishForce(); }
    else if (c == 'l')  { force_des_ = 0.0;           publishForce(); }

    // i: velocity 모드 토글 + 0.1초 후 position reset
    else if (c == 'i')  { toggleVelMode(); }

    // ARM / DISARM
    else if (c == 'o' || c == 'p') {
      if (key_pub_) {
        std_msgs::msg::String msg;
        msg.data = std::string(1, c);
        key_pub_->publish(msg);

        if (c == 'o') {
          status_msg_ = "published 'o' to keyboard_input (ARM)";
          RCLCPP_INFO(this->get_logger(), "ARM key 'o' published.");
        } else {
          status_msg_ = "published 'p' to keyboard_input (DISARM)";
          RCLCPP_INFO(this->get_logger(), "DISARM key 'p' published.");
        }
      }
    }

    // 종료
    else if (c == 't') {
      RCLCPP_INFO(this->get_logger(), "exit key pressed.");
      rclcpp::shutdown();
    }
  }

  void publishPositionCmd()
  {
    crazyflie_interfaces::msg::Position msg;
    msg.x   = cmd_xyz_yaw_[0];
    msg.y   = cmd_xyz_yaw_[1];
    msg.z   = cmd_xyz_yaw_[2];
    msg.yaw = cmd_xyz_yaw_[3];  // deg
    cf_position_pub_->publish(msg);
  }

  // force 파라미터 publish
  void publishForce()
  {
    if (!force_pub_) {
      status_msg_ = "force_pub not ready";
      return;
    }

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(force_des_);
    force_pub_->publish(msg);

    char buf[128];
    snprintf(buf, sizeof(buf),
             "set cmd_fx = %.3f (via su_interface)", force_des_);
    status_msg_ = buf;
  }

  void toggleVelMode()
  {
    use_vel_mode_ = (use_vel_mode_ < 0.5) ? 1.0 : 0.0;

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(use_vel_mode_);
    use_vel_mode_pub_->publish(msg);

    // ★ 즉시 reset
    resetPositionCmd();

    char buf[128];
    snprintf(buf, sizeof(buf),
            "use_vel_mode = %.1f (%s mode), reset immediately",
            use_vel_mode_, (use_vel_mode_ > 0.5 ? "VELOCITY" : "POSITION"));
    status_msg_ = buf;

    RCLCPP_INFO(this->get_logger(), "%s", buf);
  }


  // ★ position command 리셋 동작
  void resetPositionCmd()
  {
    cmd_xyz_yaw_.fill(0.0);

    char buf[128];
    snprintf(buf, sizeof(buf),
             "Position reset to zero (0.1s delayed)");
    status_msg_ = buf;

    RCLCPP_INFO(this->get_logger(), "[command_publisher] %s", buf);
  }

  void drawStatusLine()
  {
    const char* mode_str = (use_vel_mode_ > 0.5) ? "VELOCITY" : "POSITION";

    mvprintw(4, 0,
      "cmd: x=%6.2f  y=%6.2f  z=%6.2f  yaw=%6.1f deg  | f_x=%5.2f       ",
      cmd_xyz_yaw_[0], cmd_xyz_yaw_[1], cmd_xyz_yaw_[2], cmd_xyz_yaw_[3], force_des_);

    mvprintw(5, 0,
      "mode: %s (i: toggle+delayed reset)                                ",
      mode_str);

    move(6, 0);
    clrtoeol();

    printw("status: %s", status_msg_.c_str());

    refresh();
  }

  // ============================
  // 멤버 변수
  // ============================
  rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cf_position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           key_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr          use_vel_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr          force_pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;

  std::array<double, 4> cmd_xyz_yaw_;
  std::array<double, 3> pos_delta_;
  double yaw_delta_deg_;
  double force_des_;
  double force_delta_;
  double use_vel_mode_;


  std::string status_msg_;
  bool color_enabled_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
