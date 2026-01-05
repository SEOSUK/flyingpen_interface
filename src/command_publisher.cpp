#include <rclcpp/rclcpp.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <ncurses.h>

#include <array>
#include <chrono>
#include <deque>
#include <string>

using namespace std::chrono_literals;

class CommandPublisher : public rclcpp::Node
{
public:
  CommandPublisher()
  : Node("command_publisher")
  {
    cf_position_pub_ = this->create_publisher<crazyflie_interfaces::msg::Position>(
      "cf2/cmd_position", 10);

    key_pub_ = this->create_publisher<std_msgs::msg::String>(
      "keyboard_input", 10);

    use_vel_mode_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "su/use_vel_mode", 10);
    force_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "su/cmd_force", 10);

    pos_delta_[0] = this->declare_parameter<double>("dx", 0.03);
    pos_delta_[1] = this->declare_parameter<double>("dy", 0.03);
    pos_delta_[2] = this->declare_parameter<double>("dz", 0.03);
    yaw_delta_deg_ = this->declare_parameter<double>("dyaw_deg", 5.0);
    force_delta_   = this->declare_parameter<double>("df", 0.01);

    cmd_xyz_yaw_.fill(0.0);
    force_des_ = 0.0;
    use_vel_mode_ = 0.0;
    status_msg_ = "ready";

    // history init
    last_inputs_.clear();
    for (int i = 0; i < 5; i++) last_inputs_.push_back("-");

    // ncurses init
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    color_enabled_ = false;
    if (has_colors()) {
      start_color();
      use_default_colors();
      init_pair(1, COLOR_YELLOW, -1);
      color_enabled_ = true;
    }

    drawLayout();

    // startup log 1회만 (이 정도는 괜찮음)
    RCLCPP_INFO(this->get_logger(), "command_publisher started.");

    timer_ = this->create_wall_timer(
      50ms, std::bind(&CommandPublisher::timerCallback, this));
  }

  ~CommandPublisher() override
  {
    endwin();
  }

private:
  // --------------------------
  // Layout rows
  // --------------------------
  static constexpr int ROW_USAGE_HEADER   = 0;
  static constexpr int ROW_USAGE_1        = 2;
  static constexpr int ROW_USAGE_2        = 3;
  static constexpr int ROW_USAGE_3        = 4;

  static constexpr int ROW_STATUS_HEADER  = 6;
  static constexpr int ROW_STATUS_MODE    = 8;
  static constexpr int ROW_STATUS_FORCE   = 9;
  static constexpr int ROW_STATUS_MSG     = 10;

  static constexpr int ROW_CMD_HEADER     = 12;
  static constexpr int ROW_CMD_LINE1      = 14;
  static constexpr int ROW_CMD_LINE2      = 15;
  static constexpr int ROW_CMD_HIST_HDR   = 17;
  static constexpr int ROW_CMD_HIST_0     = 18;   // newest
  static constexpr int ROW_CMD_HIST_1     = 19;
  static constexpr int ROW_CMD_HIST_2     = 20;
  static constexpr int ROW_CMD_HIST_3     = 21;
  static constexpr int ROW_CMD_HIST_4     = 22;   // oldest

  static constexpr size_t HISTORY_LEN     = 5;

  void timerCallback()
  {
    // ✅ 키보드 버퍼를 비울 때까지 모두 읽기 (연타 누락 방지)
    int ch;
    while ((ch = getch()) != ERR) {
      handleKey(static_cast<char>(ch));
    }

    publishPositionCmd();
    drawStatusBlock();
    drawCommandBlock();
  }

  void handleKey(char c)
  {
    // position
    if (c == 'w')       { cmd_xyz_yaw_[0] += pos_delta_[0]; pushInputHistory("w : x += dx"); }
    else if (c == 's')  { cmd_xyz_yaw_[0] -= pos_delta_[0]; pushInputHistory("s : x -= dx"); }
    else if (c == 'a')  { cmd_xyz_yaw_[1] += pos_delta_[1]; pushInputHistory("a : y += dy"); }
    else if (c == 'd')  { cmd_xyz_yaw_[1] -= pos_delta_[1]; pushInputHistory("d : y -= dy"); }
    else if (c == 'e')  { cmd_xyz_yaw_[2] += pos_delta_[2]; pushInputHistory("e : z += dz"); }
    else if (c == 'q')  { cmd_xyz_yaw_[2] -= pos_delta_[2]; pushInputHistory("q : z -= dz"); }
    else if (c == 'z')  { cmd_xyz_yaw_[3] += yaw_delta_deg_; pushInputHistory("z : yaw += dyaw"); }
    else if (c == 'c')  { cmd_xyz_yaw_[3] -= yaw_delta_deg_; pushInputHistory("c : yaw -= dyaw"); }
    else if (c == 'x')  { cmd_xyz_yaw_.fill(0.0); pushInputHistory("x : reset position cmd"); }

    // force
    else if (c == 'j')  { force_des_ += force_delta_; publishForce(); pushInputHistory("j : force += df"); }
    else if (c == 'k')  { force_des_ -= force_delta_; publishForce(); pushInputHistory("k : force -= df"); }
    else if (c == 'l')  { force_des_ = 0.0;           publishForce(); pushInputHistory("l : force reset"); }

    // mode set
    else if (c == 'i')  { setVelMode(1.0); pushInputHistory("i : set VELOCITY=1 + reset"); }
    else if (c == 'u')  { setVelMode(0.0); pushInputHistory("u : set POSITION=0 + reset"); }

    // ARM/DISARM
    else if (c == 'o' || c == 'p') {
      if (key_pub_) {
        std_msgs::msg::String msg;
        msg.data = std::string(1, c);
        key_pub_->publish(msg);

        if (c == 'o') {
          status_msg_ = "published 'o' to keyboard_input (ARM)";
          pushInputHistory("o : ARM (keyboard_input)");
        } else {
          status_msg_ = "published 'p' to keyboard_input (DISARM)";
          pushInputHistory("p : DISARM (keyboard_input)");
        }
      } else {
        status_msg_ = "key_pub not ready";
      }
    }

    // quit
    else if (c == 't') {
      pushInputHistory("t : quit");
      status_msg_ = "exit key pressed";
      rclcpp::shutdown();
    }
  }

  void publishPositionCmd()
  {
    crazyflie_interfaces::msg::Position msg;
    msg.x   = cmd_xyz_yaw_[0];
    msg.y   = cmd_xyz_yaw_[1];
    msg.z   = cmd_xyz_yaw_[2];
    msg.yaw = cmd_xyz_yaw_[3];
    cf_position_pub_->publish(msg);
  }

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
    snprintf(buf, sizeof(buf), "set cmd_fx = %.3f (via su_interface)", force_des_);
    status_msg_ = buf;
  }

  void setVelMode(double mode)
  {
    double new_mode = (mode > 0.5) ? 1.0 : 0.0;
    use_vel_mode_ = new_mode;

    if (!use_vel_mode_pub_) {
      status_msg_ = "use_vel_mode_pub not ready";
      return;
    }

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(use_vel_mode_);
    use_vel_mode_pub_->publish(msg);

    resetPositionCmd();

    char buf[128];
    snprintf(buf, sizeof(buf),
            "use_vel_mode = %.0f (%s mode), reset immediately",
            use_vel_mode_, (use_vel_mode_ > 0.5 ? "VELOCITY" : "POSITION"));
    status_msg_ = buf;
  }

  void resetPositionCmd()
  {
    cmd_xyz_yaw_.fill(0.0);
    status_msg_ = "Position reset to zero (immediate)";
  }

  // --------------------------
  // Input history (최근 5개)
  // --------------------------
  void pushInputHistory(const std::string& s)
  {
    last_inputs_.push_front(s);
    while (last_inputs_.size() > HISTORY_LEN) last_inputs_.pop_back();
    while (last_inputs_.size() < HISTORY_LEN) last_inputs_.push_back("-");
  }

  // --------------------------
  // Ncurses UI
  // --------------------------
  void drawSepLine(int row, const char* title)
  {
    move(row, 0);
    clrtoeol();
    printw("========================%s========================", title);
  }

  void drawLayout()
  {
    clear();

    drawSepLine(ROW_USAGE_HEADER, "usage");
    mvprintw(ROW_USAGE_1, 0, "position command: w/s(x), a/d(y), e/q(z), z/c(yaw), x(reset pos)");
    mvprintw(ROW_USAGE_2, 0, "mode change:      i(set vel=1 + reset), u(set pos=0 + reset)");
    mvprintw(ROW_USAGE_3, 0, "force command:    j/k/l (cmd_fx)   | arm/disarm: o/p   | quit: t");

    drawSepLine(ROW_STATUS_HEADER, "status");
    mvprintw(ROW_STATUS_MODE,  0, "mode: ");
    mvprintw(ROW_STATUS_FORCE, 0, "force command: ");
    mvprintw(ROW_STATUS_MSG,   0, "status: ");

    drawSepLine(ROW_CMD_HEADER, "Position Command, Now");
    mvprintw(ROW_CMD_LINE1, 0, "x = 0.000 , y = 0.000 , z = 0.000");
    mvprintw(ROW_CMD_LINE2, 0, "yaw = 0.0 deg");

    mvprintw(ROW_CMD_HIST_HDR, 0, "last inputs (recent 5):");
    for (int i = 0; i < 5; i++) {
      mvprintw(ROW_CMD_HIST_0 + i, 0, "  %d) -", i + 1);
    }

    refresh();
  }

  void drawStatusBlock()
  {
    const char* mode_str = (use_vel_mode_ > 0.5) ? "VELOCITY" : "POSITION";

    move(ROW_STATUS_MODE, 0);
    clrtoeol();
    printw("mode: %s (i: set VELOCITY=1, u: set POSITION=0)", mode_str);

    move(ROW_STATUS_FORCE, 0);
    clrtoeol();
    printw("force command: %.3f", force_des_);

    move(ROW_STATUS_MSG, 0);
    clrtoeol();
    printw("status: %s", status_msg_.c_str());

    refresh();
  }

  void drawCommandBlock()
  {
    move(ROW_CMD_LINE1, 0);
    clrtoeol();
    printw("x = %.3f , y = %.3f , z = %.3f",
           cmd_xyz_yaw_[0], cmd_xyz_yaw_[1], cmd_xyz_yaw_[2]);

    move(ROW_CMD_LINE2, 0);
    clrtoeol();
    printw("yaw = %.1f deg", cmd_xyz_yaw_[3]);

    mvprintw(ROW_CMD_HIST_HDR, 0, "last inputs (recent 5):");
    for (int i = 0; i < 5; i++) {
      move(ROW_CMD_HIST_0 + i, 0);
      clrtoeol();
      printw("  %d) %s", i + 1, last_inputs_[static_cast<size_t>(i)].c_str());
    }

    refresh();
  }

  // --------------------------
  // Members
  // --------------------------
  rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cf_position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              key_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             use_vel_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             force_pub_;
  rclcpp::TimerBase::SharedPtr                                     timer_;

  std::array<double, 4> cmd_xyz_yaw_;
  std::array<double, 3> pos_delta_;
  double yaw_delta_deg_;
  double force_des_;
  double force_delta_;
  double use_vel_mode_;

  std::string status_msg_;
  bool color_enabled_;

  std::deque<std::string> last_inputs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
