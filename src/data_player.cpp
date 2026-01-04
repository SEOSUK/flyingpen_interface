#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <unordered_map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

struct CsvRow
{
  uint64_t timestamp_ns;
  std::string topic;
  std::vector<double> data;   // ← double 로 바꿈
};

class DataPlayer : public rclcpp::Node
{
public:
  DataPlayer()
  : Node("data_player")
  {
    start_time_sec_ = this->declare_parameter<double>("start_time_sec", 0.0);
    csv_name_       = this->declare_parameter<std::string>("csv_name", "rosbag2_20251109_225242_0.csv");
    package_name_   = this->declare_parameter<std::string>("package_name", "flying_pen");

    load_csv();
    publish_loop();
  }

private:
  std::vector<CsvRow> rows_;
  std::unordered_map<std::string,
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> publishers_;

  double      start_time_sec_;
  std::string csv_name_;
  std::string package_name_;

  void load_csv()
  {
    std::string share_dir;
    try {
      share_dir = ament_index_cpp::get_package_share_directory(package_name_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to find package '%s': %s",
                   package_name_.c_str(), e.what());
      return;
    }

    std::string filepath = share_dir + "/bag/folder/" + csv_name_;
    std::ifstream file(filepath);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file at: %s", filepath.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading CSV: %s", filepath.c_str());

    std::string line;
    // header skip
    std::getline(file, line);

    const uint64_t start_time_ns =
      static_cast<uint64_t>(start_time_sec_ * 1e9);

    while (std::getline(file, line)) {
      if (line.empty()) continue;

      std::stringstream ss(line);
      std::string cell;

      uint64_t timestamp_ns = 0;
      std::string topic_name;
      std::vector<double> data;
      std::vector<std::string> tokens;

      while (std::getline(ss, cell, ',')) {
        tokens.push_back(cell);
      }
      if (tokens.empty()) continue;

      // 0: timestamp
      timestamp_ns = std::stoull(tokens[0]);

      if (tokens.size() >= 2 && !tokens[1].empty() && tokens[1][0] == '/') {
        // 토픽 이름 있는 버전
        topic_name = tokens[1];
        for (size_t i = 2; i < tokens.size(); ++i) {
          data.push_back(std::stod(tokens[i]));
        }
      } else {
        // 토픽 이름 없는 버전 → 전부 /data_logging_msg
        topic_name = "/data_logging_msg";
        for (size_t i = 1; i < tokens.size(); ++i) {
          data.push_back(std::stod(tokens[i]));
        }
      }

      if (timestamp_ns >= start_time_ns) {
        rows_.push_back(CsvRow{timestamp_ns, topic_name, data});
      }
    }

    std::sort(rows_.begin(), rows_.end(),
      [](const CsvRow & a, const CsvRow & b) {
        return a.timestamp_ns < b.timestamp_ns;
      });

    RCLCPP_INFO(this->get_logger(), "Loaded %zu rows from CSV", rows_.size());
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
  get_or_create_publisher(const std::string & topic)
  {
    auto it = publishers_.find(topic);
    if (it != publishers_.end()) {
      return it->second;
    }
    auto pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
    publishers_[topic] = pub;
    RCLCPP_INFO(this->get_logger(), "Created publisher for topic: %s", topic.c_str());
    return pub;
  }

  void publish_loop()
  {
    std::thread([this]() {
      if (rows_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No rows to play.");
        return;
      }

      for (size_t i = 0; i < rows_.size(); ++i) {
        const auto & row = rows_[i];

        auto pub = get_or_create_publisher(row.topic);

        std_msgs::msg::Float64MultiArray msg;
        msg.data = row.data;   // ← 이제 타입이 맞으니까 그대로 대입 가능
        pub->publish(msg);

        if (i + 1 < rows_.size()) {
          uint64_t t_curr = rows_[i].timestamp_ns;
          uint64_t t_next = rows_[i + 1].timestamp_ns;
          uint64_t delta_ns = (t_next > t_curr) ? (t_next - t_curr) : 0;

          if (delta_ns > 0 && delta_ns < static_cast<uint64_t>(2e9)) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(delta_ns));
          }
        }
      }

      RCLCPP_INFO(this->get_logger(), "CSV playback finished.");
    }).detach();
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DataPlayer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
