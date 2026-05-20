#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <random>
#include <string>

double randomTemp(int lower, int upper) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> temp_dist(lower, upper);

  // To generate a random temperature:
  return temp_dist(gen);
}

class TemperatureNode : public rclcpp::Node {
public:
  TemperatureNode(const std::string &rover_name, double timer_period = 0.2)
      : Node(rover_name), rover_name_(rover_name) {
    // create a timer sending two parameters:
    // - the duration between two callbacks (timer_period seconds)
    // - the timer function (timer_callback)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(timer_period * 1000)),
        std::bind(&TemperatureNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto ros_time_stamp = this->get_clock()->now();
    double current_temp{randomTemp(20, 100)};
    RCLCPP_INFO(this->get_logger(), "%s current temperature = %f °C",
                rover_name_.c_str(), current_temp);
    if (current_temp > 70) {
      RCLCPP_WARN(this->get_logger(),
                  "WARNING %s current temperature = %f °C above threshold",
                  rover_name_.c_str(), current_temp);
    }
  }

  std::string rover_name_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  // initialize the ROS2 communication
  rclcpp::init(argc, argv);
  // declare the node constructor
  auto node = std::make_shared<TemperatureNode>("temperature_monitor", 1.0);
  // keeps the node alive, waits for a request to kill the node (ctrl+c)
  rclcpp::spin(node);
  // shutdown the ROS2 communication
  rclcpp::shutdown();
  return 0;
}