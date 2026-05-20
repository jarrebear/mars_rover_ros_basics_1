#include "rclcpp/rclcpp.hpp"
#include <string>

class ObstacleDetectorNode : public rclcpp::Node {
public:
  ObstacleDetectorNode(const std::string &node_name)
      : Node(node_name), node_name_(node_name) {
    RCLCPP_INFO(this->get_logger(), "%s Ready...", node_name_.c_str());
  }

  std::string node_name_;
};

int main(int argc, char **argv) {
  // initialize the ROS2 communication
  rclcpp::init(argc, argv);
  // declare the node constructor
  auto node = std::make_shared<ObstacleDetectorNode>("obstacle_detector_node");
  // keeps the node alive, waits for a request to kill the node (ctrl+c)
  rclcpp::spin(node);
  // shutdown the ROS2 communication
  rclcpp::shutdown();
  return 0;
}
