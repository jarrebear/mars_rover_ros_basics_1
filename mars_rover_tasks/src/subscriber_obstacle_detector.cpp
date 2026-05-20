#include "rclcpp/rclcpp.hpp"
#include <string>
// import the LaserScan module from sensor_msgs interface
#include "sensor_msgs/msg/laser_scan.hpp"
// import Quality of Service library, to set the correct profile and reliability
// to read sensor data.
#include "rclcpp/qos.hpp"

class ObstacleDetectorNode : public rclcpp::Node {
public:
  ObstacleDetectorNode(const std::string &node_name)
      : Node(node_name), node_name_(node_name) {

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    // is the most used to read LaserScan data
    // create the subscriber object
    // in this case, the subscriptor will be subscribed on /laser_scan topic
    // with a queue size of 10 messages. use the LaserScan module for
    // /laser_scan topic send the received info to the laserscan_callback
    // method.
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan", qos,
        std::bind(&ObstacleDetectorNode::laserscan_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "%s Ready...", node_name_.c_str());
  }

private:
  std::string node_name_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Distance at which we will want to turn to avoid obstacle
    float detection_distance{0.8};

    // Find the minimum distance in the ranges array
    float min_distance_lr{min_laserscan_range(msg, 167, 200)};
    float min_distance_l{min_laserscan_range(msg, 134, 166)};
    float min_distance_fl{min_laserscan_range(msg, 101, 133)};
    float min_distance_fr{min_laserscan_range(msg, 67, 100)};
    float min_distance_r{min_laserscan_range(msg, 34, 66)};
    float min_distance_rr{min_laserscan_range(msg, 0, 33)};
    // Log the minimum distance value
    RCLCPP_INFO(this->get_logger(), "Left_Rear: %.2f meters", min_distance_lr);
    RCLCPP_INFO(this->get_logger(), "Left: %.2f meters", min_distance_l);
    RCLCPP_INFO(this->get_logger(), "Front_Left: %.2f meters", min_distance_fl);
    RCLCPP_INFO(this->get_logger(), "Front_Right: %.2f meters",
                min_distance_fr);
    RCLCPP_INFO(this->get_logger(), "Right: %.2f meters", min_distance_r);
    RCLCPP_INFO(this->get_logger(), "Right_Rear: %.2f meters", min_distance_rr);
    if (min_distance_fl <= detection_distance) {
      RCLCPP_INFO(this->get_logger(), "Action Suggested = Turn Right");
    } else if (min_distance_fr <= detection_distance) {
      RCLCPP_INFO(this->get_logger(), "Action Suggested = Turn Left");
    } else if (min_distance_l <= detection_distance) {
      RCLCPP_INFO(this->get_logger(),
                  "Action Suggested = Go Forwards turning slightly right ( "
                  "proportional to the angle of detection)");
    } else if (min_distance_r <= detection_distance) {
      RCLCPP_INFO(this->get_logger(),
                  "Action Suggested = Go Forwards turning slightly left ( "
                  "proportional to the angle of detection)");
    } else if (min_distance_lr <= detection_distance) {
      RCLCPP_INFO(this->get_logger(), "Action Suggested = Go Forwards");
    } else if (min_distance_rr <= detection_distance) {
      RCLCPP_INFO(this->get_logger(), "Action Suggested = Go Forwards");
    } else {
      RCLCPP_INFO(this->get_logger(), "Action Suggested = Go Forwards");
    }
  }

  float min_laserscan_range(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                            int begin, int end) {
    float min_distance = *std::min_element(msg->ranges.begin() + begin,
                                           msg->ranges.begin() + end);
    return min_distance;
  }
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
