#include <rclcpp/rclcpp.hpp>
#include <string>
// import the LaserScan module from sensor_msgs interface
#include "sensor_msgs/msg/laser_scan.hpp"
// import twist module from geometry_msgs interface
#include <geometry_msgs/msg/twist.hpp>
// import Quality of Service library, to set the correct profile and reliability
// to read sensor data.
#include "rclcpp/qos.hpp"

class AutonomousExplorationNode : public rclcpp::Node {
public:
  AutonomousExplorationNode() : Node("autonomous_exploration_node") {
    // Initialize the AutonomousExploration

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    // is the most used to read LaserScan data
    // create the subscriber object
    // in this case, the subscriptor will be subscribed on /laser_scan topic
    // with a queue size of 10 messages. use the LaserScan module for
    // /laser_scan topic send the received info to the laserscan_callback
    // method.
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan", qos,
        std::bind(&AutonomousExplorationNode::laserscan_callback, this,
                  std::placeholders::_1));

    // Initialize the Publisher for plant detection results
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "autonomous_exploration_node is ready...");
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Distance at which we will want to turn to avoid obstacle
    float detection_distance{0.8};

    // Find the minimum distance in the ranges array
    // float min_distance_lr{min_laserscan_range(msg, 167, 200)};
    float min_distance_l{min_laserscan_range(msg, 134, 166)};
    float min_distance_fl{min_laserscan_range(msg, 101, 133)};
    float min_distance_fr{min_laserscan_range(msg, 67, 100)};
    float min_distance_r{min_laserscan_range(msg, 34, 66)};
    // float min_distance_rr{min_laserscan_range(msg, 0, 33)};
    // Log the minimum distance value
    // RCLCPP_INFO(this->get_logger(), "Left_Rear: %.2f meters",
    // min_distance_lr); RCLCPP_INFO(this->get_logger(), "Left: %.2f meters",
    // min_distance_l); RCLCPP_INFO(this->get_logger(), "Front_Left: %.2f
    // meters", min_distance_fl); RCLCPP_INFO(this->get_logger(), "Front_Right:
    // %.2f meters",
    //             min_distance_fr);
    // RCLCPP_INFO(this->get_logger(), "Right: %.2f meters", min_distance_r);
    // RCLCPP_INFO(this->get_logger(), "Right_Rear: %.2f meters",
    // min_distance_rr);
    auto pub_msg = geometry_msgs::msg::Twist();

    if (min_distance_fl <= detection_distance) {
      pub_msg.angular.z = -0.5;
      publisher_->publish(pub_msg);
      RCLCPP_INFO(this->get_logger(),
                  "Obstacle on the front left, turning right!");
    } else if (min_distance_fr <= detection_distance) {
      pub_msg.angular.z = 0.5;
      publisher_->publish(pub_msg);
      RCLCPP_INFO(this->get_logger(),
                  "Obstacle on the front right, turning left!");
    } else if (min_distance_l <= detection_distance) {
      pub_msg.linear.x = 0.2;
      pub_msg.angular.z = -0.25;
      publisher_->publish(pub_msg);
      RCLCPP_INFO(this->get_logger(),
                  "Obstacle on the left, turning slightly right!");
    } else if (min_distance_r <= detection_distance) {
      pub_msg.linear.x = 0.2;
      pub_msg.angular.z = 0.25;
      publisher_->publish(pub_msg);
      RCLCPP_INFO(this->get_logger(),
                  "Obstacle on the right, turning slightly left!");
    } else {
      pub_msg.linear.x = 0.5;
      publisher_->publish(pub_msg);
      RCLCPP_INFO(this->get_logger(), "No obstacles in front, moving forward!");
    }
  }

private:
  float min_laserscan_range(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                            int begin, int end) {
    float min_distance = *std::min_element(msg->ranges.begin() + begin,
                                           msg->ranges.begin() + end);
    return min_distance;
  }

public:
  void stop_rover() {
    auto stop_msg =
        geometry_msgs::msg::Twist(); // All fields default to zero, which
                                     // represents stopping the rover
    publisher_->publish(stop_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing stop message before shutdown");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

std::shared_ptr<AutonomousExplorationNode> autonomous_exploration_node;

void signal_handler([[maybe_unused]] int signum) {
  autonomous_exploration_node->stop_rover();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  rclcpp::shutdown();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  autonomous_exploration_node = std::make_shared<AutonomousExplorationNode>();

  signal(SIGINT, signal_handler);

  rclcpp::spin(autonomous_exploration_node);

  rclcpp::shutdown();
  return 0;
}
