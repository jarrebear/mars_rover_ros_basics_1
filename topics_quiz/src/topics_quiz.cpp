#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <limits>
#include <map>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class TopicsQuizNode : public rclcpp::Node {
public:
  TopicsQuizNode() : Node("topics_quiz_node") {
    // Subscriber to LaserScan
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan", 10,
        std::bind(&TopicsQuizNode::laserscan_callback, this,
                  std::placeholders::_1));

    // Subscriber to Odometry
    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&TopicsQuizNode::odom_callback, this, std::placeholders::_1));

    // Subscriber to NasaMission
    subscriber_mission_ = this->create_subscription<std_msgs::msg::String>(
        "/nasa_mission", 10,
        std::bind(&TopicsQuizNode::mission_callback, this,
                  std::placeholders::_1));

    // Publisher for movement commands
    publisher_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Initialize state variables
    turning_ = false;
    turn_direction_ = -0.5; // Default to turning right
    distance_from_goal_ = 0.0;
    traveling_to_goal_ = false;
    current_position_x_ = 0.0;
    current_position_y_ = 0.0;
    yaw_ = 0.0;
    current_goal_ = {0.0, 0.0};
    RCLCPP_INFO(this->get_logger(), "Topics Quiz Node Ready...");
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (traveling_to_goal_) {

      // Define the sectors
      std::map<std::string, std::pair<int, int>> sectors = {
          {"Right_Rear", {0, 33}},    {"Right", {34, 66}},
          {"Front_Right", {67, 100}}, {"Front_Left", {101, 133}},
          {"Left", {134, 166}},       {"Left_Rear", {167, 199}}};

      // Initialize the minimum distances for each sector
      std::map<std::string, float> min_distances;
      for (const auto &sector : sectors) {
        min_distances[sector.first] = std::numeric_limits<float>::infinity();
      }

      // Find the minimum distance in each sector
      for (const auto &sector : sectors) {
        int start_idx = sector.second.first;
        int end_idx = sector.second.second;

        // Ensure the index range is within bounds and not empty
        if (start_idx < static_cast<int>(msg->ranges.size()) &&
            end_idx < static_cast<int>(msg->ranges.size())) {
          float min_val = std::numeric_limits<float>::infinity();
          for (int i = start_idx; i <= end_idx; ++i) {
            if (msg->ranges[i] < min_val) {
              min_val = msg->ranges[i];
            }
          }
          min_distances[sector.first] = min_val;
        }
      }

      // Define the threshold for obstacle detection
      float obstacle_threshold = 0.4; // meters

      // Determine detected obstacles
      std::map<std::string, bool> detections;
      for (const auto &min_dist : min_distances) {
        detections[min_dist.first] = min_dist.second < obstacle_threshold;
      }

      // Determine suggested action based on detection
      auto action = geometry_msgs::msg::Twist();

      // If obstacles are detected in both front sectors, continue turning
      if (detections["Front_Left"] || detections["Front_Right"]) {
        if (!turning_) {
          // Start turning if not already turning
          turning_ = true;
          turn_direction_ = -0.5; // Turning right
        }
        action.angular.z = turn_direction_; // Continue turning
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle ahead, turning to clear path.");
      } else {
        turning_ = false; // Stop turning when the front is clear
        // Priority 2: Side detections
        if (detections["Left"]) {
          action.linear.x = 0.2;   // Move forward slowly
          action.angular.z = -0.3; // Slight right turn
          RCLCPP_INFO(this->get_logger(),
                      "Obstacle on the left, turning slightly right.");
        } else if (detections["Right"]) {
          action.linear.x = 0.2;  // Move forward slowly
          action.angular.z = 0.3; // Slight left turn
          RCLCPP_INFO(this->get_logger(),
                      "Obstacle on the right, turning slightly left.");
        }
      }

      if (!detections["Front_Left"] && !detections["Front_Right"] &&
          !detections["Left"] && !detections["Right"]) {
        travel_to_goal();
        RCLCPP_INFO(this->get_logger(), "Traveling...");
      } else { // Publish the action command
        publisher_vel_->publish(action);
      }
    }
  }

private:
  void mission_callback(const std_msgs::msg::String::SharedPtr msg) {
    // Extract the Mission message from the nasa_mision topic
    current_mission_ = msg->data;

    //
    if (traveling_to_goal_ == true) {
      RCLCPP_INFO(this->get_logger(), "Already in transit, pease wait!");
      return;
    }

    if (current_mission_ == "Go-Home") {
      current_goal_ = {0, 0};
      calc_distance_from_goal();
      if (distance_from_goal_ < 0.1) {
        RCLCPP_INFO(this->get_logger(), "Already at home. No need to move");
        return;
      } else {
        traveling_to_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "Travelling to home");
      }
    } else if (current_mission_ == "Go-Pickup") {
      current_goal_ = {-2.342, -2.432};
      calc_distance_from_goal();
      if (distance_from_goal_ < 0.1) {
        RCLCPP_INFO(this->get_logger(), "Already at pickup. No need to move");
        return;
      } else {
        traveling_to_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "Travelling to pickup");
      }
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Please give a valid command (either Go-Pickup or Go-Home)");
    }
  }

private:
  void calc_distance_from_goal() {
    // Calculate the distance from the current goal
    double dx = current_goal_[0] - current_position_x_;
    double dy = current_goal_[1] - current_position_y_;

    distance_from_goal_ = std::sqrt(dx * dx + dy * dy);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract the x, y coordinates from the odometry message
    current_position_x_ = msg->pose.pose.position.x;
    current_position_y_ = msg->pose.pose.position.y;

    // Calculate the distance from the current goal
    calc_distance_from_goal();

    // RCLCPP_INFO(this->get_logger(), "Distance from Goal: %.2f meters",
    //             distance_from_goal_);

    // Calculate the yaw (orientation around the z-axis)
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);

    // If we are travelling to the goal and within 0.1 meters stop the rover
    const double TOLERANCE = 0.1;
    if (distance_from_goal_ <= TOLERANCE && traveling_to_goal_) {
      // If the rover is back within 0.1 meters of the goal, resume normal
      // operation
      traveling_to_goal_ = false;
      publisher_vel_->publish(geometry_msgs::msg::Twist());
      RCLCPP_INFO(this->get_logger(),
                  "Within 0.1 meters of goal, stopping rover.");
    }
  }

private:
  void travel_to_goal() {
    auto action = geometry_msgs::msg::Twist();

    // Calculate the desired angle to the goal
    double desired_yaw = std::atan2(current_goal_[1] - current_position_y_,
                                    current_goal_[0] - current_position_x_);

    // Calculate the difference between current yaw and desired yaw
    double yaw_error = desired_yaw - yaw_;

    // Normalize the yaw error to the range [-pi, pi]
    yaw_error = std::fmod(yaw_error + M_PI, 2 * M_PI) - M_PI;

    // If the yaw error is significant, rotate towards the goal
    if (std::abs(yaw_error) > 0.1) // 0.1 radians threshold for orientation
    {
      action.angular.z = yaw_error > 0 ? 0.5 : -0.5;
      RCLCPP_INFO(this->get_logger(), "Turning towards goal. Yaw error: % .2f ",
                  yaw_error);
    } else {
      // If oriented towards the goal, move forward
      action.linear.x = 0.5;
      RCLCPP_INFO(this->get_logger(), "Heading towards goal.");
    }

    // Publish the action command
    publisher_vel_->publish(action);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_mission_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;

  bool turning_;
  double turn_direction_;
  double distance_from_goal_;
  bool traveling_to_goal_;
  double current_position_x_;
  double current_position_y_;
  double yaw_;
  std::array<double, 2> current_goal_;
  std::string current_mission_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopicsQuizNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}