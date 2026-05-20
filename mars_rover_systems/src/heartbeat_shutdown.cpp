#include "rclcpp/rclcpp.hpp"
#include <iostream>

int main(int argc, char **argv) {
  // initialize the ROS communication
  rclcpp::init(argc, argv);
  // print a message to the terminal
  std::cout << "Shutting down Mars rover 1..." << std::endl;
  // shutdown the ROS communication
  rclcpp::shutdown();
  return 0;
}
