#include "rclcpp/rclcpp.hpp"
#include <QtCore>
#include <iostream>

int main(int argc, char **argv) {
  // Command line argument:
  if(argc != 3) {
    std::cout << "Please specify which robot should be controlled (color id)\n";
    std::cout << argc << std::endl;
    return 0;
  }

  std::string team = argv[1];
  int id = atoi(argv[2]);

//  // ROS 2
//  rclcpp::init(argc, argv);
//  rclcpp::executors::MultiThreadedExecutor executor;
//  auto ctr_node = std::make_shared<SSL_Controller>(team, id);
//  executor.add_node(ctr_node);
//  executor.spin();
//  rclcpp::shutdown();
  return 0;
}
