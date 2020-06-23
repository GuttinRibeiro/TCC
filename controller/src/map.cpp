#include "rclcpp/rclcpp.hpp"
#include <QtCore>
#include <string>
#include <iostream>
#include "map/map_node.hpp"

int main(int argc, char **argv) {
  // Command line argument:
  if(argc != 3) {
    std::cout << "Please specify which robot should be controlled (color id)\n";
    std::cout << argc << std::endl;
    return 0;
  }

  std::string team = argv[1];
  int id = atoi(argv[2]);

  //ROS 2
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto map_node = std::make_shared<Map_Node>(team, id, 60);
  executor.add_node(map_node);
  rclcpp::shutdown();
  return 0;
}
