#include "rclcpp/rclcpp.hpp"
#include <QtCore>
#include <iostream>

#include "navigation/navigation.hpp"

int main(int argc, char **argv) {
  // Command line argument:
  if(argc < 3) {
    std::cout << "Please specify which robot should be controlled (color id)\n";
    std::cout << argc << std::endl;
    return 0;
  }

  std::string team = argv[1];
  int id = atoi(argv[2]);
  bool enableMultipleSources = false;

  if(argc > 3 && strcmp(argv[3], "true")) {
    enableMultipleSources = true;
  }

//  // ROS 2
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto nav_node = std::make_shared<Navigation>(team, id);
  executor.add_node(nav_node);
  std:: cout << "Number of threads: "<< executor.get_number_of_threads() << "\n";
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
