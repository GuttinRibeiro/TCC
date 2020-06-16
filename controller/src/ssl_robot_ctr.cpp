#include "rclcpp/rclcpp.hpp"
#include <QtCore>
#include <iostream>
#include "controller/controller.hpp"

int main(int argc, char **argv) {
  // Command line argument:
  if(argc != 2) {
    std::cout << "Please specify which robot should be controlled (color id)\n";
    return 0;
  }

  std::string team = argv[1];
  int id = atoi(argv[2]);


  // ROS 2
  /*rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto ctr_node = std::make_shared<Controller>(team, id); // Change later for my SSL controller
  executor.add_node(ctr_node);
  executor.spin();
  rclcpp::shutdown();*/
  return 0;
}
