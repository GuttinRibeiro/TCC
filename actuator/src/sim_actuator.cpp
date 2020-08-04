#include <QtCore>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "grsim_actuator_sub.hpp"

int main(int argc, char ** argv) { 
  // Command line argument:
  if(argc != 3) {
    std::cout << "Please specify which robot should be controlled (color id)\n";
    std::cout << argc << std::endl;
    return 0;
  }

  std::string team = argv[1];
  int id = atoi(argv[2]);

  //ROS
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto actuator_node = std::make_shared<Grsim_Actuator_Sub>(team, id, "actuator_"+team, id);
  executor.add_node(actuator_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
