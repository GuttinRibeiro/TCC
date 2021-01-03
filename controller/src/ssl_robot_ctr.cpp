#include "rclcpp/rclcpp.hpp"
#include <QtCore>
#include <iostream>
#include "controller/ssl_behavioralnode.hpp"
#include "utils/fields/field_ssl2019.hpp"
#define NUM_ROBOTS 6

int main(int argc, char **argv) {
  // Command line argument:
  if(argc != 3) {
    std::cout << "Please specify which robot should be controlled (color id)\n";
    std::cout << argc << std::endl;
    return 0;
  }

  std::string team = argv[1];
  int id = atoi(argv[2]);
  QList<int> ids;
  for (int i = 0; i < NUM_ROBOTS; i++) {
    ids.append(i);
  }


  // ROS 2
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto ctr_node = std::make_shared<SSL_BehavioralNode>(team, id, ids);
  executor.add_node(ctr_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
