#include "rclcpp/rclcpp.hpp"
#include <QtCore>
#include <string>
#include <iostream>
#include "map/map_node.hpp"
#include "utils/fields/field_ssl2019.hpp"
#include <QApplication>
#include <thread>

void runROS(int argc, char **argv, std::string team, int id, Field *field) {
  //ROS 2
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto map_node = std::make_shared<Map_Node>(team, id, field);
  executor.add_node(map_node);
  std:: cout << "Number of threads: "<< executor.get_number_of_threads() << "\n";
  executor.spin();
}

int main(int argc, char **argv) {
  // Command line argument:
  if(argc != 3) {
    std::cout << "Please specify which robot should be controlled (color id)\n";
    std::cout << argc << std::endl;
    return 0;
  }

  std::string team = argv[1];
  int id = atoi(argv[2]);

  Field_SSL2019 field;

  QApplication app(argc, argv);
/*  //ROS 2
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto map_node = std::make_shared<Map_Node>(team, id, &field);
  executor.add_node(map_node);
  std:: cout << "Number of threads: "<< executor.get_number_of_threads() << "\n";
  executor.spin();*/

  std::thread rosThread(runROS, argc, argv, team, id, &field);

  // Block main thread
  int retn = app.exec();
  rosThread.join();

  // Stop node
  rclcpp::shutdown();
  return retn;
}
