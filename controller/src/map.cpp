#include "rclcpp/rclcpp.hpp"
#include <QtCore>
#include <string>
#include <iostream>
#include "map/map_node.hpp"
#include "utils/fields/field_ssl2019.hpp"
#include <QApplication>
#include <thread>
#include <ctime>

#include "map/gui/soccerview.hh"
#include "map/exithandler.hh"

void runROS(int argc, char **argv, std::string team, int id, std::string side, Field *field, WorldMap *wm) {
  //ROS 2
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto map_node = std::make_shared<Map_Node>(team, id, side, wm, field);
  executor.add_node(map_node);
  std:: cout << "Number of threads: "<< executor.get_number_of_threads() << "\n";
  executor.spin();
}

[[noreturn]] void runGUI(GLSoccerView *view, WorldMap *wm, Field *field, int frequency = 50) {
  const auto timeWindow = std::chrono::milliseconds(1000/frequency);
  while (true) {
    auto start = std::chrono::steady_clock::now();
    view->updateFieldGeometry(field);
    view->updateDetection(wm->getElement(Groups::BALL, 0), wm->getGroup(Groups::BLUE), wm->getGroup(Groups::YELLOW));
    auto end = std::chrono::steady_clock::now();
    auto elapsed = end - start;
    auto timeToWait = timeWindow - elapsed;
    if(timeToWait > std::chrono::milliseconds::zero()) {
      std::this_thread::sleep_for(timeToWait);
    }
  }
}

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  // Command line argument:
  if(argc != 4) {
    std::cout << "Please specify which robot should be controlled (color id side)\n";
    std::cout << argc << std::endl;
    return 0;
  }

  std::string team = argv[1];
  int id = atoi(argv[2]);
  std::string side = argv[3];
  QString window = QString::fromStdString("Map: "+team+" "+ std::to_string(id) + " " + side);

  app.setApplicationName(window);
  app.setApplicationVersion("0.0");

  Field_SSL2019 field;
  WorldMap wm(1.0);
  GLSoccerView *view = new GLSoccerView();
  view->show();

  std::thread guiThread(runGUI, view, &wm, &field, 60);
  std::thread rosThread(runROS, argc, argv, team, id, side, &field, &wm);

  // Setup ExitHandler
  ExitHandler::setup(&app);

  // Block main thread
  int retn = app.exec();
  guiThread.join();
  rosThread.join();

  // Stop node
  rclcpp::shutdown();

  view->close();
  delete view;
  return retn;
}
