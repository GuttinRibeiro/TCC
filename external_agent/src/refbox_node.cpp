#include <QApplication>
#include "rclcpp/rclcpp.hpp"
#include "refbox_interface.hpp"
#include "colors.h"

int main(int argc, char ** argv) {
    QHash<int, QList<qint8>> robots;
//    if(argc < 2) {
//        QList<qint8> yellowTeam;
//        for(qint8 i = 0; i < 6; i++) {
//            yellowTeam.append(i);
//        }
//        robots.insert(Colors::YELLOW, yellowTeam);

//        QList<qint8> blueTeam;
//        for(qint8 i = 0; i < 6; i++) {
//            blueTeam.append(i);
//        }
//        robots.insert(Colors::BLUE, blueTeam);
//    }
    QList<qint8> blueTeam;
    blueTeam.append(0);
    robots.insert(Colors::BLUE, blueTeam);

    // ROS:
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Refbox_Interface>(robots);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
