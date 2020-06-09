#include <QtCore>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "grsim_actuator_sub.hpp"

int main(int argc, char ** argv) { 
    QHash<int, QList<qint8>> robots;
    std::string team = "all";
    int id = 0;
    if(argc < 2) {
        QList<qint8> yellowTeam;
        for(qint8 i = 0; i < 6; i++) {
            yellowTeam.append(i);
        }
        robots.insert(1, yellowTeam);

        QList<qint8> blueTeam;
        for(qint8 i = 0; i < 6; i++) {
            blueTeam.append(i);
        }
        robots.insert(0, blueTeam);
    } else if(argc == 3) {
        int key = 0;
        team = argv[1];
        if(team == "yellow") {
            key = 1;
        }

        id = atoi(argv[2]);
        QList<qint8> ids;
        ids.append((qint8)atoi(argv[2]));

        robots.insert(key, ids);
    }

    //ROS
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Grsim_Actuator_Sub>(robots, "actuator_"+team, id));
    rclcpp::shutdown();
    return 0;
}