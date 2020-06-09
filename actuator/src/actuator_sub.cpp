#include "actuator_sub.hpp"

using std::placeholders::_1;

Actuator_Sub::Actuator_Sub(QHash<int, QList<qint8>> control_table, std::string nodeName, int nodeID) : Node(nodeName+std::to_string(nodeID)) {
    // Create the table of subscriptions for this node
    QList<int> keys = control_table.keys();
    for(int i = 0; i < keys.size(); i++) {
        // Check team id:
        std::string teamColor;
        if(keys[i] == 0) {
            teamColor = "blue";
        } else {
            teamColor = "yellow";
        }

        QList<qint8> team = control_table.value(keys[i]);
        QHash<qint8, rclcpp::Subscription<ctr_msgs::msg::Command>::SharedPtr> aux;
        for(qint8 j = 0; j < team.size(); j++) {
            rclcpp::Subscription<ctr_msgs::msg::Command>::SharedPtr element;
            
            element = this->create_subscription<ctr_msgs::msg::Command>("command/"+teamColor+"_"+std::to_string(team[j]), 10, std::bind(&Actuator_Sub::send_cmd, this, _1));
            aux.insert(team[j], element);
        }

        _subsTable.insert(keys[i], aux);
    }
}