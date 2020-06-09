#ifndef ACTUATOR_SUB_HH
#define ACTUATOR_SUB_HH

#include "rclcpp/rclcpp.hpp"
#include <QtCore>
#include "ctr_msgs/msg/command.hpp"

class Actuator_Sub : public rclcpp::Node {
    private:
        QHash<int, QHash<qint8, rclcpp::Subscription<ctr_msgs::msg::Command>::SharedPtr>> _subsTable;
    protected:

        virtual void send_cmd(ctr_msgs::msg::Command::SharedPtr msg) = 0;
    public:
        Actuator_Sub(QHash<int, QList<qint8>> control_table, std::string nodeName, int nodeID);
};

#endif