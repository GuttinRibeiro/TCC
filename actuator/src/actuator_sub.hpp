#ifndef ACTUATOR_SUB_HH
#define ACTUATOR_SUB_HH

#include "rclcpp/rclcpp.hpp"
#include <QHash>
#include "ctr_msgs/msg/command.hpp"
#include "ctr_msgs/msg/velocity.hpp"

class Actuator_Sub : public rclcpp::Node {
    private:
        rclcpp::CallbackGroup::SharedPtr _callback_group;

        rclcpp::Subscription<ctr_msgs::msg::Command>::SharedPtr _subsTable1;
        rclcpp::Subscription<ctr_msgs::msg::Velocity>::SharedPtr _subsTable2;

        int _id;
        std::string _team;
    protected:

        virtual void send_cmd(ctr_msgs::msg::Command::SharedPtr msg) = 0;
        virtual void send_velocity(ctr_msgs::msg::Velocity::SharedPtr msg) = 0;
    public:
        Actuator_Sub(std::string team, int id, std::string nodeName, int nodeID);
};

#endif
