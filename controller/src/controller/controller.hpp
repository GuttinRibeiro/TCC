#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <QtCore>
#include <QString>
#include <string>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "ctr_msgs/msg/command.hpp"
#include "ctr_msgs/srv/state.hpp"
#include "../utils/field.hpp"
//#include "infobus.hpp"

class Controller : public rclcpp::Node {
  private:
    rclcpp::CallbackGroup::SharedPtr _callback_group_actuator;
    rclcpp::CallbackGroup::SharedPtr _callback_group_external_agent;
    rclcpp::CallbackGroup::SharedPtr _callback_group_controller;

    rclcpp::Service<ctr_msgs::srv::State>::SharedPtr _serviceExternalAgent;
    rclcpp::TimerBase::SharedPtr _timerCtr;
    rclcpp::Publisher<ctr_msgs::msg::Command>::SharedPtr _pubActuator;

  protected:
    qint8 _id;
    std::string _team;

    virtual void updateState(const std::shared_ptr<rmw_request_id_t> request_header,
                             const std::shared_ptr<ctr_msgs::srv::State::Request> request,
                             const std::shared_ptr<ctr_msgs::srv::State::Response> response) = 0;

    virtual void run() = 0;

    void sendCommand(const ctr_msgs::msg::Command *message);
  public:
    Controller(std::string team, int id, Field *field, int frequency = 60);
    ~Controller();
};

#endif // CONTROLLER_HPP
