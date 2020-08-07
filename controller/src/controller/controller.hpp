#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <QtCore>
#include <QString>
#include <string>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "ctr_msgs/srv/state.hpp"
#include "ctr_msgs/srv/elementrequest.hpp"
#include "ctr_msgs/srv/inforequest.hpp"
#include "ctr_msgs/srv/fieldinformationrequest.hpp"
#include "ctr_msgs/msg/command.hpp"
#include "ctr_msgs/msg/navigation.hpp"
#include "../utils/field.hpp"
#include "../utils/vector.hpp"
#include "infobus.hpp"

class Controller : public rclcpp::Node {
  private:
    rclcpp::CallbackGroup::SharedPtr _callback_group_actuator;
    rclcpp::CallbackGroup::SharedPtr _callback_group_external_agent;
    rclcpp::CallbackGroup::SharedPtr _callback_group_controller;
    rclcpp::CallbackGroup::SharedPtr _callback_group_map;
    rclcpp::CallbackGroup::SharedPtr _callback_group_navigation;

    rclcpp::Service<ctr_msgs::srv::State>::SharedPtr _serviceExternalAgent;
    rclcpp::TimerBase::SharedPtr _timerCtr;
    rclcpp::Publisher<ctr_msgs::msg::Command>::SharedPtr _pubActuator;
    rclcpp::Publisher<ctr_msgs::msg::Navigation>::SharedPtr _pubNavigation;
    rclcpp::Client<ctr_msgs::srv::Elementrequest>::SharedPtr _clientPositionRequest;
    rclcpp::Client<ctr_msgs::srv::Inforequest>::SharedPtr _clientInfoRequest;
    rclcpp::Client<ctr_msgs::srv::Fieldinformationrequest>::SharedPtr _clientFieldRequest;

    InfoBus *_ib;
  protected:
    qint8 _id;
    std::string _team;

    virtual void updateState(const std::shared_ptr<rmw_request_id_t> request_header,
                             const std::shared_ptr<ctr_msgs::srv::State::Request> request,
                             const std::shared_ptr<ctr_msgs::srv::State::Response> response) = 0;

    virtual void run() = 0;
    virtual ctr_msgs::msg::Navigation encodeNavMessage(Vector destination, float orientation) = 0;
    void goToLookTo(Vector destination, float orientation);
    void goTo(Vector destination);
    void lookTo(float orientation);
    void send_command(const ctr_msgs::msg::Command &msg);
    InfoBus* infoBus() const {return _ib;}
  public:
    Controller(std::string team, int id, int frequency = 60);
    ~Controller();
};

#endif // CONTROLLER_HPP
