#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <QtCore>
#include <QString>
#include <string>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "ctr_msgs/msg/command.hpp"
#include "ctr_msgs/srv/state.hpp"
#include "vision/msg/ball.hpp"
#include "vision/msg/robot.hpp"
#include "vision/msg/visionpkg.hpp"
#include "worldmap.hpp"
#include "../utils/field.hpp"
#include "infobus.hpp"

class Controller : public rclcpp::Node {
  private:
    WorldMap *_wm;
    InfoBus *_ib;
    timespec _start, _stop;
    double _time_now;

    rclcpp::CallbackGroup::SharedPtr _callback_group_vision;
    rclcpp::CallbackGroup::SharedPtr _callback_group_actuator;
    rclcpp::CallbackGroup::SharedPtr _callback_group_external_agent;
    rclcpp::CallbackGroup::SharedPtr _callback_group_controller;

    rclcpp::Subscription<vision::msg::Visionpkg>::SharedPtr _subVision;
    rclcpp::TimerBase::SharedPtr _timerVision;
    rclcpp::Service<ctr_msgs::srv::State>::SharedPtr _serviceExternalAgent;
    rclcpp::TimerBase::SharedPtr _timerCtr;
    rclcpp::Publisher<ctr_msgs::msg::Command>::SharedPtr _pubActuator;

    void visionCallback(const vision::msg::Visionpkg::SharedPtr msg);
    void updateWorldMap();
  protected:
    qint8 _id;
    std::string _team;

    InfoBus* infoBus() {return _ib;}

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
