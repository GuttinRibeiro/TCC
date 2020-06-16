#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <QtCore>
#include <QString>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ctr_msgs/msg/command.hpp"
#include "ctr_msgs/srv/state.hpp"
#include "vision/msg/ball.hpp"
#include "vision/msg/robot.hpp"
#include "vision/msg/visionpkg.hpp"
#include "worldmap.hpp"

class Controller : public rclcpp::Node {
  private:
    qint8 _id;
    std::string _team;
    int _state;

    rclcpp::CallbackGroup::SharedPtr _callback_group_vision;
    rclcpp::CallbackGroup::SharedPtr _callback_group_actuator;
    rclcpp::CallbackGroup::SharedPtr _callback_group_external_agent;

    void updateWorldMap(const vision::msg::Visionpkg::SharedPtr msg);
  protected:
    rclcpp::Publisher<ctr_msgs::msg::Command>::SharedPtr _pubActuator;
    rclcpp::Subscription<vision::msg::Visionpkg>::SharedPtr _subVision;
    rclcpp::Service<ctr_msgs::srv::State>::SharedPtr _serviceExternalAgent;

    virtual void updateState(const std::shared_ptr<rmw_request_id_t> request_header,
                             const std::shared_ptr<ctr_msgs::srv::State::Request> request,
                             const std::shared_ptr<ctr_msgs::srv::State::Response> response) = 0;
  public:
    Controller(std::string team, int id);
};

#endif // CONTROLLER_HPP
