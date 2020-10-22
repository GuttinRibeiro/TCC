#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <QtCore>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ctr_msgs/srv/elementrequest.hpp"
#include "ctr_msgs/srv/inforequest.hpp"
#include "ctr_msgs/srv/fieldinformationrequest.hpp"
#include "ctr_msgs/msg/state.hpp"
#include "ctr_msgs/msg/command.hpp"
#include "ctr_msgs/msg/navigation.hpp"
#include "../utils/field.hpp"
#include "../utils/vector.hpp"
#include "../utils/entity.hpp"
#include "infobus.hpp"

class Controller : public rclcpp::Node, public Entity {
  private:
    rclcpp::CallbackGroup::SharedPtr _callback_group_actuator;
    rclcpp::CallbackGroup::SharedPtr _callback_group_external_agent;
    rclcpp::CallbackGroup::SharedPtr _callback_group_map;
    rclcpp::CallbackGroup::SharedPtr _callback_group_navigation;

    rclcpp::Subscription<ctr_msgs::msg::State>::SharedPtr _subExternalAgent;
    rclcpp::Publisher<ctr_msgs::msg::Command>::SharedPtr _pubActuator;
    rclcpp::Publisher<ctr_msgs::msg::Navigation>::SharedPtr _pubNavigation;
    rclcpp::Client<ctr_msgs::srv::Elementrequest>::SharedPtr _clientPositionRequest;
    rclcpp::Client<ctr_msgs::srv::Inforequest>::SharedPtr _clientInfoRequest;
    rclcpp::Client<ctr_msgs::srv::Fieldinformationrequest>::SharedPtr _clientFieldRequest;

    InfoBus *_ib;

  protected:
    qint8 _id;
    std::string _team;

    virtual void updateState(ctr_msgs::msg::State::SharedPtr msg) = 0;

    virtual void run() {return;}
    virtual void configure() {return;}
    virtual std::string name() {return "Controller";}
    virtual ctr_msgs::msg::Navigation encodeNavMessage(Vector destination, float orientation, bool avoidBall, bool avoidAllies, bool avoidEnemies) = 0;
    void goToLookTo(Vector destination, Vector posToLook, bool avoidBall = false, bool avoidAllies = true, bool avoidEnemies = true);
    void goTo(Vector destination, bool avoidBall = false, bool avoidAllies = true, bool avoidEnemies = true);
    void lookTo(Vector posToLook);
    void send_command(const ctr_msgs::msg::Command &msg);
    InfoBus* infoBus() const {return _ib;}
  public:
    Controller(std::string team, int id, int frequency = 60);
    ~Controller();
};

#endif // CONTROLLER_HPP
