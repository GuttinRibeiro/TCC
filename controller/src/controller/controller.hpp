#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <QtCore>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ctr_msgs/srv/elementrequest.hpp"
#include "ctr_msgs/srv/idrequest.hpp"
#include "ctr_msgs/srv/fieldinformationrequest.hpp"
#include "ctr_msgs/msg/state.hpp"
#include "ctr_msgs/msg/command.hpp"
#include "ctr_msgs/msg/navigation.hpp"
#include "../utils/field.hpp"
#include "../utils/vector.hpp"
#include "../utils/entity.hpp"
#include "../utils/infobus.hpp"

class State;

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
    rclcpp::Client<ctr_msgs::srv::Idrequest>::SharedPtr _clientInfoRequest;
    rclcpp::Client<ctr_msgs::srv::Fieldinformationrequest>::SharedPtr _clientFieldRequest;

    InfoBus *_ib;
    bool _holdBall;
    float _kickSpeedY;
    float _kickSpeedZ;
  protected:
    qint8 _id;
    std::string _team;
    State *_current;

    virtual void updateState(ctr_msgs::msg::State::SharedPtr msg) = 0;

    virtual void run() {return;}
    virtual void configure() {return;}
    virtual std::string name() {return "Controller";}
    virtual ctr_msgs::msg::Navigation encodeNavMessage(Vector destination, float orientation, bool avoidBall, bool avoidAllies, bool avoidEnemies) = 0;
    void send_command(const ctr_msgs::msg::Command &msg);
  public:
    Controller(std::string team, int id, int frequency = 60);
    ~Controller();
    virtual void nextState(int nextStateName) = 0;
    void goToLookTo(Vector destination, Vector posToLook, bool avoidBall = false, bool avoidAllies = true, bool avoidEnemies = true);
    void goTo(Vector destination, bool avoidBall = false, bool avoidAllies = true, bool avoidEnemies = true);
    void lookTo(Vector posToLook);
    void kick(float kickPower = 8.0f);
    void holdBall(bool turnOn = false);
    void chipkick(float kickPower = 8.0f, float kickAngle = 0.0f);
    InfoBus* infoBus() const {return _ib;}
};

#endif // CONTROLLER_HPP
