#ifndef BEHAVIORALNODE_HPP
#define BEHAVIORALNODE_HPP

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

class BehavioralNode : public rclcpp::Node, public Entity {
  private:
    rclcpp::CallbackGroup::SharedPtr _callback_group_actuator;
    rclcpp::CallbackGroup::SharedPtr _callback_group_states;
    rclcpp::CallbackGroup::SharedPtr _callback_group_map;
    rclcpp::CallbackGroup::SharedPtr _callback_group_navigation;

    rclcpp::Subscription<ctr_msgs::msg::State>::SharedPtr _subExternalAgent;
    rclcpp::Publisher<ctr_msgs::msg::State>::SharedPtr _pubState;
    rclcpp::Publisher<ctr_msgs::msg::Command>::SharedPtr _pubActuator;
    rclcpp::Publisher<ctr_msgs::msg::Navigation>::SharedPtr _pubNavigation;
    rclcpp::Client<ctr_msgs::srv::Elementrequest>::SharedPtr _clientPositionRequest;
    rclcpp::Client<ctr_msgs::srv::Idrequest>::SharedPtr _clientInfoRequest;
    rclcpp::Client<ctr_msgs::srv::Fieldinformationrequest>::SharedPtr _clientFieldRequest;

    QHash<int, rclcpp::Subscription<ctr_msgs::msg::State>::SharedPtr> _subStateTable;

    InfoBus *_ib;
    bool _holdBall;
    float _kickSpeedY;
    float _kickSpeedZ;
    QList<int> _teamIDs;
  protected:
    qint8 _id;
    std::string _team;
    State *_current;
    QHash<qint8, int> _stateTable;

    virtual void updateState(ctr_msgs::msg::State::SharedPtr msg) = 0;
    void hearState(ctr_msgs::msg::State::SharedPtr msg);
    virtual int stringStateToInt(const std::string &state) = 0;

    virtual void run() {return;}
    virtual void configure() {return;}
    virtual std::string name() {return "Controller";}
    virtual ctr_msgs::msg::Navigation encodeNavMessage(Vector destination, float orientation, bool avoidBall, bool avoidAllies, bool avoidEnemies) = 0;
    void send_command(const ctr_msgs::msg::Command &msg);
    void publish_mystate(const ctr_msgs::msg::State &state);
  public:
    BehavioralNode(std::string team, int id,  QList<int> teamIds, int frequency = 60);
    ~BehavioralNode();
    virtual void nextState(int nextStateName) = 0;
    void goToLookTo(Vector destination, Vector posToLook, bool avoidBall = false, bool avoidAllies = true, bool avoidEnemies = true);
    void goTo(Vector destination, bool avoidBall = false, bool avoidAllies = true, bool avoidEnemies = true);
    void lookTo(Vector posToLook);
    void kick(float kickPower = 8.0f);
    void holdBall(bool turnOn = false);
    void chipkick(float kickPower = 8.0f, float kickAngle = 0.0f);
    InfoBus* infoBus() const {return _ib;}
};

#endif // BEHAVIORALNODE_HPP
