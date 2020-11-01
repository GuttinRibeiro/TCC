#include "behavioralnode.hpp"
#include "../utils/utils.hpp"
#define KICK_PRECISION 0.01

using namespace std::chrono_literals;

static const rmw_qos_profile_t rmw_qos_custom_profile {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    100,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

BehavioralNode::BehavioralNode(std::string team, int id, QList<int> teamIds, int frequency) : rclcpp::Node("BehavioralNode_"+team+"_"+std::to_string(id)), Entity (frequency) {
  // Internal
  _team = team;
  _id = static_cast<qint8>(id);
  _holdBall = false;
  _kickSpeedY = 0.0;
  _kickSpeedZ = 0.0;
  _teamIDs = teamIds;

  std::string robotToken = team+"_"+std::to_string(id);

  // Create callback groups for multi-threading execution
  _callback_group_actuator = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_navigation = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_states = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _callback_group_map = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Initialize ROS 2 interfaces
  // My state
  auto state_opt = rclcpp::PublisherOptions();
  state_opt.callback_group = _callback_group_states;
  _pubState = this->create_publisher<ctr_msgs::msg::State>("behavioral_node/"+robotToken+"/mystate",
                                                           rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_custom_profile.history, rmw_qos_custom_profile.depth), rmw_qos_custom_profile),
                                                           state_opt);
  // Subscribe to the state topic of all other robots
  auto hearing_state_opt = rclcpp::SubscriptionOptions();
  hearing_state_opt.callback_group = _callback_group_states;
  for(int id : teamIds) {
    id = static_cast<qint8>(id);
    if(id != _id) {
      std::string token = team+"_"+std::to_string(id);
      rclcpp::Subscription<ctr_msgs::msg::State>::SharedPtr subs = this->create_subscription<ctr_msgs::msg::State>("behavioral_node/"+token+"/mystate",
                                                                                             rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_custom_profile.history, rmw_qos_custom_profile.depth), rmw_qos_custom_profile),
                                                                                             std::bind(&BehavioralNode::hearState, this, std::placeholders::_1),
                                                                                             hearing_state_opt);
      _subStateTable.insert(id, subs);
    }

  }

  // Actuator
  auto act_opt = rclcpp::PublisherOptions();
  act_opt.callback_group = _callback_group_actuator;
  _pubActuator = this->create_publisher<ctr_msgs::msg::Command>("actuator/command/"+robotToken,
                                                                rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                                                                act_opt);

  // Navigation
  auto nav_opt = rclcpp::PublisherOptions();
  nav_opt.callback_group = _callback_group_navigation;
  _pubNavigation = this->create_publisher<ctr_msgs::msg::Navigation>("navigation/motion_specification/"+robotToken,
                                                                     rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                                                                     nav_opt);
  // External Agent
  auto ea_opt = rclcpp::SubscriptionOptions();
  ea_opt.callback_group = _callback_group_states;
  ea_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  ea_opt.topic_stats_options.publish_topic = "external_agent/state/"+robotToken+"/statistics";
  _subExternalAgent = this->create_subscription<ctr_msgs::msg::State>("external_agent/state/"+robotToken,
                                                                      rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_custom_profile.history, rmw_qos_custom_profile.depth), rmw_qos_custom_profile),
                                                                      std::bind(&BehavioralNode::updateState, this, std::placeholders::_1),
                                                                      ea_opt);

  // Map
  _clientPositionRequest = this->create_client<ctr_msgs::srv::Elementrequest>("map_service/"+robotToken+"/position",
                                                                               rmw_qos_profile_services_default,
                                                                               _callback_group_map);

  _clientInfoRequest = this->create_client<ctr_msgs::srv::Idrequest>("map_service/"+robotToken+"/id",
                                                                       rmw_qos_profile_services_default,
                                                                       _callback_group_map);

  _clientFieldRequest = this->create_client<ctr_msgs::srv::Fieldinformationrequest>("map_service/"+robotToken+"/field",
                                                                                    rmw_qos_profile_services_default,
                                                                                    _callback_group_map);

  // Info bus
  _ib = new InfoBus(_id, _team, &_clientPositionRequest, &_clientInfoRequest, &_clientFieldRequest);

  // Start this entity's thread
  this->start();
}

BehavioralNode::~BehavioralNode() {
  delete _ib;
}

void BehavioralNode::send_command(const ctr_msgs::msg::Command &msg) {
  _pubActuator->publish(msg);
}

void BehavioralNode::goToLookTo(Vector destination, Vector posToLook, bool avoidBall, bool avoidAllies, bool avoidEnemies) {
  if(destination.isUnknown()) {
    std::cout << "[BehavioralNode] Desired destination is unknown!\n";
    return;
  }

  if(posToLook.isUnknown()) {
    std::cout << "[BehavioralNode] posToLook is unknown!\n";
    return;
  }

  Vector direction = posToLook-_ib->myPosition();
  float orientation = Utils::wrapToTwoPi(Utils::getAngle(direction));
  _pubNavigation->publish(encodeNavMessage(destination, orientation, avoidBall, avoidAllies, avoidEnemies));
}

void BehavioralNode::goTo(Vector destination, bool avoidBall, bool avoidAllies, bool avoidEnemies) {
  if(destination.isUnknown()) {
    std::cout << "[BehavioralNode] Desired destination is unknown!\n";
    return;
  }

  float orientation = Utils::wrapToTwoPi(infoBus()->myOrientation());
  _pubNavigation->publish(encodeNavMessage(destination, orientation, avoidBall, avoidAllies, avoidEnemies));
}

void BehavioralNode::lookTo(Vector posToLook) {
  if(posToLook.isUnknown()) {
    std::cout << "[BehavioralNode] posToLook is unknown!\n";
    return;
  }
  Vector myPos = _ib->myPosition();
  Vector direction = posToLook-myPos;
  float orientation = Utils::wrapToTwoPi(Utils::getAngle(direction));
  _pubNavigation->publish(encodeNavMessage(myPos, orientation, false, false, false));
}

void BehavioralNode::kick(float kickPower) {
  if(fabs(_kickSpeedY-kickPower) > KICK_PRECISION) {
    _kickSpeedY = kickPower;
    _kickSpeedZ = 0.0;
    ctr_msgs::msg::Command cmd;
    cmd.haskickinformation = true;
    cmd.hasholdballinformation = false;
    cmd.kickspeedz = 0.0;
    cmd.kickspeedy = kickPower;
    cmd.header.stamp = this->get_clock()->now();
    send_command(cmd);
  }
}

void BehavioralNode::chipkick(float kickPower, float kickAngle) {
  if(fabs(_kickSpeedY-kickPower) > KICK_PRECISION || fabs(_kickSpeedZ-kickPower) > KICK_PRECISION) {
    _kickSpeedY = kickPower*cos(kickAngle);
    _kickSpeedZ = kickPower*sin(kickAngle);
    ctr_msgs::msg::Command cmd;
    cmd.haskickinformation = true;
    cmd.hasholdballinformation = false;
    cmd.kickspeedz = _kickSpeedZ;
    cmd.kickspeedy = _kickSpeedY;
    cmd.header.stamp = this->get_clock()->now();
    send_command(cmd);
  }
}

void BehavioralNode::holdBall(bool turnOn) {
  if(turnOn != _holdBall) {
    _holdBall = turnOn;
    ctr_msgs::msg::Command cmd;
    cmd.haskickinformation = false;
    cmd.hasholdballinformation = true;
    cmd.holdball = turnOn;
    cmd.header.stamp = this->get_clock()->now();
    send_command(cmd);
  }
}

void BehavioralNode::publish_mystate(const ctr_msgs::msg::State &msg) {
  _pubState->publish(msg);
}

void BehavioralNode::hearState(ctr_msgs::msg::State::SharedPtr msg) {
  int state = stringStateToInt(msg->state);
  _stateTable.insert(msg->id, state);
}
