#include "controller.hpp"
#include "../utils/utils.hpp"

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

Controller::Controller(std::string team, int id, int frequency) : rclcpp::Node("controller_"+team+"_"+std::to_string(id)), Entity (frequency) {
  // Internal
  _team = team;
  _id = (qint8) id;

  std::string robotToken = team+"_"+std::to_string(id);

  // Create callback groups for multi-threading execution
  _callback_group_actuator = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_navigation = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_external_agent = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_map = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Initialize ROS 2 interaces
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
  ea_opt.callback_group = _callback_group_external_agent;
  ea_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  ea_opt.topic_stats_options.publish_topic = "external_agent/state/"+robotToken+"/statistics";
  _subExternalAgent = this->create_subscription<ctr_msgs::msg::State>("external_agent/state/"+robotToken,
                                                                      rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_custom_profile.history, rmw_qos_custom_profile.depth), rmw_qos_custom_profile),
                                                                      std::bind(&Controller::updateState, this, std::placeholders::_1),
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

Controller::~Controller() {
  delete _ib;
}

void Controller::send_command(const ctr_msgs::msg::Command &msg) {
  _pubActuator->publish(msg);
}

void Controller::goToLookTo(Vector destination, Vector posToLook, bool avoidBall, bool avoidAllies, bool avoidEnemies) {
  if(destination.isUnknown()) {
    std::cout << "[Controller] Desired destination is unknown!\n";
    return;
  }

  if(posToLook.isUnknown()) {
    std::cout << "[Controller] posToLook is unknown!\n";
    return;
  }

  Vector direction = posToLook-_ib->myPosition();
  float orientation = Utils::wrapToTwoPi(Utils::getAngle(direction));
  _pubNavigation->publish(encodeNavMessage(destination, orientation, avoidBall, avoidAllies, avoidEnemies));
}

void Controller::goTo(Vector destination, bool avoidBall, bool avoidAllies, bool avoidEnemies) {
  if(destination.isUnknown()) {
    std::cout << "[Controller] Desired destination is unknown!\n";
    return;
  }

  float orientation = Utils::wrapToTwoPi(infoBus()->myOrientation());
  _pubNavigation->publish(encodeNavMessage(destination, orientation, avoidBall, avoidAllies, avoidEnemies));
}

void Controller::lookTo(Vector posToLook) {
  if(posToLook.isUnknown()) {
    std::cout << "[Controller] posToLook is unknown!\n";
    return;
  }
  Vector myPos = _ib->myPosition();
  Vector direction = posToLook-myPos;
  float orientation = Utils::wrapToTwoPi(Utils::getAngle(direction));
  _pubNavigation->publish(encodeNavMessage(myPos, orientation, false, false, false));
}

void Controller::nextState(std::string nextStateName) {

}
