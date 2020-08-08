#include "navigation.hpp"

Navigation::Navigation(std::string team, int id) : Node("navigation_"+team+"_"+std::to_string(id)){
  _team = team;
  _id = (qint8)id;
  std::string robotToken = team+"_"+std::to_string(id);

  // Create callback groups for multi-threading execution
  _callback_group_map = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_server = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_pathFinder = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _callback_group_pathFollower = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Map
  _clientElementRequest = this->create_client<ctr_msgs::srv::Elementrequest>("map_service/"+robotToken+"/position",
                                                                               rmw_qos_profile_services_default,
                                                                               _callback_group_map);

  _clientInfoRequest = this->create_client<ctr_msgs::srv::Inforequest>("map_service/"+robotToken+"/info",
                                                                       rmw_qos_profile_services_default,
                                                                       _callback_group_map);

  _clientFieldRequest = this->create_client<ctr_msgs::srv::Fieldinformationrequest>("map_service/"+robotToken+"/field",
                                                                                    rmw_qos_profile_services_default,
                                                                                    _callback_group_map);

  // Info bus
  _ib = new InfoBus(_id, _team, &_clientElementRequest, &_clientInfoRequest, &_clientFieldRequest);

  // Navigation action server
  _serverNavigation = rclcpp_action::create_server<ctr_msgs::action::Nav>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(), this->get_node_waitables_interface(),
        "/nav"+robotToken, std::bind(&Navigation::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Navigation::handle_cancel, this, std::placeholders::_1),
        std::bind(&Navigation::handle_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(), _callback_group_server);
}

Navigation::~Navigation() {
  delete _ib;
}

rclcpp_action::GoalResponse Navigation::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ctr_msgs::action::Nav::Goal> goal) {
  (void)uuid;
  if(goal->destination.isvalid == false) {
    RCLCPP_INFO(this->get_logger(), "[Navigation] Rejecting goal request: destination received is not a valid position");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // TODO: tratar posição e orientação recebidos
  return  rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void Navigation::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_msgs::action::Nav> > goal_handle) {
  std::thread{std::bind(&Navigation::execute, this, std::placeholders::_1), goal_handle}.detach();
}

rclcpp_action::CancelResponse Navigation::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_msgs::action::Nav>> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "[Navigation] A new final state was received. Last goal cancelled");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
