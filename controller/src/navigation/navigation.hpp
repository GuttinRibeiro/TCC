#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <QtCore>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ctr_msgs/srv/elementrequest.hpp"
#include "ctr_msgs/srv/inforequest.hpp"
#include "ctr_msgs/srv/fieldinformationrequest.hpp"
#include "ctr_msgs/action/nav.hpp"
#include "../controller/infobus.hpp"

class Navigation : public rclcpp::Node {
private:
  rclcpp::CallbackGroup::SharedPtr _callback_group_map;
  rclcpp::CallbackGroup::SharedPtr _callback_group_server;
  rclcpp::CallbackGroup::SharedPtr _callback_group_pathFinder;
  rclcpp::CallbackGroup::SharedPtr _callback_group_pathFollower;

  rclcpp::Client<ctr_msgs::srv::Elementrequest>::SharedPtr _clientElementRequest;
  rclcpp::Client<ctr_msgs::srv::Inforequest>::SharedPtr _clientInfoRequest;
  rclcpp::Client<ctr_msgs::srv::Fieldinformationrequest>::SharedPtr _clientFieldRequest;
  rclcpp_action::Server<ctr_msgs::action::Nav>::SharedPtr _serverNavigation;

  InfoBus *_ib;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                          std::shared_ptr<const ctr_msgs::action::Nav::Goal> goal);
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_msgs::action::Nav>> goal_handle);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_msgs::action::Nav>> goal_handle);
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_msgs::action::Nav>> goal_handle);
protected:
  qint8 _id;
  std::string _team;

public:
  Navigation(std::string team, int id);
  ~Navigation();
};

#endif // NAVIGATION_HPP
