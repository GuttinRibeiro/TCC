#ifndef INFO_HPP
#define INFO_HPP

#include <string>
#include <QList>
#include "rclcpp/rclcpp.hpp"
#include "../utils/vector.hpp"
#include "ctr_msgs/srv/elementrequest.hpp"
#include "ctr_msgs/srv/inforequest.hpp"
#include "ctr_msgs/srv/fieldinformationrequest.hpp"

class InfoBus {
private:
  int _group;
  qint8 _id;
  rclcpp::Client<ctr_msgs::srv::Elementrequest>::SharedPtr *_clientPosition;
  rclcpp::Client<ctr_msgs::srv::Inforequest>::SharedPtr *_clientInformation;
  rclcpp::Client<ctr_msgs::srv::Fieldinformationrequest>::SharedPtr *_clientField;

  float requestFieldValue(std::string info_requested) const;
  Vector requestFieldPosition(std::string info_requested) const;
public:
  InfoBus(qint8 id, std::string team, rclcpp::Client<ctr_msgs::srv::Elementrequest>::SharedPtr *clientPosition,
          rclcpp::Client<ctr_msgs::srv::Inforequest>::SharedPtr *clientInformation,
          rclcpp::Client<ctr_msgs::srv::Fieldinformationrequest>::SharedPtr *cliendField);

  // Myself
  Vector myPosition() const;
  float myRadius() const;
  float myOrientation() const;

  // Ball
  Vector ballPosition() const;
  float ballRadius() const;
  float ballOrientation() const;

  // Generic element
  QList<qint8> ourPlayers() const;
  QList<qint8> theirPlayers() const;
  Vector robotPosition(const int &group, const qint8 &id) const;
  float robotRadius(const int &group, const qint8 &id) const;
  float robotOrientation(const int &group, const qint8 &id) const;
  int ourGroup() const;
  int theirGroup() const;

  // Field
  float centerRadius() const;
  float defenseLength() const;
  float defenseWidth() const;
  float fieldLength() const;
  float fieldWidth() const;
  float goalDepth() const;
  float goalWidth() const;
  float minX() const;
  float maxX() const;
  float minY() const;
  float maxY() const;

  Vector ourGoal() const;
  Vector theirGoal() const;
  Vector fieldCenter() const;
  Vector ourGoalLeftPost() const;
  Vector ourGoalRightPost() const;
  Vector theirGoalLeftPost() const;
  Vector theirGoalRightPost() const;
  Vector ourFieldLeftCorner() const;
  Vector ourFieldRightCorner() const;
  Vector theirFieldLeftCorner() const;
  Vector theirFieldRightCorner() const;
};

#endif // INFO_HPP
