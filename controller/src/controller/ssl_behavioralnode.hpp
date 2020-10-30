#ifndef SSL_CONTROLLER_HPP
#define SSL_CONTROLLER_HPP

#include "behavioralnode.hpp"
#include "state_halt.hpp"
#include "state_gk.hpp"

class SSL_BehavioralNode : public BehavioralNode {
private:
  void updateState(ctr_msgs::msg::State::SharedPtr msg);
  void run();
  void configure();
  std::string name() {return "SSL_BehavioralNode";}
  ctr_msgs::msg::Navigation encodeNavMessage(Vector destination, float orientation,
                                             bool avoidBall, bool avoidAllies, bool avoidEnemies);

public:
  SSL_BehavioralNode(std::string team, int id, int frequency = 60);
  ~SSL_BehavioralNode();
  void nextState(int nextStateName);
};

#endif // SSL_CONTROLLER_HPP
