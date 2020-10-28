#ifndef SSL_CONTROLLER_HPP
#define SSL_CONTROLLER_HPP

#include "controller.hpp"
#include "state_halt.hpp"

class SSL_Controller : public Controller {
private:
  void updateState(ctr_msgs::msg::State::SharedPtr msg);
  void run();
  void configure();
  std::string name() {return "SSL_Controller";}
  ctr_msgs::msg::Navigation encodeNavMessage(Vector destination, float orientation,
                                             bool avoidBall, bool avoidAllies, bool avoidEnemies);

public:
  SSL_Controller(std::string team, int id, int frequency = 60);
  ~SSL_Controller();
  void nextState(int nextStateName);
};

#endif // SSL_CONTROLLER_HPP
