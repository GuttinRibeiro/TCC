#ifndef SSL_CONTROLLER_HPP
#define SSL_CONTROLLER_HPP

#include "controller.hpp"

class SSL_Controller : public Controller {
private:
  bool _holdBall;
  float _kickSpeedY;
  float _kickSpeedZ;

  void updateState(const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<ctr_msgs::srv::State::Request> request,
                   const std::shared_ptr<ctr_msgs::srv::State::Response> response);
  void run();
  void configure();
  std::string name() {return "SSL_Controller";}
  ctr_msgs::msg::Navigation encodeNavMessage(Vector destination, float orientation);
  void kick(float kickPower = 8.0f);
  void holdBall(bool turnOn = false);
  void chipkick(float kickPower = 8.0f, float kickAngle = 0.0f);
public:
  SSL_Controller(std::string team, int id, int frequency = 60);
  ~SSL_Controller();
};

#endif // SSL_CONTROLLER_HPP
