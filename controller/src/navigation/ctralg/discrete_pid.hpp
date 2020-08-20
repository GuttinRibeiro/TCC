#ifndef DISCRETE_PID_HPP
#define DISCRETE_PID_HPP

#include "control_algorithm.hpp"
#include <QMutex>

class Discrete_PID : public Control_Algorithm {
private:
  float _q0, _q1, _q2, _T0;
  float _kp, _kd, _ki;
  float _lastError, _lastlastError, _lastOutput;
  QMutex _mutex;

  void convertKtoQ();
public:
  Discrete_PID(rclcpp::Node *owner, std::string name);
  ~Discrete_PID();
  float iterate(float error);
  void getExternalKp();
  void getExternalKd();
  void getExternalKi();
  void getExternalT0();
};

#endif // DISCRETE_PID_HPP
