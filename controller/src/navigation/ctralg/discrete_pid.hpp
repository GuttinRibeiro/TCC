#ifndef DISCRETE_PID_HPP
#define DISCRETE_PID_HPP

#include "control_algorithm.hpp"
#include <QMutex>

class Discrete_PID : public Control_Algorithm {
private:
  double _q0, _q1, _q2, _T0;
  double _kp, _kd, _ki;
  double _lastError, _lastlastError, _lastOutput;

  void convertKtoQ();
public:
  Discrete_PID(std::string name, int frequency = 60);
  ~Discrete_PID();
  double iterate(double error);
};

#endif // DISCRETE_PID_HPP
