#include "discrete_pid.hpp"
//#include <iostream>

Discrete_PID::Discrete_PID(std::string name, int frequency) : Control_Algorithm (name) {
  _kp = 0.0;
  _kd = 0.0;
  _ki = 0.0;
  _T0 = 1.0/frequency;
  _lastError = 0.0;
  _lastlastError = 0.0;
  _lastOutput = 0.0;
  declareDoubleParameter(name+"/kp", &_kp, _kp);
  declareDoubleParameter(name+"/ki", &_ki, _ki);
  declareDoubleParameter(name+"/kd", &_kd, _kd);
  declareDoubleParameter(name+"/T0", &_T0, _T0);
}

Discrete_PID::~Discrete_PID() {

}

double Discrete_PID::iterate(double error) {
  // u(k) = u(k-1) + q0*e(k) + q1*e(k-1) + q2*e(k-2)
  convertKtoQ();
  _lastOutput = _lastOutput + _q0*error + _q1*_lastError + _q2*_lastlastError;
  _lastlastError = _lastError;
  _lastError = _lastOutput;
  return _lastOutput;
}

void Discrete_PID::convertKtoQ() {
  _q0 = _kp+_kd/_T0;
  _q1 = -_kp - 2*(_kd/_T0) + _ki*_T0;
  _q2 = _kd/_T0;
}
