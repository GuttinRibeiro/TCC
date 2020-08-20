#include "discrete_pid.hpp"

Discrete_PID::Discrete_PID(rclcpp::Node *owner, std::string name) : Control_Algorithm (owner, name) {
  _kp = 0.0;
  _kd = 0.0;
  _ki = 0.0;
  _T0 = 1/60;
  _lastError = 0.0;
  _lastlastError = 0.0;
  _lastOutput = 0.0;
  _mutex.unlock();
  declareFloatParameter("kp", _kp, std::bind(&Discrete_PID::getExternalKp, this));
  declareFloatParameter("kd", _kd, std::bind(&Discrete_PID::getExternalKd, this));
  declareFloatParameter("ki", _ki, std::bind(&Discrete_PID::getExternalKi, this));
  declareFloatParameter("T0", _T0, std::bind(&Discrete_PID::getExternalT0, this));
}

Discrete_PID::~Discrete_PID() {

}

float Discrete_PID::iterate(float error) {
  // u(k) = u(k-1) + q0*e(k) + q1*e(k-1) + q2*e(k-2)
  _mutex.lock();
  _lastOutput = _lastOutput + _q0*error + _q1*_lastError + _q2*_lastlastError;
  _mutex.unlock();
  _lastlastError = _lastError;
  _lastError = error;
  return _lastOutput;
}

void Discrete_PID::getExternalKp() {
  _mutex.lock();
  getFloatParameter("kp", &_kp);
  convertKtoQ();
  _mutex.unlock();
}

void Discrete_PID::getExternalKd() {
  std::cout << "Function called\n";
  _mutex.lock();
  getFloatParameter("kd", &_kd);
  std::cout << "[PID] New kd " << _kd << "\n";
  convertKtoQ();
  _mutex.unlock();
}

void Discrete_PID::getExternalKi() {
  _mutex.lock();
  getFloatParameter("ki", &_ki);
  convertKtoQ();
  _mutex.unlock();
}

void Discrete_PID::getExternalT0() {
  _mutex.lock();
  getFloatParameter("T0", &_T0);
  convertKtoQ();
  _mutex.unlock();
}

void Discrete_PID::convertKtoQ() {
  _q0 = _kp+_kd/_T0+_ki*_T0;
  _q1 = -_kp - 2*(_kd/_T0);
  _q2 = _kd/_T0;
}
