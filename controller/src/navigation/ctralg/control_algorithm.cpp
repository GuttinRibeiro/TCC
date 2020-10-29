#include "control_algorithm.hpp"
#include <iostream>
#include <chrono>

Control_Algorithm::Control_Algorithm(std::string name) {
  _name = name;
//  _mutex.unlock();
}

Control_Algorithm::~Control_Algorithm() {
  _paramTable.clear();
}

bool Control_Algorithm::declareDoubleParameter(std::string variableName, double *paramAddress, double defaultValue) {
  if(paramAddress == nullptr) {
    return false;
  }
//  _mutex.lock();
  _paramTable.insert(variableName, std::make_pair(paramAddress, defaultValue));
//  _mutex.unlock();
  return true;
}

QMap<std::string, std::pair<double *, double> > Control_Algorithm::getParamTable() {
//  _mutex.lock();
  QMap<std::string, std::pair<double *, double>> ret = _paramTable;
//  _mutex.unlock();
  return ret;
}
