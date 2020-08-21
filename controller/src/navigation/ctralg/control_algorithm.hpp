#ifndef CONTROL_ALGORITHM_HPP
#define CONTROL_ALGORITHM_HPP

#include <string>
#include <functional>
#include <QMap>
#include <QMutex>
#include <utility>

class Control_Algorithm {
private:
  QMap<std::string, std::pair<double *, double>> _paramTable;
  QMutex _mutex;

protected:
  std::string _name;

public:
  Control_Algorithm(std::string name);
  virtual ~Control_Algorithm();
  virtual double iterate(double error) = 0;
  std::string name() const {return _name;}

  bool declareDoubleParameter(std::string variableName, double *paramAddress, double defaultValue = 0);
  QMap<std::string, std::pair<double *, double>> getParamTable();
};

#endif // CONTROL_ALGORITHM_HPP
