#ifndef STATE_HPP
#define STATE_HPP

#include "controller.hpp"
#include <string>

class State {
private:
  Controller *_ctr;

protected:
  Controller * ctrAccess() const {return _ctr;}
public:
  virtual int nextState() = 0;
  State(Controller *ctr);
  virtual ~State();
  void goNext();
  virtual void runState() = 0;
};

#endif // STATE_HPP
