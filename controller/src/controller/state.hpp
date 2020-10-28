#ifndef STATE_HPP
#define STATE_HPP

#include "../utils/entity.hpp"
#include "controller.hpp"
#include <string>

class State : public Entity {
private:
  Controller *_ctr;

protected:
  virtual void run() {return;}
  virtual void configure() {return;}
  virtual std::string name() {return "State";}

public:
  virtual std::string nextState() = 0;
  State(Controller *ctr, int frequency = 60);
  void goNext();
};

#endif // STATE_HPP
