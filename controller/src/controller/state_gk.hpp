#ifndef STATE_GK_HPP
#define STATE_GK_HPP

#include "state.hpp"
#include "../utils/states.hpp"

class State_GK : public State {
public:
  State_GK(Controller *ctr);
  ~State_GK();
  int nextState();
  void runState();
};

#endif // STATE_GK_HPP
